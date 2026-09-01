"""FastAPI web UI for IMX678 on the Qualcomm IQ9 (QCS9075).

Structure A (shared monorepo): reuses the portable science modules from
../metro_server/webui (mtf/mtf_analyze/imaging/history) with an IQ9-specific
capture backend (camera_qmmf -> qtiqmmfsrc NV12). NV12 is the ISP-processed path,
so this delivers: live view, MTF focus assist, histogram. Linear-RAW features
(derived-CCM colour, dark-integrity, HDR) await the CamX RAW/RDI usecase + CHI-CDK.

qtiqmmfsrc can't be instantiated twice per process, so ONE persistent capture is
opened for the server's lifetime; frame pulls are serialised with a lock.

Run:  IQ9_W=1920 IQ9_H=1080 IQ9_FPS=30 IQ9_CAM=0 PORT=8080 python3 server.py
"""
import ctypes
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import cv2
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.concurrency import run_in_threadpool

HERE = os.path.dirname(os.path.abspath(__file__))
# Approach A: find the portable modules (deployed alongside as _shared/, or in the repo tree).
for _p in (HERE, os.path.join(HERE, "_shared"),
           os.path.join(HERE, "..", "metro_server", "webui")):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)

import camera_qmmf            # noqa: E402  (IQ9 capture backend)
import raw_tools              # noqa: E402  (IQ9 RAW Bayer characterization + preview)
import mtf_analyze            # noqa: E402  (shared: slanted-edge MTF, analyze_gray path)
try:
    import colorchecker       # noqa: E402  (shared: detect + CIEDE2000; analyze / analyze_processed)
    _HAS_CC = True
except Exception:
    _HAS_CC = False
try:
    import raw_ptc            # noqa: E402  (RAW PTC/OETF/SNR characterization)
    _HAS_PTC = True
except Exception:
    _HAS_PTC = False
try:
    import history            # noqa: E402  (shared: HDF5 session save)
    _HAS_HISTORY = True
except Exception:
    _HAS_HISTORY = False

# static dir: iq9_server/static preferred, else the shared metro pages
STATIC = next((c for c in (os.path.join(HERE, "static"),
                           os.path.join(HERE, "..", "metro_server", "webui", "static"),
                           os.path.join(HERE, "_shared", "static"))
               if os.path.isdir(c)), os.path.join(HERE, "static"))

# PC-side DMX agent (lab lights are on the PC's USB; the board proxies over the direct link)
DMX_AGENT_URL = os.environ.get("DMX_AGENT_URL", "http://192.168.99.1:9200")

W = int(os.environ.get("IQ9_W", "1920"))
H = int(os.environ.get("IQ9_H", "1080"))
FPS = int(os.environ.get("IQ9_FPS", "30"))
CAM = int(os.environ.get("IQ9_CAM", "0"))
RAW_DIR = os.environ.get("IQ9_RAW_DIR", os.path.join(HERE, "raw_captures"))
# Quiesce (seconds) after fully killing the NV12 worker before starting an RDI capture, to let
# cam-server complete the IFE/RDI release. 3.0 gave 27/27 clean; a short value is the causality test.
RAW_QUIESCE_S = float(os.environ.get("IQ9_RAW_QUIESCE_S", "3.0"))
# RAW16 capture is PROVEN (verified 12-bit RGGB) but INTERMITTENTLY hangs the camera
# subsystem on this firmware -> hard watchdog reboot (no kernel panic logged), seen both
# with the NV12->RAW handoff and on an idle camera. Gate it OFF by default so a UI click
# can't reboot a shared device; enable only for supervised testing:  IQ9_RAW_ENABLE=1
RAW_ENABLE = os.environ.get("IQ9_RAW_ENABLE", "0") == "1"
RAW_DISABLED_MSG = ("RAW capture is disabled (IQ9_RAW_ENABLE=0). RAW16 works and yields "
                    "verified 12-bit RGGB, but on this firmware it intermittently hangs the "
                    "camera subsystem and hard-reboots the board. Enable only for supervised "
                    "testing (ideally with a serial console / hvo watching).")

app = FastAPI(title="IMX678 on IQ9 — Web UI")

# NV12 live view runs in a KILLABLE child process (camera_worker.py) that owns qtiqmmfsrc
# and publishes BGR frames to POSIX shm. An in-process Gst NULL does NOT fully disconnect
# the cam-server recorder client, so the camera stays claimed and RAW capture gets no frame;
# fully KILLING the worker is the only reliable path to release it for RAW. The server kills
# the worker for the RAW window, then respawns it. See docs/qualcomm-raw-dlkm-stability.md.
_worker = None
_cam_lock = threading.Lock()
_raw_mode = False                                   # cold RAW mode: worker killed, camera idle
# Setup-safe camera lock: when True, reject raw-mode toggles and RAW grabs (the camera
# reconfigures that wedge the 2.0 cam-server). NV12 live view + fieldmap work fine while locked.
# Unlock via POST /api/cam_lock {"locked": false} for RAW characterization (PTC/SNR1s/CCM/DCG).
_cam_locked = os.environ.get("IQ9_CAM_LOCK", "1") not in ("0", "false", "False")
_exposure_comp = None                               # last-requested exposure-compensation (cached)
SHM = os.environ.get("IQ9_SHM", "/dev/shm/iq9_nv12")
CTL = SHM + ".ctl"
_MAGIC = b"IQ9N"
_HDR = len(_MAGIC) + 12                              # magic + u32 width,height,seq
# RAW16 shm published by the dual-pad daemon (camera_worker) -- second pad of the SAME camera
# session, so RAW no longer needs to kill/reopen the camera (the fix for the 2.0 CCI wedge).
RAW_SHM = os.environ.get("IQ9_RAW_SHM", "/dev/shm/iq9_raw")
RAW_ON = RAW_SHM + ".on"                             # touch => daemon publishes RAW; remove => stop
_RAW_MAGIC = b"IQ9R"

# Per-frame provenance the daemon publishes alongside each frame: a JSON sidecar (sensor timestamp +
# decoded CamX 'actual' values) and the raw serialized camera_metadata_t (webui re-parses it
# authoritatively via cam_meta). See docs/daq-camera-streaming-metadata.md.
NV_META = SHM + ".meta"
RAW_META = RAW_SHM + ".meta"
CAM_BIN = "/dev/shm/iq9_cammeta.bin"
# Requested sensor config (what the daemon COMMANDS via its env; None => 3A auto). The daemon
# (iq9cam.service) sets these; mirror them here for the requested-vs-actual record.
REQ_EXPOSURE_NS = os.environ.get("IQ9_EXP_NS")
REQ_ISO = os.environ.get("IQ9_ISO")
# Vendor ISP tuning in effect for the NV12/ISP product (factory Chromatix). See memory
# imx678-iq9-chromatix-tuning: Scenario.Default / IPE / cc13_ipe_v2.xml.
TUNING_SCENARIO = "Chromatix Default (IPE cc13_ipe_v2)"

try:
    import cam_meta                                   # pure-Python camera_metadata_t parser
except Exception:
    cam_meta = None


def _pdeathsig():
    """Child preexec (Linux): die if the server process dies, so no orphan holds the camera."""
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGKILL)   # PR_SET_PDEATHSIG
    except Exception:
        pass


def _worker_alive():
    """The dual-pad camera daemon runs as the standalone iq9cam systemd service; treat it as alive
    when its NV12 shm is fresh (it publishes continuously at ~30 fps)."""
    try:
        return (time.time() - os.path.getmtime(SHM)) < 5.0
    except OSError:
        return False


def _start_worker():
    """No-op. The dual-pad daemon (camera_worker.py) runs as the standalone iq9cam.service and owns
    the camera from boot; the webui is a pure shm consumer + control-file writer and NEVER opens or
    kills the camera (opening it here would be the single-client collision that wedges the 2.0 CCI)."""
    return


def _set_resolution(w, h):
    """NV12 resolution is fixed at daemon boot (iq9cam.service env IQ9_W/IQ9_H). Changing it needs a
    unit edit + reboot (a daemon restart re-wedges the 2.0 camera). Runtime no-op."""
    return


def _kill_worker():
    """No-op -- see _start_worker. The daemon is a systemd service, not a webui child, so the webui
    never kills it (a kill+respawn is the open-after-close that wedges the 2.0 camera)."""
    return


def _write_ctl(exposure_comp):
    try:
        import json
        with open(CTL, "w") as f:
            json.dump({"exposure_compensation": int(exposure_comp)}, f)
    except Exception:
        pass


def _read_shm(timeout_s=5.0):
    """Latest BGR frame from the worker's shm, or None. Waits up to timeout for a fresh one."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            with open(SHM, "rb") as f:
                buf = f.read()
            if len(buf) >= _HDR and buf[:4] == _MAGIC:
                w, h, _seq = struct.unpack("<III", buf[4:_HDR])
                need = _HDR + w * h * 3
                if len(buf) >= need:
                    return (np.frombuffer(buf, np.uint8, count=w * h * 3, offset=_HDR)
                            .reshape(h, w, 3).copy())
        except (FileNotFoundError, ValueError):
            pass
        except Exception:
            pass
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.05)


def _set_raw_mode(on):
    """OBSOLETE with the dual-pad daemon: NV12 and RAW16 share ONE camera session, so there is no
    NV12<->RAW handoff to toggle. Kept as a hard no-op (camera stays NV12-live; RAW pulled on demand
    via the shm flag) so an old /raw or /ptc client can't kill the daemon. _raw_mode stays False."""
    return False


def _frame(timeout_s=5.0):
    """Latest BGR frame from the NV12 worker (via shm). None while cold."""
    with _cam_lock:
        if _raw_mode:
            return None
        if not _worker_alive():
            _start_worker()
    return _read_shm(timeout_s=timeout_s)           # lock-free read (may wait for first frame)


def _read_raw_shm(prev_seq, timeout_s=6.0):
    """Latest RAW16 frame (H,W uint16) from the daemon's shm with seq > prev_seq (a NEW frame),
    else (None, prev_seq). Deep-copies out of the mmap'd file."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            with open(RAW_SHM, "rb") as f:
                buf = f.read()
            if len(buf) >= _HDR and buf[:4] == _RAW_MAGIC:
                w, h, seq = struct.unpack("<III", buf[4:_HDR])
                need = _HDR + w * h * 2
                if len(buf) >= need and seq > prev_seq:
                    a = np.frombuffer(buf, dtype="<u2", count=w * h, offset=_HDR).reshape(h, w)
                    return a.astype(np.uint16), seq
        except (FileNotFoundError, ValueError):
            pass
        except Exception:
            pass
        if time.monotonic() >= deadline:
            return None, prev_seq
        time.sleep(0.02)


def _grab_raw(n_frames=1, width=None, height=None, shdr=False):
    """Capture n RAW16 Bayer frames from the persistent DUAL-PAD daemon via shm. The daemon holds
    NV12 + bayer on ONE camera session, so RAW no longer kills/reopens the camera -- the 2.0
    open-after-close CCI wedge can't trigger. Touch the raw-publish flag so the daemon starts
    emitting RAW, read n distinct (consecutive) frames, then clear it. width/height/shdr are not
    supported on this path (the daemon serves the standard RGGB geometry); use the offline
    build_clearhdr flow for DCG/SHDR."""
    if not _worker_alive():
        with _cam_lock:
            _start_worker()
    frames = []
    try:
        open(RAW_ON, "w").close()                    # ask the daemon to publish RAW frames
        seq = 0
        for _ in range(max(1, int(n_frames))):
            a, seq = _read_raw_shm(seq, timeout_s=6.0)
            if a is None:
                break
            frames.append(a)
    finally:
        try:
            os.remove(RAW_ON)
        except OSError:
            pass
    if not frames:
        raise RuntimeError("no RAW frame from camera daemon (is camera_worker the dual-pad build?)")
    meta = {"width": int(frames[0].shape[1]), "height": int(frames[0].shape[0]),
            "frames": len(frames), "source": "dual-pad-daemon"}
    return frames, meta


@app.on_event("shutdown")
def _shutdown():
    with _cam_lock:
        _kill_worker()


def _channel(bgr, chan):
    if chan == "R":
        return bgr[:, :, 2]
    if chan == "B":
        return bgr[:, :, 0]
    return bgr[:, :, 1]                        # green as luma proxy ('Y'/'G')


def _page(name):
    p = os.path.join(STATIC, name)
    if not os.path.exists(p):
        return HTMLResponse("<h3>%s not deployed</h3>" % name, status_code=404)
    with open(p, encoding="utf-8") as f:
        return HTMLResponse(f.read())


def _resize(bgr, width):
    if width and bgr.shape[1] > width:
        return cv2.resize(bgr, (width, max(1, int(bgr.shape[0] * width / bgr.shape[1]))),
                          interpolation=cv2.INTER_AREA)
    return bgr


# ── pages ───────────────────────────────────────────────────────────────────
@app.get("/", response_class=HTMLResponse)
def index():
    return _page("index.html")


@app.get("/mtf", response_class=HTMLResponse)
def mtf_page():
    return _page("mtf.html")


@app.get("/colorchecker", response_class=HTMLResponse)
def cc_page():
    return _page("colorchecker.html")


@app.get("/history", response_class=HTMLResponse)
def history_page():
    return _page("history.html")


@app.get("/raw", response_class=HTMLResponse)
def raw_page():
    return _page("raw.html")


@app.get("/ptc", response_class=HTMLResponse)
def ptc_page():
    return _page("ptc.html")


# ── camera info / params ─────────────────────────────────────────────────────
@app.get("/api/info")
def api_info():
    exp_comp = _exposure_comp
    return {
        "platform": "IQ9 QCS9075", "source": "nv12-isp (qtiqmmfsrc)",
        "camera": CAM, "width": W, "height": H, "fps": FPS,
        "bit_depth": 8, "sensormode": 0, "exposure_ns": 0, "gain": 0,
        "exposure_compensation": exp_comp,
        "raw_available": True,
        "raw_enabled": RAW_ENABLE,
        "raw_mode": _raw_mode,
        "live_view": (not _raw_mode),
        "raw": {"width": camera_qmmf.RAW_W, "height": camera_qmmf.RAW_H,
                "bit_depth": 12, "cfa": "RGGB", "format": "RAW16 (bpp=16)",
                "caution": None if RAW_ENABLE else RAW_DISABLED_MSG},
        "note": "ISP-processed NV12 live; native 12-bit RAW16 via /api/raw/capture"
                + ("" if RAW_ENABLE else " (gated off — see raw.caution)"),
    }


def _read_json_sidecar(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def _read_cammeta_bin():
    """Authoritatively re-parse the latest raw camera_metadata_t on the webui side (so parser
    fixes need no daemon reboot). Returns the decoded 'actual' dict, or None."""
    if cam_meta is None:
        return None
    try:
        with open(CAM_BIN, "rb") as f:
            raw = f.read()
        return cam_meta.decode(cam_meta.parse(raw))
    except Exception:
        return None


def _isp_tasks(product, act):
    """ISP task provenance. RAW16 = sensor data (everything off). NV12 = the CamX vendor pipeline;
    each task's source is the factory Chromatix tuning unless a user override is applied. `act` is
    the decoded CamX actual dict (may be empty)."""
    act = act or {}
    if product == "raw16":
        return [{"task": t, "applied": False, "source": "off"} for t in (
            "DPC", "BLC", "LSC", "CAC", "demosaic", "AWB", "CCM", "GTM", "LTM", "gamma_OETF",
            "sharpen", "NR", "WDR_DRC", "HDR_fusion", "binning_scaling", "EIS")]
    ccm = act.get("ccm")
    return [
        {"task": "DPC", "applied": True, "source": "vendor-default"},
        {"task": "BLC", "applied": True, "source": "vendor-default",
         "params": {"dynamic_black_level": act.get("dynamic_black_level"),
                    "white_level": act.get("dynamic_white_level")}},
        {"task": "LSC", "applied": True, "source": "vendor-default"},
        {"task": "CAC", "applied": True, "source": "vendor-default"},
        {"task": "demosaic", "applied": True, "source": "vendor-default"},
        {"task": "AWB", "applied": True, "source": "vendor-default",
         "params": {"gains": act.get("awb_gains")}},
        {"task": "CCM", "applied": ccm is not None, "source": "vendor-default",
         "params": {"illuminant": "auto (scene-selected)", "matrix": ccm,
                    "traceability": "CamX Chromatix " + TUNING_SCENARIO
                    + "; a user custom CCM applies only in the offline cc_analyze path, not this live stream"}},
        {"task": "GTM", "applied": True, "source": "vendor-default"},
        {"task": "LTM", "applied": True, "source": "vendor-default"},
        {"task": "gamma_OETF", "applied": True, "source": "vendor-default"},
        {"task": "sharpen", "applied": True, "source": "vendor-default"},
        {"task": "NR", "applied": True, "source": "vendor-default"},
        {"task": "WDR_DRC", "applied": False, "source": "off"},
        {"task": "HDR_fusion", "applied": False, "source": "off",
         "note": "linear NV12 live (no SHDR/DOL)"},
        {"task": "binning_scaling", "applied": True, "source": "vendor-default",
         "params": {"note": "ISP downscale to the output resolution"}},
        {"task": "EIS", "applied": False, "source": "off"},
    ]


def _compose_frame_meta(product="nv12"):
    """Full per-frame provenance record: requested (what the daemon commands) + actual (CamX result
    metadata) + ISP task table + CCM traceability + lens/focus + product. See the design doc."""
    side = _read_json_sidecar(NV_META if product == "nv12" else RAW_META) or {}
    act = side.get("cam") if side.get("cam", {}).get("_available") else None
    if act is None:
        act = _read_cammeta_bin()                    # fallback: re-parse the latest raw blob
    act = act or {}
    if product == "raw16":
        prod = {"type": "RAW16", "resolution": "%dx%d" % (camera_qmmf.RAW_W, camera_qmmf.RAW_H_ACTIVE),
                "pixfmt": "RGGB bayer", "bit_depth": 12}
    else:
        prod = {"type": "NV12-ISP", "resolution": "%dx%d" % (W, H),
                "pixfmt": "NV12 (delivered BGR)", "bit_depth": 8}
    req_exp = int(REQ_EXPOSURE_NS) if REQ_EXPOSURE_NS else None
    req_iso = int(REQ_ISO) if REQ_ISO else None
    fps_act = round(1e9 / act["frame_duration_ns"], 3) if act.get("frame_duration_ns") else None
    return {
        "product": prod,
        "timing": {
            "frame_seq": side.get("seq"),
            "pts_ns": side.get("pts_ns"),
            "sensor_timestamp_ns": act.get("sensor_timestamp_ns"),
            "host_recv_ns": side.get("host_ns"),
            "host_epoch_ns": side.get("host_epoch_ns"),
            "frame_count": act.get("frame_count"),
        },
        "sensor": {
            "bit_depth": prod["bit_depth"],
            "exposure_ns": {"requested": req_exp if req_exp is not None else "auto(3A)",
                            "actual": act.get("exposure_ns")},
            "gain_iso": {"requested": req_iso if req_iso is not None else "auto(3A)",
                         "actual": act.get("iso")},
            "conv_gain": {"requested": "n/a (ISP path)", "actual": "not in NV12 result-meta"},
            "fps": {"requested": FPS, "actual": fps_act},
            "roi": {"requested": "full", "actual": act.get("crop_region")},
            "hdr_mode": {"requested": "linear", "actual": "linear"},
            "black_level": {"actual": act.get("dynamic_black_level")},
            "white_level": {"actual": act.get("dynamic_white_level")},
            "rolling_shutter_skew_ns": act.get("rolling_shutter_skew_ns"),
        },
        "lens_focus": {
            "type": "fixed", "focus_pos": {"requested": None, "actual": None},
            "focus_distance_m": None, "calib_id": "fixed-lens-0",
            "intrinsics": None, "distortion": {"radial": None, "tangential": None},
            "note": "fixed lens => single calibration entry. Schema ready for a liquid lens (driver "
                    "on the IQ9): a per-focus-position calib table keyed focus_pos -> calib_id, each "
                    "with its own radial/tangential distortion for LDC/CAC to consume per frame.",
        },
        "isp": _isp_tasks(product, act),
        "provenance": {
            "tuning_scenario": TUNING_SCENARIO,
            "pipeline": "CamX (qtiqmmfsrc)",
            "metadata_source": "CamX result-metadata (camera_metadata_t via qmmf::CameraMetadata::getbuffer)",
            "awb_gains_actual": act.get("awb_gains"),
            "control_mode": act.get("control_mode"),
            "capture_intent": act.get("capture_intent"),
        },
        "_meta_available": bool(act.get("_available")) or bool(act.get("exposure_ns")),
        "_notes": "requested = commanded by the daemon (env/API); actual = CamX per-frame result "
                  "metadata. RAW16 product => all ISP tasks off (raw sensor data).",
    }


@app.get("/api/frame_meta")
def api_frame_meta(product: str = "nv12"):
    """Per-frame provenance metadata (requested vs actual, ISP task table, CCM traceability,
    lens/focus, product). product=nv12 (default) or raw16."""
    p = "raw16" if str(product).lower() in ("raw", "raw16", "bayer") else "nv12"
    return _compose_frame_meta(p)


@app.post("/api/params")
async def api_params(request: Request):
    """Best-effort live control on the running qtiqmmfsrc. NV12 is 3A-auto; we can
    nudge exposure-compensation. ns-exposure/gain (raw-path controls) don't map here."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    global _exposure_comp
    if _raw_mode:
        return JSONResponse({"error": "camera is in cold RAW mode; exit RAW mode for NV12 controls"},
                            status_code=409)
    applied = {}
    if "exposure_compensation" in body:
        _exposure_comp = max(-12, min(12, int(body["exposure_compensation"])))
        _write_ctl(_exposure_comp)                  # NV12 worker polls the control file and applies it
        applied["exposure_compensation"] = True
    if "width" in body and "height" in body:
        w2 = max(320, min(3856, int(body["width"])))
        h2 = max(240, min(2180, int(body["height"])))
        if (w2, h2) != (W, H):
            await run_in_threadpool(_set_resolution, w2, h2)   # ISP downscale; restarts the worker
        applied["resolution"] = "%dx%d" % (w2, h2)
    info = api_info()
    info["applied"] = applied
    return info


# ── lighting (PC-side DMX agent, proxied over the direct link) ───────────────
@app.get("/api/dmx")
async def api_dmx_get():
    """Current illuminant levels from the PC-side DMX agent (d65/tungsten, 0-255)."""
    def work():
        import json as _j
        import urllib.request
        try:
            with urllib.request.urlopen(DMX_AGENT_URL + "/dmx", timeout=5) as r:
                return _j.loads(r.read())
        except Exception as e:
            return {"error": "DMX agent unreachable (%s): %s" % (DMX_AGENT_URL, e)}
    return await run_in_threadpool(work)


@app.get("/api/dmx/status")
async def api_dmx_status():
    """Current DMX agent URL and whether it's reachable right now."""
    def work():
        import json as _j
        import urllib.request
        try:
            with urllib.request.urlopen(DMX_AGENT_URL + "/dmx", timeout=4) as r:
                return {"url": DMX_AGENT_URL, "connected": True, "levels": _j.loads(r.read())}
        except Exception as e:
            return {"url": DMX_AGENT_URL, "connected": False, "error": str(e)}
    return await run_in_threadpool(work)


@app.post("/api/dmx/connect")
async def api_dmx_connect(request: Request):
    """(Re)point the webui at the PC-side DMX agent and test it. With no host/url in the body,
    uses the requesting client's IP -- the browser runs on the PC that hosts the agent, so this
    works over whatever network is currently up (LAN/WiFi), not just the dead direct link.
    Latches the new URL only if the agent answers, so a bad address can't break a working one."""
    global DMX_AGENT_URL
    try:
        body = await request.json()
    except Exception:
        body = {}
    url = str(body.get("url", "")).strip()
    if not url:
        host = str(body.get("host", "")).strip() or (request.client.host if request.client else "127.0.0.1")
        port = int(body.get("port", 9200))
        url = "http://%s:%d" % (host, port)
    if not url.startswith("http"):
        url = "http://" + url
    url = url.rstrip("/")

    def work():
        import json as _j
        import urllib.request
        try:
            with urllib.request.urlopen(url + "/dmx", timeout=5) as r:
                return {"connected": True, "url": url, "levels": _j.loads(r.read())}
        except Exception as e:
            return {"connected": False, "url": url, "error": str(e)}
    res = await run_in_threadpool(work)
    if res.get("connected"):
        DMX_AGENT_URL = url          # latch only on success
    return res


@app.post("/api/dmx")
async def api_dmx_set(request: Request):
    """Set illuminant levels via the PC-side DMX agent (d65/tungsten, 0-255)."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    fwd = {}
    for k in ("d65", "tungsten"):
        if k in body:
            fwd[k] = max(0, min(255, int(body[k])))

    def work():
        import json as _j
        import urllib.error
        import urllib.request
        req = urllib.request.Request(DMX_AGENT_URL + "/dmx", data=_j.dumps(fwd).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=12) as r:
                return _j.loads(r.read())
        except urllib.error.HTTPError as e:
            try:
                return _j.loads(e.read())
            except Exception:
                return {"error": "DMX agent HTTP %d" % e.code}
        except Exception as e:
            return {"error": "DMX agent unreachable (%s): %s" % (DMX_AGENT_URL, e)}
    return await run_in_threadpool(work)


# ── frames ───────────────────────────────────────────────────────────────────
@app.get("/api/frame.jpg")
def api_frame(width: int = 960):
    bgr = _frame()
    if bgr is None:
        return JSONResponse({"error": "no frame"}, status_code=502)
    ok, buf = cv2.imencode(".jpg", _resize(bgr, width), [cv2.IMWRITE_JPEG_QUALITY, 80])
    return Response(content=buf.tobytes(), media_type="image/jpeg")


def _mjpeg(width):
    boundary = b"--frame"
    while True:
        bgr = _frame()
        if bgr is None:
            break
        ok, buf = cv2.imencode(".jpg", _resize(bgr, width), [cv2.IMWRITE_JPEG_QUALITY, 80])
        jpg = buf.tobytes()
        yield (boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
               + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.03)


@app.get("/stream.mjpg")
def stream(width: int = 960):
    return StreamingResponse(_mjpeg(width),
                             media_type="multipart/x-mixed-replace; boundary=frame")


# ── RAW (native 12-bit RGGB Bayer via qtiqmmfsrc RAW16) ──────────────────────
@app.post("/api/raw/mode")
async def api_raw_mode(request: Request):
    """OBSOLETE with the dual-pad daemon. NV12 and RAW16 share ONE camera session, so there is no
    cold 'raw mode' to enter -- RAW capture (/api/raw/*, /api/ptc/*, colorchecker) coexists with the
    live NV12 view. This is now a no-op that always reports NV12 live; kept for old clients."""
    try:
        await request.json()
    except Exception:
        pass
    _set_raw_mode(False)                              # no-op; daemon always serves NV12 + RAW
    return {"raw_mode": False, "live_view": True,
            "note": "dual-pad daemon: NV12 live + RAW16 coexist; raw-mode toggle obsolete"}


@app.get("/api/cam_lock")
def api_cam_lock_get():
    """Setup-safe camera lock state. Locked => NV12-only: raw-mode toggles are no-ops and RAW
    grabs are refused, so a stray /raw, /ptc, or raw-mode request (e.g. from a second browser)
    can't reconfigure and wedge the 2.0 camera. NV12 live view + /fieldmap work while locked."""
    return {"locked": _cam_locked, "raw_mode": _raw_mode}


@app.post("/api/cam_lock")
async def api_cam_lock_set(request: Request):
    """Lock/unlock the camera. Unlock (locked:false) before RAW characterization (PTC/SNR1s/
    CCM/DCG); lock (locked:true) for setup/uniformity/alignment so nothing can wedge it."""
    global _cam_locked
    try:
        body = await request.json()
    except Exception:
        body = {}
    _cam_locked = bool(body.get("locked", True))
    return {"locked": _cam_locked, "raw_mode": _raw_mode}


@app.get("/api/raw/capture")
def api_raw_capture(width: int = 960, wb: int = 1):
    """Capture one native RAW16 frame; return characterization stats + a preview PNG."""
    if not RAW_ENABLE:
        return JSONResponse({"error": RAW_DISABLED_MSG}, status_code=503)
    try:
        frames, meta = _grab_raw(1)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)
    raw = frames[0]
    stats = raw_tools.characterize(raw)
    bgr = raw_tools.preview_bgr8(raw, out_w=width, wb=bool(wb), black=stats["black_level"])
    ok, buf = cv2.imencode(".png", bgr)
    import base64
    _last["raw"] = {"raw": raw, "results": stats,
                    "meta": {"source": "raw16", "camera": CAM,
                             "w": meta["width"], "h": meta["height"]}}
    return {"stats": stats, "meta": meta,
            "preview_png": "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()}


@app.get("/fieldmap", response_class=HTMLResponse)
def fieldmap_page():
    return _page("fieldmap.html")


@app.get("/api/field_map")
def api_field_map(width: int = 900):
    """Live illumination-uniformity heatmap for light tuning, off the NV12 worker (8-bit ISP
    luma = lens-shading-corrected illumination; STABLE, no camera reconfigure). Reports overall
    non-uniformity, 3x3 zone brightness (% of the brightest zone), mean and clip fraction.
    NV12 is gamma-encoded, so the % is relative (good for leveling, not an absolute figure)."""
    bgr = _frame(timeout_s=5.0)
    if bgr is None:
        return JSONResponse({"error": "no NV12 frame (camera in raw mode?)"}, status_code=502)
    g = np.clip(bgr[:, :, 1].astype(np.float64), 1.0, None)                    # ISP green as luma
    H, W = g.shape
    zy = np.linspace(0, H, 4).astype(int)
    zx = np.linspace(0, W, 4).astype(int)
    Z = np.array([[g[zy[i]:zy[i + 1], zx[j]:zx[j + 1]].mean() for j in range(3)] for i in range(3)])
    hm = cv2.applyColorMap((np.clip(g / max(g.max(), 1.0), 0, 1) * 255).astype(np.uint8),
                           cv2.COLORMAP_JET)
    ok, buf = cv2.imencode(".jpg", _resize(hm, width), [cv2.IMWRITE_JPEG_QUALITY, 80])
    import base64
    return {"nonunif_pct": round(float(100.0 * g.std() / g.mean()), 1),
            "zones": (100.0 * Z / Z.max()).round(0).astype(int).tolist(),
            "mean_DN": round(float(g.mean())),
            "clip_frac": round(float((bgr[:, :, 1] >= 250).mean()), 4),
            "map_jpg": "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()}


# Only one persistent bayer stream at a time (qtiqmmfsrc can't be opened twice) — shared by the
# field-map MJPEG and the PTC sweep so they can't collide on the single camera pipeline.
_stream = {"busy": None, "lock": threading.Lock()}


def _stream_acquire(name):
    """Reserve the persistent camera stream. Returns None on success, else the current holder."""
    with _stream["lock"]:
        if _stream["busy"]:
            return _stream["busy"]
        _stream["busy"] = name
        return None


def _stream_release(name):
    with _stream["lock"]:
        if _stream["busy"] == name:
            _stream["busy"] = None


# (removed) the old RAW/bayer _fieldmap_jpeg + _fieldmap_stream: they opened a private
# QmmfCapture(bayer) and killed the NV12 worker -> collide with the dual-pad daemon and wedge the
# 2.0 camera. The field-map now runs off NV12 (below).


def _fieldmap_jpeg_nv12(bgr):
    """NV12 (BGR) frame -> illumination-uniformity heatmap JPEG with non-uniformity + zone%
    overlaid. Uses the ISP green (luma proxy) so it reflects illumination AFTER lens-shading
    correction (8-bit, gamma-encoded -> relative). No pedestal subtraction (NV12 black ~0)."""
    g = np.clip(cv2.resize(bgr[:, :, 1].astype(np.float32), (480, 271),
                           interpolation=cv2.INTER_AREA), 1.0, None)
    nonunif = float(100.0 * g.std() / g.mean())
    zy = np.linspace(0, 271, 4).astype(int); zx = np.linspace(0, 480, 4).astype(int)
    Z = np.array([[g[zy[i]:zy[i + 1], zx[j]:zx[j + 1]].mean() for j in range(3)] for i in range(3)])
    zn = (100.0 * Z / Z.max()).round(0).astype(int)
    hm = cv2.applyColorMap((np.clip(g / max(g.max(), 1.0), 0, 1) * 255).astype(np.uint8),
                           cv2.COLORMAP_JET)
    hm = cv2.resize(hm, (900, 508), interpolation=cv2.INTER_NEAREST)
    clip = float((bgr[::4, ::4, 1] >= 250).mean())
    col = (90, 210, 90) if nonunif < 10 else (60, 200, 235) if nonunif < 20 else (70, 70, 235)
    cv2.putText(hm, "non-unif %.1f%%   mean %d   clip %.2f%%   [NV12 8-bit]" %
                (nonunif, g.mean(), clip * 100), (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
    hh, ww = hm.shape[:2]
    for i in range(3):
        for j in range(3):
            cv2.putText(hm, "%d%%" % zn[i, j], (int((j + 0.33) * ww / 3), int((i + 0.55) * hh / 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", hm, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buf.tobytes()


def _fieldmap_stream_nv12():
    """Live illumination heatmap MJPEG off the NV12 worker. Reads shared frames via _frame()
    (no camera reconfigure, no exclusive lock) so it's stable and coexists with the live view."""
    import time as _t
    while True:
        bgr = _frame(timeout_s=5.0)
        if bgr is None:
            _t.sleep(0.15)
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + _fieldmap_jpeg_nv12(bgr) + b"\r\n"
        _t.sleep(0.05)                                   # ~20 fps cap


@app.get("/stream.fieldmap.mjpg")
def stream_fieldmap():
    """Live illumination heatmap MJPEG off the NV12 worker (8-bit, stable). No RAW_ENABLE gate
    and no exclusive stream lock -- it just reads the shared NV12 frames, so multiple viewers
    and the main live view coexist. (The RAW/bayer field-map wedged cam-server on 2.0.)"""
    return StreamingResponse(_fieldmap_stream_nv12(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/api/raw/save")
async def api_raw_save(request: Request):
    """Capture N native RAW16 frames and save them (uint16 .npy + JSON sidecar) for
    offline colour-science (derived-CCM / dark-integrity / HDR)."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    if not RAW_ENABLE:
        return JSONResponse({"error": RAW_DISABLED_MSG}, status_code=503)
    n = max(1, min(int(body.get("n_frames", 1)), 32))
    label = "".join(c for c in str(body.get("label", "raw")) if c.isalnum() or c in "-_")[:40] or "raw"
    width, height = body.get("width"), body.get("height")
    shdr = bool(body.get("shdr", False))
    try:
        frames, meta = _grab_raw(n, width=width, height=height, shdr=shdr)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)
    os.makedirs(RAW_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(RAW_DIR, "%s_%s" % (ts, label))
    arr = np.stack(frames, 0)                       # (n, H, W) uint16
    np.save(base + ".npy", arr)
    stats = raw_tools.characterize(frames[0])
    import json
    sidecar = dict(meta)
    sidecar.update({"saved": ts, "label": label, "frames": len(frames),
                    "file": os.path.basename(base) + ".npy", "dtype": "uint16",
                    "stats_frame0": stats})
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump(sidecar, f, indent=2)
    return {"saved": True, "file": base + ".npy", "dir": RAW_DIR,
            "frames": len(frames), "bytes": int(arr.nbytes), "stats": stats}


@app.get("/api/raw/list")
def api_raw_list():
    import glob
    if not os.path.isdir(RAW_DIR):
        return {"dir": RAW_DIR, "captures": []}
    items = []
    for npy in sorted(glob.glob(os.path.join(RAW_DIR, "*.npy")), reverse=True)[:100]:
        items.append({"file": os.path.basename(npy),
                      "bytes": os.path.getsize(npy),
                      "mtime": int(os.path.getmtime(npy))})
    return {"dir": RAW_DIR, "captures": items}


@app.get("/api/histogram")
def api_histogram():
    bgr = _frame()
    if bgr is None:
        return JSONResponse({"error": "no frame"}, status_code=502)
    g = _channel(bgr, "Y")
    hist, _ = np.histogram(g, bins=64, range=(0, 256))
    return {"bins": hist.tolist(), "maxv": 255, "n": int(g.size),
            "clip_frac": float((g >= 254).mean())}


# ── MTF focus assist (analyze_gray on a demosaiced channel) ──────────────────
@app.post("/api/mtf")
async def api_mtf(request: Request):
    try:
        body = await request.json()
    except Exception:
        body = {}
    bgr = _frame()
    if bgr is None:
        return JSONResponse({"error": "no frame"}, status_code=502)
    gray = _channel(bgr, body.get("chan", "Y")).astype(np.float32) / 255.0
    try:
        res = mtf_analyze.analyze_gray(gray, body)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    res["source"] = "nv12"
    res["frame_source"] = "qtiqmmfsrc"
    _last["mtf"] = {"frame": bgr, "results": res,
                    "meta": {"source": "nv12-isp", "camera": CAM, "w": W, "h": H}}
    return res


# ── history / save (reused shared module; stores the processed frame) ────────
_last = {"mtf": None, "colorchecker": None, "colorchecker_raw": None, "raw": None, "ptc": None}


@app.post("/api/history/save")
async def api_history_save(request: Request):
    if not _HAS_HISTORY:
        return JSONResponse({"error": "history module not deployed"}, status_code=501)
    try:
        body = await request.json()
    except Exception:
        body = {}
    kind = body.get("kind", "mtf")
    slot = _last.get(kind)
    if slot is None:
        return JSONResponse({"error": "nothing to save for '%s' — measure first" % kind},
                            status_code=400)
    try:
        ov = slot["results"].get("overlay_png", "")
        import base64
        ovb = base64.b64decode(ov.split(",", 1)[1]) if "," in ov else None
        bid = history.save_bundle(kind, slot["results"], overlay_jpeg=ovb,
                                  meta=slot.get("meta"))
        return {"saved": True, "id": bid}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/api/history")
def api_history_list():
    if not _HAS_HISTORY:
        return {"bundles": [], "dir": ""}
    return {"bundles": history.list_bundles(), "dir": history.SESS_DIR}


# ── colorchecker: vendor-ISP colour eval on the NV12 (ISP-processed) frame ───
@app.post("/api/colorchecker")
async def api_colorchecker(request: Request):
    if not _HAS_CC:
        return JSONResponse({"error": "colorchecker module not deployed"}, status_code=501)
    bgr = _frame()
    if bgr is None:
        return JSONResponse({"error": "no frame"}, status_code=502)
    try:
        res = await run_in_threadpool(_cc_venv, "processed", bgr)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    if res.get("detected"):
        _last["colorchecker"] = {"frame": bgr, "results": res,
                                 "meta": {"source": "nv12-isp", "camera": CAM, "w": W, "h": H}}
    return res


CCVENV_PY = os.environ.get("IQ9_CCVENV", os.path.join(HERE, ".ccvenv", "bin", "python"))


def _cc_venv(mode, frame, maxv=4095):
    """Run ColorChecker analysis in the .ccvenv (working cv2.mcc 4.11). The webui's own cv2 has a
    BROKEN mcc stub, so in-process detection always fails; instead dump the frame to a temp .npy and
    subprocess cc_web.py under the venv, then parse the marker-delimited JSON result (with overlay/
    swatch pngs intact). mode='raw' -> colorchecker.analyze(frame, maxv); else analyze_processed."""
    import tempfile
    import json as _j
    fd, p = tempfile.mkstemp(suffix=".npy", dir="/dev/shm")
    os.close(fd)
    try:
        np.save(p, frame)
        args = [CCVENV_PY, os.path.join(HERE, "cc_web.py"), mode, p]
        if mode == "raw":
            args.append(str(int(maxv)))
        r = subprocess.run(args, capture_output=True, text=True, timeout=90)
        for line in r.stdout.splitlines():
            if line.startswith("@@CCJSON@@"):
                return _j.loads(line[len("@@CCJSON@@"):])
        return {"detected": False, "error": "cc venv: no result", "stderr": (r.stderr or "")[-300:]}
    except Exception as e:
        return {"detected": False, "error": "cc venv failed: %s" % e}
    finally:
        try:
            os.remove(p)
        except OSError:
            pass


# ── colorchecker: RAW-derived CCM (native linear RAW) + head-to-head vs ISP ──
def _grab_raw_mean(n=4):
    """Grab n RAW16 frames and return (mean_frame_uint16, meta). Averaging cuts temporal
    noise for a cleaner CCM fit; detection/analysis then run on the single mean frame."""
    frames, meta = _grab_raw(max(1, int(n)))
    if len(frames) <= 1:
        return frames[0], meta
    m = np.mean(np.stack(frames, 0).astype(np.float64), axis=0)
    return np.clip(m, 0, 65535).astype(np.uint16), meta


@app.post("/api/colorchecker/raw")
async def api_colorchecker_raw(request: Request):
    """RAW-derived CCM colour eval: capture native RAW16, detect the chart on linear RAW,
    fit a 3x3 CCM and report leave-one-out cross-validated ΔE00 (derived vs vendor) plus a
    root-poly upper bound — 'what the sensor can do', vs the ISP's baked-in colour."""
    if not _HAS_CC:
        return JSONResponse({"error": "colorchecker module not deployed"}, status_code=501)
    if not RAW_ENABLE:
        return JSONResponse({"error": RAW_DISABLED_MSG}, status_code=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    n = max(1, min(int(body.get("n_frames", 4)), 16))
    try:
        frame, meta = await run_in_threadpool(_grab_raw_mean, n)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)
    try:
        res = await run_in_threadpool(_cc_venv, "raw", frame, 4095)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    res["source"] = "raw16-derived"
    res["n_frames"] = n
    if res.get("detected"):
        _last["colorchecker_raw"] = {"frame": frame, "results": res,
                                     "meta": {"source": "raw16", "camera": CAM,
                                              "w": meta.get("width"), "h": meta.get("height")}}
    return res


@app.post("/api/colorchecker/headtohead")
async def api_colorchecker_h2h(request: Request):
    """Apples-to-apples: RAW-derived CCM (analyze) vs the ISP's factory colour
    (analyze_processed on NV12), same ColorChecker reference. Returns {raw, isp}."""
    if not _HAS_CC:
        return JSONResponse({"error": "colorchecker module not deployed"}, status_code=501)
    if not RAW_ENABLE:
        return JSONResponse({"error": RAW_DISABLED_MSG}, status_code=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    n = max(1, min(int(body.get("n_frames", 4)), 16))
    out = {}
    try:
        frame, meta = await run_in_threadpool(_grab_raw_mean, n)   # kills NV12 worker, then respawns
        raw_res = await run_in_threadpool(_cc_venv, "raw", frame, 4095)
        raw_res["source"] = "raw16-derived"
        raw_res["n_frames"] = n
        out["raw"] = raw_res
        if raw_res.get("detected"):
            _last["colorchecker_raw"] = {"frame": frame, "results": raw_res,
                                         "meta": {"source": "raw16", "camera": CAM,
                                                  "w": meta.get("width"), "h": meta.get("height")}}
    except Exception as e:
        out["raw"] = {"error": str(e)}
    try:
        bgr = None
        for _ in range(25):                                        # wait for a fresh NV12 frame
            bgr = _frame(timeout_s=5.0)
            if bgr is not None:
                break
            time.sleep(0.2)
        if bgr is None:
            out["isp"] = {"error": "no NV12 frame after RAW capture"}
        else:
            isp_res = await run_in_threadpool(_cc_venv, "processed", bgr)
            out["isp"] = isp_res
            if isp_res.get("detected"):
                _last["colorchecker"] = {"frame": bgr, "results": isp_res,
                                         "meta": {"source": "nv12-isp", "camera": CAM, "w": W, "h": H}}
    except Exception as e:
        out["isp"] = {"error": str(e)}
    return out


# ── RAW PTC / OETF / SNR (stream-based light sweep) ──────────────────────────
def _dmx_set(fwd, timeout=12):
    import json as _j
    import urllib.request
    try:
        req = urllib.request.Request(DMX_AGENT_URL + "/dmx", data=_j.dumps(fwd).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return _j.loads(r.read())
    except Exception as e:
        return {"error": "DMX agent unreachable: %s" % e}


def _roi_even(H, Wd, frac):
    """Central ROI (y0,y1,x0,x1) with even offsets so the RGGB Bayer phase is preserved."""
    frac = max(0.05, min(0.9, float(frac)))
    fh = int(H * frac); fw = int(Wd * frac)
    y0 = (H - fh) // 2; x0 = (Wd - fw) // 2
    y0 -= y0 % 2; x0 -= x0 % 2
    return (y0, y0 + fh, x0, x0 + fw)


_ptc = {"dark": None, "roi": None, "roi_frac": None}
_PTC_DARK_FILE = "/home/metro/.iq9_ptc_dark.json"   # persistent (survives reboot; /var/volatile is tmpfs)


def _ptc_save_dark():
    """Persist the cached dark so a webui restart (e.g. after a sensor-bin swap) doesn't force
    re-capping the lens."""
    try:
        import json as _j
        os.makedirs(os.path.dirname(_PTC_DARK_FILE), exist_ok=True)
        with open(_PTC_DARK_FILE, "w") as f:
            _j.dump({"dark": _ptc["dark"], "roi": _ptc["roi"], "roi_frac": _ptc["roi_frac"]}, f)
    except Exception:
        pass


def _ptc_load_dark():
    if _ptc["dark"] is not None:
        return
    try:
        import json as _j
        with open(_PTC_DARK_FILE) as f:
            d = _j.load(f)
        _ptc["dark"] = d.get("dark")
        _ptc["roi"] = tuple(d["roi"]) if d.get("roi") else None
        _ptc["roi_frac"] = d.get("roi_frac")
    except Exception:
        pass


def _ptc_capture_dark(nframes, roi_frac):
    """Dark reference: n CONSECUTIVE RAW frames from the persistent dual-pad daemon (shm) ->
    per-channel pedestal + temporal read noise. No private bayer stream, no NV12 kill."""
    frames, _ = _grab_raw(nframes)
    if len(frames) < 2:
        raise RuntimeError("dual-pad daemon yielded <2 RAW frames")
    H, Wd = frames[0].shape
    roi = _roi_even(H, Wd, roi_frac)
    dark = raw_ptc.measure(frames, roi)
    _ptc["dark"] = dark; _ptc["roi"] = roi; _ptc["roi_frac"] = float(roi_frac)
    _ptc_save_dark()
    return dark, roi


def _ptc_capture_sweep(channel, levels, nframes, settle, roi_frac, lux):
    """PTC/OETF/SNR light sweep on the persistent dual-pad daemon (RAW via shm; no camera reopen,
    no NV12<->RAW handoff). Per level: set the DMX, settle, then read n CONSECUTIVE RAW frames for
    the true 2-frame-diff temporal noise. The daemon keeps pulling NV12 continuously, so cam-server
    stays healthy across the DMX call + settle WITHOUT this code having to pump a private stream
    (that continuous-pull requirement was an artifact of the old single-client bayer stream)."""
    probe, _ = _grab_raw(1)
    H, Wd = probe[0].shape
    if _ptc["roi"] is not None and _ptc.get("roi_frac") == float(roi_frac):
        roi = _ptc["roi"]                                          # reuse the dark ROI when it matches
    else:
        roi = _roi_even(H, Wd, roi_frac)
    points = []; table = []
    for lvl in levels:
        _dmx_set({channel: int(lvl)})                              # set the illuminant
        time.sleep(settle)                                        # settle (daemon keeps NV12 alive)
        frames, _ = _grab_raw(nframes)                            # n consecutive RAW frames
        if len(frames) < 2:
            raise RuntimeError("dual-pad daemon yielded <2 RAW frames at level %d" % lvl)
        meas = raw_ptc.measure(frames, roi)
        sub = frames[0][roi[0]:roi[1], roi[2]:roi[3]]
        gp = sub[0::2, 1::2].astype(np.float64)                    # green within ROI (spatial uniformity)
        light = float(lux[str(lvl)]) if (lux and str(lvl) in lux) else float(lvl)
        points.append({"light": light, "meas": meas})
        table.append({"level": int(lvl), "light": light,
                      "green_DN": round(float(gp.mean()), 1),
                      "nonunif_pct": round(float(100.0 * gp.std() / max(gp.mean(), 1e-6)), 1),
                      "flicker_pct": round(float(meas["G1"]["mean_cv_pct"]), 3),
                      "clip": round(float((sub >= 4095).mean()), 4)})
    return points, roi, table


@app.post("/api/ptc/dark")
async def api_ptc_dark(request: Request):
    """Capture a dark reference (CAP THE LENS): consecutive frames from the persistent bayer
    stream -> per-channel pedestal + temporal read noise. Cached server-side for the sweep."""
    if not (_HAS_PTC and RAW_ENABLE):
        return JSONResponse({"error": RAW_DISABLED_MSG if not RAW_ENABLE else "raw_ptc not deployed"},
                            status_code=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    nframes = max(2, min(int(body.get("n_frames", 8)), 32))
    roi_frac = float(body.get("roi_frac", 0.4))
    holder = _stream_acquire("ptc")
    if holder:
        return JSONResponse({"error": "camera stream busy (%s)" % holder}, status_code=409)
    try:
        dark, roi = await run_in_threadpool(_ptc_capture_dark, nframes, roi_frac)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)
    finally:
        _stream_release("ptc")
    return {"ok": True, "roi": roi, "n_frames": nframes,
            "channels": {ch: {"pedestal_DN": round(float(dark[ch]["mean"]), 2),
                              "read_DN": round(float(dark[ch]["var"]) ** 0.5, 3),
                              "flicker_pct": round(float(dark[ch]["mean_cv_pct"]), 3)}
                         for ch in raw_ptc.BAYER},
            "note": "dark cached — uncap the lens, turn the light on, then run the sweep"}


@app.post("/api/ptc/sweep")
async def api_ptc_sweep(request: Request):
    """Stream-based PTC/OETF/SNR light sweep. body: channel(d65|tungsten), levels[], n_frames,
    settle_s, roi_frac, lux{level:lux}. Uses CONSECUTIVE frames per level from ONE persistent
    bayer stream (correct 2-frame-diff temporal noise). With lux, adds responsivity (e-/lux)
    and lux-referred SNR1s per channel."""
    if not (_HAS_PTC and RAW_ENABLE):
        return JSONResponse({"error": RAW_DISABLED_MSG if not RAW_ENABLE else "raw_ptc not deployed"},
                            status_code=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    channel = body.get("channel", "tungsten")
    if channel not in ("d65", "tungsten"):
        return JSONResponse({"error": "channel must be d65 or tungsten"}, status_code=400)
    lv = body.get("levels", [])
    if isinstance(lv, str):
        lv = [x for x in lv.replace(",", " ").split() if x]
    try:
        levels = [max(0, min(255, int(x))) for x in lv]
    except Exception:
        return JSONResponse({"error": "levels must be integers 0-255"}, status_code=400)
    levels = [x for x in levels if x > 0]                           # dark comes from /api/ptc/dark
    if len(levels) < 3:
        return JSONResponse({"error": "need >=3 non-zero levels for a PTC fit"}, status_code=400)
    nframes = max(2, min(int(body.get("n_frames", 3)), 16))
    settle = max(0.2, min(float(body.get("settle_s", 1.5)), 8.0))
    roi_frac = float(body.get("roi_frac", 0.4))
    lux = body.get("lux") or {}
    lux = {str(k): float(v) for k, v in lux.items()} if isinstance(lux, dict) else {}
    _ptc_load_dark()                                                # reload a persisted dark if memory is empty
    holder = _stream_acquire("ptc")
    if holder:
        return JSONResponse({"error": "camera stream busy (%s)" % holder}, status_code=409)
    try:
        points, roi, table = await run_in_threadpool(
            _ptc_capture_sweep, channel, levels, nframes, settle, roi_frac, lux or None)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)
    finally:
        _stream_release("ptc")
    rep = raw_ptc.characterize(points, _ptc["dark"])
    have_lux = bool(lux)
    curves = {}
    for ch in raw_ptc.BAYER:
        ped = rep["channels"][ch]["pedestal_DN"]
        sig = [p["meas"][ch]["mean"] - ped for p in points]
        var = [p["meas"][ch]["var"] for p in points]
        snr = [(s / (v ** 0.5)) if v > 0 else None for s, v in zip(sig, var)]
        curves[ch] = {"signal_DN": [round(float(s), 2) for s in sig],
                      "var_DN": [round(float(v), 2) for v in var],
                      "light": [p["light"] for p in points],
                      "snr": [round(float(x), 3) if x is not None else None for x in snr]}
        if have_lux:
            rep["channels"][ch].update(raw_ptc.lux_metrics(rep["channels"][ch]))
    result = {"channels": rep["channels"], "curves": curves, "table": table, "roi": roi,
              "channel": channel, "levels": levels, "lux_referred": have_lux,
              "dark_source": ("cached dark frames" if _ptc["dark"] else "darkest sweep level"),
              "notes": rep["notes"]}
    _last["ptc"] = {"results": result, "meta": {"source": "raw16-ptc", "camera": CAM,
                                                "channel": channel, "levels": levels}}
    return result


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", "8080")))
