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
    import colorchecker       # noqa: E402  (shared: detect + CIEDE2000; analyze_processed)
    _HAS_CC = True
except Exception:
    _HAS_CC = False
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
_exposure_comp = None                               # last-requested exposure-compensation (cached)
SHM = os.environ.get("IQ9_SHM", "/dev/shm/iq9_nv12")
CTL = SHM + ".ctl"
_MAGIC = b"IQ9N"
_HDR = len(_MAGIC) + 12                              # magic + u32 width,height,seq


def _pdeathsig():
    """Child preexec (Linux): die if the server process dies, so no orphan holds the camera."""
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGKILL)   # PR_SET_PDEATHSIG
    except Exception:
        pass


def _worker_alive():
    return _worker is not None and _worker.poll() is None


def _start_worker():
    """Spawn the NV12 worker if not running (caller holds _cam_lock)."""
    global _worker
    if _raw_mode or _worker_alive():
        return
    subprocess.run(["pkill", "-9", "-f", "camera_worker.py"],
                   capture_output=True)             # reap any orphan from a prior server
    for p in (SHM, CTL):
        try:
            os.remove(p)
        except OSError:
            pass
    env = dict(os.environ, IQ9_W=str(W), IQ9_H=str(H), IQ9_FPS=str(FPS),
               IQ9_CAM=str(CAM), IQ9_SHM=SHM)
    if _exposure_comp is not None:
        _write_ctl(_exposure_comp)
    _worker = subprocess.Popen([sys.executable, os.path.join(HERE, "camera_worker.py")],
                               env=env, preexec_fn=_pdeathsig)


def _kill_worker():
    """Fully kill the NV12 worker so cam-server releases the camera (caller holds _cam_lock)."""
    global _worker
    if _worker is not None:
        try:
            _worker.terminate()
            try:
                _worker.wait(timeout=3)
            except subprocess.TimeoutExpired:
                _worker.kill()
                _worker.wait(timeout=3)
        except Exception:
            pass
        _worker = None
    subprocess.run(["pkill", "-9", "-f", "camera_worker.py"], capture_output=True)
    try:
        os.remove(SHM)
    except OSError:
        pass
    time.sleep(RAW_QUIESCE_S)                       # let cam-server fully complete the IFE/RDI release
                                                    # (short quiesce correlated with the RDI hang)


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
    """Enter/exit cold RAW mode (kills / restores the NV12 worker)."""
    global _raw_mode
    with _cam_lock:
        if on and not _raw_mode:
            _kill_worker()
            _raw_mode = True
        elif not on and _raw_mode:
            _raw_mode = False
            _start_worker()
    return _raw_mode


def _frame(timeout_s=5.0):
    """Latest BGR frame from the NV12 worker (via shm). None while cold."""
    with _cam_lock:
        if _raw_mode:
            return None
        if not _worker_alive():
            _start_worker()
    return _read_shm(timeout_s=timeout_s)           # lock-free read (may wait for first frame)


def _grab_raw(n_frames=1, width=None, height=None):
    """Capture native RAW16 Bayer frames. Fully KILLS the NV12 worker first (the only
    reliable way to release the camera from cam-server), captures, then respawns it
    (unless in cold RAW mode). width/height override the capture geometry (e.g. the taller
    DCG/Clear HDR frame); omit for the shipping-mode default."""
    with _cam_lock:
        cold = _raw_mode
        _kill_worker()                              # full camera release
        try:
            kw = {}
            if width:
                kw["width"] = int(width)
            if height:
                kw["height"] = int(height)
            return camera_qmmf.grab_raw16(n_frames=n_frames, camera=CAM, **kw)
        finally:
            if not cold:
                _start_worker()


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
    info = api_info()
    info["applied"] = applied
    return info


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
    """Enter/exit cold RAW mode. Cold mode releases the NV12 live view and leaves the
    camera idle so RAW captures avoid the NV12<->RAW handoff churn (the most reliable
    trigger of the CAMSS/RDI hang). Entering requires RAW enabled; exiting is always OK."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    want = bool(body.get("raw", False))
    if want and not RAW_ENABLE:
        return JSONResponse({"error": RAW_DISABLED_MSG}, status_code=503)
    on = _set_raw_mode(want)
    return {"raw_mode": on, "live_view": (not on),
            "note": ("camera idle — RAW captures run without NV12 handoff churn" if on
                     else "NV12 live view active")}


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
    try:
        frames, meta = _grab_raw(n, width=width, height=height)
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
_last = {"mtf": None, "colorchecker": None, "raw": None}


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
        res = colorchecker.analyze_processed(bgr)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    if res.get("detected"):
        _last["colorchecker"] = {"frame": bgr, "results": res,
                                 "meta": {"source": "nv12-isp", "camera": CAM, "w": W, "h": H}}
    return res


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", "8080")))
