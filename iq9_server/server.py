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
import os
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

_cam = None
_cam_lock = threading.Lock()


def _get_cam():
    global _cam
    if _cam is None:
        _cam = camera_qmmf.QmmfCapture(W, H, FPS, mode="nv12", camera=CAM).start()
        time.sleep(1.2)                       # let 3A (AE/AWB) settle
    return _cam


def _frame(timeout_s=3.0):
    """Latest BGR frame (thread-safe; qtiqmmfsrc is single-instance)."""
    with _cam_lock:
        return _get_cam().frame(timeout_s=timeout_s)


def _grab_raw(n_frames=1):
    """Capture native RAW16 Bayer frames. The camera is single-client, so this releases
    the persistent NV12 capture, runs the isolated grab_raw16 subprocess, then resumes
    NV12 -- all under the camera lock. Returns (frames, meta)."""
    global _cam
    with _cam_lock:
        running = _cam is not None
        if running:
            # Fully DISPOSE the NV12 capture (not just stop) so the qtiqmmfsrc recorder
            # client disconnects and cam-server runs its full release of the IFE/RDI --
            # mirroring the process-death path that was stable. Then quiesce before RAW.
            try:
                _cam.stop()
            except Exception:
                pass
            _cam = None
            import gc
            gc.collect()
            time.sleep(2.0)                       # let cam-server fully release the camera
        try:
            return camera_qmmf.grab_raw16(n_frames=n_frames, camera=CAM)
        finally:
            if running:
                try:
                    _get_cam()                    # recreate NV12 fresh (includes 3A settle)
                except Exception:
                    _cam = None                   # lazy re-create on next NV12 use


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
    exp_comp = None
    try:
        exp_comp = _get_cam().get_prop("exposure-compensation")
    except Exception:
        pass
    return {
        "platform": "IQ9 QCS9075", "source": "nv12-isp (qtiqmmfsrc)",
        "camera": CAM, "width": W, "height": H, "fps": FPS,
        "bit_depth": 8, "sensormode": 0, "exposure_ns": 0, "gain": 0,
        "exposure_compensation": exp_comp,
        "raw_available": True,
        "raw_enabled": RAW_ENABLE,
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
    applied = {}
    with _cam_lock:
        cam = _get_cam()
        if "exposure_compensation" in body:
            applied["exposure_compensation"] = cam.set_prop(
                "exposure-compensation", int(body["exposure_compensation"]))
        for k in ("antibanding", "white-balance-mode", "control-mode"):
            if k in body:
                applied[k] = cam.set_prop(k, body[k])
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
    try:
        frames, meta = _grab_raw(n)
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
