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
        "note": "ISP-processed NV12; linear-RAW features await CamX RDI usecase",
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
_last = {"mtf": None, "colorchecker": None}


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
