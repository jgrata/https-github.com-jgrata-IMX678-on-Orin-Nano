"""FastAPI web-UI skeleton for the IMX678/Jetson camera.

Runs on the Jetson; talks to image_server.py over localhost:9000 as an additional
client. Endpoints:
  GET  /                 -> live view + controls + histogram (static/index.html)
  GET  /api/info         -> camera info (CMD_GET_INFO)
  POST /api/params       -> set exposure/gain/mode/... (CMD_SET_PARAMS) -> new info
  GET  /api/frame.jpg    -> single preview JPEG
  GET  /api/histogram    -> raw-frame histogram JSON
  GET  /stream.mjpg      -> multipart MJPEG live stream

Run:  IMG_HOST=127.0.0.1 IMG_PORT=9000 PORT=8080 python3 server.py
"""
import os
import sys
import threading
import time

import numpy as np
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # webui/
from camera_client import CameraClient  # noqa: E402
import imaging  # noqa: E402
import colorchecker  # noqa: E402
import mtf_analyze  # noqa: E402
import darkcheck  # noqa: E402
import history  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IMG_HOST = os.environ.get("IMG_HOST", "127.0.0.1")
IMG_PORT = int(os.environ.get("IMG_PORT", "9000"))

app = FastAPI(title="Metro Camera Web UI (skeleton)")


def _client():
    return CameraClient(IMG_HOST, IMG_PORT)


# Last measurement per kind, so "Save" persists EXACTLY what's on screen (no
# re-capture). Holds the raw frame too, for an optional full-fidelity bundle.
_last = {"colorchecker": None, "mtf": None}
_CAPTURE_META_KEYS = ("exposure_ns", "gain", "sensormode", "sensor_mode", "fps",
                      "bit_depth", "width", "height", "actual_exposure_ns", "actual_gain")


def _capture_meta(info):
    return {k: info[k] for k in _CAPTURE_META_KEYS if isinstance(info, dict) and k in info}


def _overlay_bytes(results):
    uri = results.get("overlay_png") or ""
    if "," in uri:
        import base64
        try:
            return base64.b64decode(uri.split(",", 1)[1])
        except Exception:
            return None
    return None


# --- Local zero-copy frame source (shared memory) ---------------------------
# raw_capture publishes decoded uint16 frames to /dev/shm/metro_raw. Reading them
# here skips RAW10 pack + localhost TCP + NumPy unpack -- the fast LOCAL path.
# A FRESH reader is opened per call (open+mmap is cheap; the frame copy dominates):
# this is robust against a raw_capture restart recreating the shm as a NEW inode
# (a persistent mmap would freeze on the dead inode -> stale frames forever) and
# against cross-thread sharing (stream/frame/mtf all call this concurrently).


def _get_frame(prefer_shm=True):
    """Return (frame uint16 Bayer, maxv, meta). Prefers the local shm ring; falls
    back to the TCP capture. meta.source is 'shm' or 'tcp'."""
    if prefer_shm and not _isp_active():
        try:
            import shm_reader
            with shm_reader.ShmReader() as r:          # fresh mmap each call (see note above)
                f = r.latest()
            if f is not None:
                return f["frame"], f["maxv"], {
                    "exposure_ns": int(f["exp_ns"]), "gain": float(f["gain"]),
                    "sof_ns": int(f["sof_ns"]), "source": "shm"}
        except Exception:
            pass
    with _client() as c:
        frame, maxv = c.capture()
        meta = _capture_meta(c.info())
    meta["source"] = "tcp"
    return frame, maxv, meta


# --- ISP capture mode (nvarguscamerasrc) ------------------------------------
# The RAW path (raw_capture -> image_server) and the ISP path (nvarguscamerasrc,
# in-process here) can't hold Argus at once. Entering ISP mode releases the RAW
# camera (CMD_RELEASE_CAM); leaving it reacquires (CMD_REACQUIRE_CAM). One stream
# at a time, guarded by a lock.
_isp_lock = threading.Lock()
_isp = {"stream": None, "w": 0, "h": 0, "fps": 0, "last": 0.0}
_ISP_IDLE_LIMIT = 30.0        # s: auto-leave ISP if the page stops polling (closed tab)


def _isp_active():
    return _isp["stream"] is not None


def _isp_channel(bgr, chan):
    """Pick a single plane from an ISP BGR frame for the SFR (no Bayer bin here --
    the ISP already demosaiced, so every plane is full-res with signal)."""
    if chan == "R":
        return bgr[:, :, 2]
    if chan == "B":
        return bgr[:, :, 0]
    return bgr[:, :, 1]        # green as the luma proxy for 'Y'/'G'


def _isp_watchdog():
    """A released camera lives in THIS process, not the browser tab. If the page
    stops polling /api/mtf (tab closed, navigated away) while ISP mode is on, leave
    ISP and reacquire RAW so raw_capture doesn't stay dark indefinitely."""
    while True:
        time.sleep(5.0)
        idle = False
        with _isp_lock:
            s = _isp["stream"]
            if s is not None and (time.monotonic() - _isp["last"]) >= _ISP_IDLE_LIMIT:
                idle = True
                try:
                    s.stop()
                except Exception:
                    pass
                _isp.update(stream=None, w=0, h=0, fps=0)
        if idle:
            try:
                with _client() as c:
                    c.reacquire_camera()
            except Exception:
                pass


threading.Thread(target=_isp_watchdog, daemon=True).start()


@app.get("/", response_class=HTMLResponse)
def index():
    with open(os.path.join(HERE, "static", "index.html"), encoding="utf-8") as f:
        return f.read()


@app.get("/api/info")
def api_info():
    try:
        with _client() as c:
            return c.info()
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.post("/api/params")
async def api_params(request: Request):
    body = await request.json()
    allowed = {"sensormode", "fps", "exposure_ns", "gain", "bit_depth",
               "lossless", "sat_threshold", "hdr_exp_ratio"}
    params = {k: v for k, v in body.items() if k in allowed}
    try:
        with _client() as c:
            c.set_params(params)
            return c.info()
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.get("/api/frame.jpg")
def api_frame(width: int = 960):
    if _isp_active():
        return JSONResponse({"error": "camera in ISP mode (use the MTF page)"}, status_code=409)
    try:
        frame, maxv, _ = _get_frame()
        jpg = imaging.encode_jpeg(imaging.fast_preview(frame, maxv, out_width=width))
        return Response(content=jpg, media_type="image/jpeg")
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.get("/api/histogram")
def api_histogram():
    try:
        with _client() as c:
            frame, maxv = c.capture()
        return imaging.histogram(frame, maxv)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


def _mjpeg_generator(width):
    boundary = b"--frame"
    while True:
        try:
            frame, maxv, _ = _get_frame()          # local shm if available
            jpg = imaging.encode_jpeg(imaging.fast_preview(frame, maxv, out_width=width))
        except Exception:
            break
        yield (boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
               + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.03)                           # cap ~30 fps; shm.latest() is instant


@app.get("/colorchecker", response_class=HTMLResponse)
def colorchecker_page():
    with open(os.path.join(HERE, "static", "colorchecker.html"), encoding="utf-8") as f:
        return f.read()


@app.post("/api/colorchecker")
async def api_colorchecker(request: Request):
    try:
        body = await request.json()
    except Exception:
        body = {}
    bl = body.get("black_level")
    rp_deg = int(body.get("rootpoly_degree", 2))
    try:
        with _client() as c:
            frame, maxv = c.capture()
            meta = _capture_meta(c.info())
        res = colorchecker.analyze(frame, maxv, black_level=bl, rootpoly_degree=rp_deg)
        _last["colorchecker"] = {"frame": frame, "maxv": maxv, "results": res, "meta": meta}
        return res
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.get("/mtf", response_class=HTMLResponse)
def mtf_page():
    with open(os.path.join(HERE, "static", "mtf.html"), encoding="utf-8") as f:
        return f.read()


@app.get("/api/isp/status")
def api_isp_status():
    return {"active": _isp_active(), "width": _isp["w"], "height": _isp["h"], "fps": _isp["fps"]}


@app.post("/api/isp/start")
async def api_isp_start(request: Request):
    """Enter ISP mode: release the RAW camera, then open nvarguscamerasrc in-process.
    Confirms a first frame before committing; on any failure it reacquires RAW."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    w = int(body.get("width", 1920)); h = int(body.get("height", 1080)); fps = int(body.get("fps", 60))
    with _isp_lock:
        if _isp["stream"] is not None:
            return {"active": True, "width": _isp["w"], "height": _isp["h"], "fps": _isp["fps"],
                    "note": "already active"}
        try:
            import isp_stream
        except Exception as e:
            return JSONResponse({"error": "ISP unavailable (no GStreamer): " + str(e)}, status_code=501)
        try:
            with _client() as c:
                c.release_camera()
        except Exception as e:
            return JSONResponse({"error": "release_camera failed: " + str(e)}, status_code=502)
        # Releasing the RAW camera tears down its Argus CameraProvider, which
        # frequently crashes nvargus-daemon; systemd restarts it but it takes
        # ~13 s to relist. So retry (recreate the pipeline each attempt -- a failed
        # "connection refused" attempt returns immediately) until a frame arrives.
        last_err = None
        for attempt in range(11):                        # ~28 s budget (covers the daemon restart)
            s = None
            try:
                s = isp_stream.ISPStream(w, h, fps).start()
                if s.frame(timeout_s=2.5) is not None:
                    _isp.update(stream=s, w=w, h=h, fps=fps, last=time.monotonic())
                    return {"active": True, "width": w, "height": h, "fps": fps,
                            "attempts": attempt + 1}
                last_err = "no frame"
            except Exception as e:
                last_err = str(e)
            try:
                if s is not None:
                    s.stop()
            except Exception:
                pass
            time.sleep(2.5)                              # wait for nvargus-daemon to relist
        try:
            with _client() as c:
                c.reacquire_camera()                     # give up -> roll back to RAW
        except Exception:
            pass
        return JSONResponse({"error": "ISP failed after retries: %s" % last_err, "active": False},
                            status_code=502)


@app.post("/api/isp/stop")
def api_isp_stop():
    """Leave ISP mode: stop nvarguscamerasrc and reacquire the RAW camera."""
    with _isp_lock:
        s = _isp["stream"]
        if s is not None:
            try:
                s.stop()
            finally:
                _isp.update(stream=None, w=0, h=0, fps=0)
        try:
            with _client() as c:
                c.reacquire_camera()
        except Exception as e:
            return JSONResponse({"error": "stopped ISP but reacquire failed: " + str(e),
                                 "active": False}, status_code=502)
    return {"active": False}


@app.post("/api/mtf")
async def api_mtf(request: Request):
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        if _isp_active():
            with _isp_lock:
                s = _isp["stream"]
                _isp["last"] = time.monotonic()
            bgr = s.frame(timeout_s=2.0) if s is not None else None
            if bgr is None:
                return JSONResponse({"error": "ISP frame timeout"}, status_code=502)
            gray = _isp_channel(bgr, body.get("chan", "R"))
            res = mtf_analyze.analyze_gray(gray, body)
            res["source"] = "isp"
            return res
        frame, maxv, meta = _get_frame()          # local shm if available, else TCP
        res = mtf_analyze.analyze(frame, maxv, body)
        res["source"] = "raw"
        res["frame_source"] = meta.get("source")
        _last["mtf"] = {"frame": frame, "maxv": maxv, "results": res, "meta": meta}
        return res
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.get("/api/sensor_timing")
def api_sensor_timing(duration_s: float = 2.0):
    """Measure true sensor frame timing (period, fps, jitter, drops) from the local
    shm ring's per-frame sensor timestamps -- a full-rate, non-ISP timing probe that
    doesn't touch the network."""
    try:
        import shm_reader
    except Exception as e:
        return JSONResponse({"error": "shm_reader unavailable: %s" % e}, status_code=500)
    try:
        return shm_reader.sensor_timing(max(0.2, min(float(duration_s), 10.0)))
    except shm_reader.ShmUnavailable as e:
        return JSONResponse({"error": str(e)}, status_code=503)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/history", response_class=HTMLResponse)
def history_page():
    with open(os.path.join(HERE, "static", "history.html"), encoding="utf-8") as f:
        return f.read()


@app.post("/api/history/save")
async def api_history_save(request: Request):
    """Persist the last measurement of `kind` (exactly what's on screen). Set
    save_raw to also store the 4K uint16 frame for MATLAB re-analysis."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    kind = body.get("kind")
    save_raw = bool(body.get("save_raw", False))
    slot = _last.get(kind)
    if slot is None:
        return JSONResponse({"error": "nothing to save for '%s' — measure first" % kind},
                            status_code=400)
    try:
        bid = history.save_bundle(
            kind, slot["results"], overlay_jpeg=_overlay_bytes(slot["results"]),
            raw=(slot["frame"] if save_raw else None), maxv=slot.get("maxv"),
            meta=slot.get("meta"))
        return {"saved": True, "id": bid, "saved_raw": save_raw}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/api/history")
def api_history_list():
    try:
        return {"bundles": history.list_bundles(), "dir": history.SESS_DIR}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/api/history/item")
def api_history_item(stem: str):
    b = history.get_bundle(stem)
    if b is None:
        return JSONResponse({"error": "not found"}, status_code=404)
    return b


@app.get("/api/history/thumb")
def api_history_thumb(stem: str):
    jpg = history.overlay_jpeg(stem)
    if jpg is None:
        return Response(status_code=404)
    return Response(content=jpg, media_type="image/jpeg")


@app.get("/api/history/download")
def api_history_download(stem: str):
    from fastapi.responses import FileResponse
    p = history.download_path(stem)
    if p is None:
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p, media_type="application/x-hdf5", filename=os.path.basename(p))


@app.post("/api/history/delete")
async def api_history_delete(request: Request):
    try:
        body = await request.json()
    except Exception:
        body = {}
    stem = body.get("stem")
    if not stem:
        return JSONResponse({"error": "stem required"}, status_code=400)
    return {"deleted": history.delete_bundle(stem)}


@app.post("/api/colorchecker/meter")
async def api_meter(request: Request):
    """Auto-find an HDR bracket that keeps the WHOLE chart unclipped with good
    shadow SNR: meter the brightest patch-channel to `target_hi` of full scale
    (short leg, no clipping) and the darkest patch to `target_lo` (long leg, SNR).
    Closed loop first drops exposure until the highlight is unclipped so its true
    level is measurable. Reports the leg exposures; does not commit them."""
    import math
    try:
        body = await request.json()
    except Exception:
        body = {}
    target_hi = float(body.get("target_hi", 0.90))     # brightest channel -> 90% FS (10% headroom)
    target_lo = float(body.get("target_lo", 0.35))     # darkest patch -> 35% FS (strong shadow SNR)
    MIN_NS, MAX_NS = 50_000, 500_000_000               # 0.05 ms .. 500 ms
    try:
        with _client() as c:
            exp = int(c.info().get("exposure_ns", 8_000_000))
            lv = None
            trace = []
            for _ in range(5):
                frame, maxv = c.capture()
                lv = colorchecker.meter_levels(frame, maxv)
                if not lv.get("detected"):
                    return JSONResponse({"error": "chart not detected — reframe/expose",
                                         "clip_frac": lv.get("clip_frac")}, status_code=200)
                trace.append({"exp_ms": exp / 1e6, "hi": round(lv["hi"], 3), "lo": round(lv["lo"], 4)})
                if lv["hi_clip"] and exp > MIN_NS * 2:
                    exp = max(int(exp * 0.5), MIN_NS)
                    c.set_params({"exposure_ns": exp}); time.sleep(1.3)
                    continue
                break
            hi, lo = max(lv["hi"], 1e-6), max(lv["lo"], 1e-6)
            t_short = min(max(int(exp * target_hi / hi), MIN_NS), MAX_NS)
            t_long = min(max(int(exp * target_lo / lo), MIN_NS), MAX_NS)
            if t_long < t_short:
                t_long = t_short
            ratio = t_long / t_short
            n = 1 if ratio < 1.5 else max(2, min(5, int(round(math.log2(ratio))) + 1))
            if n == 1:
                legs = [t_short]
            else:
                legs = [t_short * (ratio ** (i / (n - 1))) for i in range(n)]
            legs_ms = [round(x / 1e6, 3) for x in legs]
            single_ok = lv["chart_dr"] <= 200          # ~7.6 stops fits one 12-bit frame cleanly
            note = ("chart fits one exposure - a single well-exposed frame is the cleanest CCM capture; "
                    "HDR mainly improves shadow-patch SNR") if single_ok else \
                   ("bracket needed for unclipped highlights + shadow SNR")
            return {
                "detected": True,
                "chart_dr": round(lv["chart_dr"], 1),
                "chart_dr_stops": round(math.log2(lv["chart_dr"]), 1) if lv["chart_dr"] > 0 else None,
                "hi_at_meter": round(hi, 3), "lo_at_meter": round(lo, 4),
                "meter_exp_ms": round(exp / 1e6, 3),
                "t_short_ms": round(t_short / 1e6, 3), "t_long_ms": round(t_long / 1e6, 3),
                "bracket_ratio": round(ratio, 1),
                "exceeds_native_16to1": ratio > 16.5,
                "legs_ms": legs_ms,
                "exposures_str": ", ".join("%.3g" % x for x in legs_ms),
                "target_hi": target_hi, "target_lo": target_lo,
                "clamped": (t_short <= MIN_NS or t_long >= MAX_NS),
                "note": note, "trace": trace,
            }
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.post("/api/darkcheck")
async def api_darkcheck(request: Request):
    """Certify a capped-lens dark: capture at a short and a long exposure and test
    for light leakage (exposure invariance is decisive). Restores exposure after."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    exp_short_ms = float(body.get("exp_short_ms", 0.2))
    exp_long_ms = float(body.get("exp_long_ms", 30.0))
    try:
        with _client() as c:
            orig = int(c.info().get("exposure_ns", 8_000_000))

            def grab(exp_ms):
                c.set_params({"exposure_ns": int(exp_ms * 1e6)})
                time.sleep(1.3)
                frame, maxv = c.capture()
                return darkcheck.dark_stats(frame), maxv

            s, maxv = grab(exp_short_ms)
            l, _ = grab(exp_long_ms)
            c.set_params({"exposure_ns": orig})           # restore
            pedestal = 200.0 if maxv > 2000 else 50.0
            v = darkcheck.verdict(s, l, exp_short_ms, exp_long_ms, pedestal)
            return {
                "pedestal_expected": pedestal,
                "exp_short_ms": exp_short_ms, "exp_long_ms": exp_long_ms,
                "short": s, "long": l, **v,
            }
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.get("/stream.mjpg")
def stream(width: int = 960):
    return StreamingResponse(
        _mjpeg_generator(width),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", "8080")))
