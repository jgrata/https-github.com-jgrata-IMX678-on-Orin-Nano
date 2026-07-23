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
import time

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # webui/
from camera_client import CameraClient  # noqa: E402
import imaging  # noqa: E402
import colorchecker  # noqa: E402
import mtf_analyze  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IMG_HOST = os.environ.get("IMG_HOST", "127.0.0.1")
IMG_PORT = int(os.environ.get("IMG_PORT", "9000"))

app = FastAPI(title="Metro Camera Web UI (skeleton)")


def _client():
    return CameraClient(IMG_HOST, IMG_PORT)


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
    try:
        with _client() as c:
            frame, maxv = c.capture()
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
    with _client() as c:
        while True:
            try:
                frame, maxv = c.capture()
                jpg = imaging.encode_jpeg(imaging.fast_preview(frame, maxv, out_width=width))
            except Exception:
                break
            yield (boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")


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
    try:
        with _client() as c:
            frame, maxv = c.capture()
        return colorchecker.analyze(frame, maxv, black_level=bl)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


@app.get("/mtf", response_class=HTMLResponse)
def mtf_page():
    with open(os.path.join(HERE, "static", "mtf.html"), encoding="utf-8") as f:
        return f.read()


@app.post("/api/mtf")
async def api_mtf(request: Request):
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        with _client() as c:
            frame, maxv = c.capture()
        return mtf_analyze.analyze(frame, maxv, body)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=502)


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


@app.get("/stream.mjpg")
def stream(width: int = 960):
    return StreamingResponse(
        _mjpeg_generator(width),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", "8080")))
