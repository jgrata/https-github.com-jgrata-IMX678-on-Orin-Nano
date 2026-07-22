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


@app.get("/stream.mjpg")
def stream(width: int = 960):
    return StreamingResponse(
        _mjpeg_generator(width),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", "8080")))
