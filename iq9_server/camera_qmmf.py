"""IQ9 (Qualcomm QCS9075) camera capture via GStreamer qtiqmmfsrc.

The IQ9 analog of the Jetson raw_capture/camera_client. `qtiqmmfsrc` is the QMMF
video source (the Qualcomm counterpart of nvarguscamerasrc): it exposes RAW Bayer
AND ISP-processed (NV12/NV16/RGB) from one source, up to 120fps, coordinated by the
system `cam-server`. Frames come to Python through a GStreamer appsink -- no CUDA,
no EGLStream, no C++ (unlike the Tegra path).

Status (2026-07-30, QCS9075 IQ-9075 EVK, Leopard Imaging IMX678, sensor mode
3856x2180 12-bit 30fps):
  - mode="nv12"  -> VALIDATED: processed frames as BGR (for live view / MTF focus).
  - RAW Bayer    -> VALIDATED via grab_raw16(): native 3856x2180 12-bit RGGB. The key
    was requesting RAW16 (bpp=(string)16) -- the plugin's default RAW10 is rejected by
    CamX (max 3840x2160 < 3856x2180). RAW12 crashes cam-server; use RAW16 (12-bit data
    in a 16-bit container). See docs/raw-enablement.md.

Caveat: qtiqmmfsrc does NOT tolerate multiple instances in one process
(`qmmfsrc_init` asserts context!=NULL) -- use ONE QmmfCapture per process. RAW capture
therefore runs in an isolated `gst-launch-1.0` SUBPROCESS (grab_raw16), so the caller
must first release the camera (stop its own QmmfCapture) -- the camera is single-client.

    with QmmfCapture(1920, 1080, 30) as cam:
        bgr = cam.frame()          # HxWx3 uint8 BGR, or None on timeout

    cam.stop()                     # free the camera (single-client)
    frames = grab_raw16(n_frames=1)  # list of HxW uint16 RGGB (12-bit, max 4095)
    cam.start()                    # resume NV12
"""
import glob
import os
import subprocess
import tempfile

import numpy as np

# Sensor-native RAW readout (Leopard IMX678 on this EVK): full RGGB, 12-bit.
RAW_W, RAW_H, RAW_FPS = 3856, 2180, 30


def grab_raw16(n_frames=1, width=RAW_W, height=RAW_H, fps=RAW_FPS, camera=0,
               timeout_s=25, retries=1):
    """Capture native RAW16 Bayer frames via an isolated gst-launch subprocess.

    Returns (frames, meta): frames is a list of HxW uint16 arrays (RGGB, 12-bit,
    values 0..4095); meta carries the exact pipeline + per-frame raw byte size.
    The caller MUST have released the camera first (single-client). Raises
    RuntimeError with the gst stderr tail if nothing was captured.

    Why RAW16 (not RAW10/12): RAW10's advertised max is 3840x2160 < 3856x2180 so
    CamX rejects it at CheckValidStreamConfig; RAW12 destabilises cam-server. RAW16
    validates at native res and carries the 12-bit data in a 16-bit container.
    """
    caps = ("video/x-bayer,format=rggb,bpp=(string)16,"
            "width=%d,height=%d,framerate=%d/1" % (width, height, fps))
    last_err = ""
    for _attempt in range(retries + 1):
        tmp = tempfile.mkdtemp(prefix="iq9raw_")
        pat = os.path.join(tmp, "f_%03d.bin")
        cam = [] if camera == 0 else ["camera=%d" % camera]
        # eos-after must be n_frames+1: `identity eos-after=N` fires EOS as the Nth buffer
        # passes, and that EOS races the sink -> the Nth buffer is often torn down unwritten.
        # Grabbing one extra buffer guarantees n_frames complete files (verified: eos-after=1
        # yields 0 bytes; eos-after=2 yields a full frame).
        argv = (["gst-launch-1.0", "-e", "qtiqmmfsrc"] + cam +
                ["!", caps,
                 "!", "identity", "eos-after=%d" % (int(n_frames) + 1),
                 "!", "multifilesink", "location=%s" % pat])
        try:
            p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout_s)
            stderr = p.stderr or ""
        except subprocess.TimeoutExpired as e:
            stderr = (e.stderr.decode() if isinstance(e.stderr, bytes) else (e.stderr or ""))
        files = sorted(glob.glob(os.path.join(tmp, "f_*.bin")))
        frames, raw_bytes = [], 0
        for fp in files:
            a = np.fromfile(fp, dtype="<u2")
            raw_bytes = max(raw_bytes, a.size * 2)
            if a.size >= width * height:
                frames.append(a[:width * height].reshape(height, width).copy())
            try:
                os.remove(fp)
            except OSError:
                pass
        try:
            os.rmdir(tmp)
        except OSError:
            pass
        if frames:
            return frames, {"caps": caps, "width": width, "height": height,
                            "fps": fps, "raw_bytes_per_buffer": raw_bytes,
                            "n": len(frames), "bit_depth": 12, "cfa": "RGGB"}
        last_err = "\n".join(l for l in stderr.splitlines()
                             if "MESA" not in l and "driver name" not in l)[-1500:]
    raise RuntimeError("RAW capture produced no frame. gst-launch stderr tail:\n" + last_err)

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    _GST = True
except Exception:                      # gi/Gst absent (e.g. dev PC)
    _GST = False

_INIT = False


def _ensure_gst():
    global _INIT
    if not _GST:
        raise RuntimeError("GStreamer (python3-gi + gstreamer1.0) not available")
    if not _INIT:
        Gst.init(None)
        _INIT = True


class QmmfCapture:
    def __init__(self, width=1920, height=1080, fps=30, mode="nv12", camera=0):
        _ensure_gst()
        self.w, self.h, self.fps, self.mode = width, height, fps, mode
        cam = "" if camera == 0 else ("camera=%d " % camera)
        if mode == "nv12":
            desc = ("qtiqmmfsrc name=c %s! video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 "
                    "! videoconvert ! video/x-raw,format=BGRx "
                    "! appsink name=s max-buffers=2 drop=true sync=false"
                    % (cam, width, height, fps))
        elif mode == "bayer":
            # RAW16 Bayer -> uint16. bpp MUST be a caps string; RAW16 (not RAW10/12).
            # Prefer grab_raw16() (isolated subprocess); this in-process path is for tools
            # that own the camera exclusively.
            desc = ("qtiqmmfsrc name=c %s! video/x-bayer,format=rggb,bpp=(string)16,"
                    "width=%d,height=%d,framerate=%d/1 "
                    "! appsink name=s max-buffers=2 drop=true sync=false"
                    % (cam, width, height, fps))
        else:
            raise ValueError("mode must be 'nv12' or 'bayer'")
        self.desc = desc
        self.pipe = Gst.parse_launch(desc)
        self.sink = self.pipe.get_by_name("s")
        self.src = self.pipe.get_by_name("c")        # the qtiqmmfsrc element (for live props)

    def set_prop(self, name, value):
        """Best-effort live set of a qtiqmmfsrc property (e.g. exposure-compensation).
        Returns True if the property exists and was set."""
        try:
            if self.src is not None and self.src.find_property(name) is not None:
                self.src.set_property(name, value)
                return True
        except Exception:
            pass
        return False

    def get_prop(self, name, default=None):
        try:
            if self.src is not None and self.src.find_property(name) is not None:
                return self.src.get_property(name)
        except Exception:
            pass
        return default

    def start(self):
        self.pipe.set_state(Gst.State.PLAYING)
        return self

    def stop(self):
        self.pipe.set_state(Gst.State.NULL)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    def frame(self, timeout_s=5.0):
        """Latest frame: BGR HxWx3 uint8 (nv12 mode) or raw bytes (bayer), or None."""
        samp = self.sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if samp is None:
            return None
        buf = samp.get_buffer()
        st = samp.get_caps().get_structure(0)
        w = st.get_value("width"); h = st.get_value("height")
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            if self.mode == "nv12":
                # BGRx = 4 bytes/px; account for row-stride padding (stride = size/h).
                stride = mi.size // h
                a = np.frombuffer(mi.data, np.uint8, count=stride * h).reshape(h, stride)
                return a[:, :w * 4].reshape(h, w, 4)[:, :, :3].copy()      # -> BGR
            # bayer (RAW16): uint16 LE, stride == width (no line pad); take the h image rows.
            a = np.frombuffer(mi.data, dtype="<u2")
            if a.size < w * h:
                return None
            return a[:w * h].reshape(h, w).copy()
        finally:
            buf.unmap(mi)
