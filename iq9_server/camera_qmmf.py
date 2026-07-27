"""IQ9 (Qualcomm QCS9075) camera capture via GStreamer qtiqmmfsrc.

The IQ9 analog of the Jetson raw_capture/camera_client. `qtiqmmfsrc` is the QMMF
video source (the Qualcomm counterpart of nvarguscamerasrc): it exposes RAW Bayer
AND ISP-processed (NV12/NV16/RGB) from one source, up to 120fps, coordinated by the
system `cam-server`. Frames come to Python through a GStreamer appsink -- no CUDA,
no EGLStream, no C++ (unlike the Tegra path).

Status (2026-07-24, QCS9075 IQ-9075 EVK, Leopard Imaging IMX678, sensor mode
3856x2180 12-bit 30fps):
  - mode="nv12"  -> VALIDATED: processed frames as BGR (for live view / MTF focus).
  - mode="bayer" -> caps expose video/x-bayer up to 120fps, but naive caps deliver
    NO frame yet (RAW stream needs explicit camx/QMMF config -- the color/CCM path
    depends on this; see README "Open: RAW Bayer").

Caveat: qtiqmmfsrc does NOT tolerate multiple instances in one process
(`qmmfsrc_init` asserts context!=NULL) -- use ONE QmmfCapture per process.

    with QmmfCapture(1920, 1080, 30) as cam:
        bgr = cam.frame()          # HxWx3 uint8 BGR, or None on timeout
"""
import numpy as np

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
            # RAW Bayer -> uint16. NOTE: not yet delivering frames (camx/QMMF config WIP).
            desc = ("qtiqmmfsrc name=c %s! video/x-bayer,format=rggb,width=%d,height=%d,framerate=%d/1 "
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
            return np.frombuffer(mi.data, np.uint8).copy()                 # bayer: unpack TBD
        finally:
            buf.unmap(mi)
