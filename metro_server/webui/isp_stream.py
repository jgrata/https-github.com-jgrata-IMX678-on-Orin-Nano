"""ISP-processed high-fps capture via nvarguscamerasrc (GStreamer) -> appsink.

Delivers ISP-processed BGR frames at ~60 fps (validated on hardware: 1080p60 =
58 fps, 4K30 = 30 fps, 4K60 ~ 50-60 fps, 0 dropped). Uses the Argus ISP pipeline
(the same path the vendor eCAM app uses), so it CANNOT run while raw_capture holds
the camera -- it is an ALTERNATE capture MODE:
  - RAW path (raw_capture -> image_server:9000): linear Bayer, for colour/CCM/IQ.
  - ISP path (this): fast processed frames, for focus assist / vendor-ISP eval.
Switching modes means releasing the other's hold on Argus (see the integration
that stops raw_capture before starting this).

    with ISPStream(1920, 1080, 60) as s:
        bgr = s.frame()          # HxWx3 uint8 BGR, or None on timeout
"""
import numpy as np

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    _GST = True
except Exception:                     # gi/Gst not present (e.g. dev PC)
    _GST = False


class ISPStream:
    def __init__(self, width=1920, height=1080, fps=60, sensor_id=0):
        if not _GST:
            raise RuntimeError("GStreamer (python3-gi + gstreamer1.0) not available")
        Gst.init(None)
        self.w, self.h = width, height
        caps = ("video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1"
                % (width, height, fps))
        desc = ("nvarguscamerasrc sensor-id=%d ! %s ! nvvidconv ! "
                "video/x-raw,format=BGRx ! "
                "appsink name=sink max-buffers=1 drop=true sync=false" % (sensor_id, caps))
        self.pipe = Gst.parse_launch(desc)
        self.sink = self.pipe.get_by_name("sink")

    def start(self):
        self.pipe.set_state(Gst.State.PLAYING)
        return self

    def stop(self):
        self.pipe.set_state(Gst.State.NULL)

    def frame(self, timeout_s=2.0):
        """Latest ISP frame as HxWx3 uint8 BGR (drop=true -> newest), or None."""
        sample = self.sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        st = sample.get_caps().get_structure(0)
        w = st.get_value("width"); h = st.get_value("height")
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            bgrx = np.frombuffer(mi.data, np.uint8).reshape(h, w, 4)   # BGRx from nvvidconv
            return bgrx[:, :, :3].copy()                              # -> BGR
        finally:
            buf.unmap(mi)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
