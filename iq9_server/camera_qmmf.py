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


def _destride_raw16(a, width, height):
    """De-pad a flat RAW16 (uint16 LE) buffer to an (H, width) array. On QLI 1.7 the
    qtiqmmfsrc row stride equalled the requested width; on 2.0 it pads the stride (e.g.
    3856 -> 3872 px) and returns a few fewer rows (2176), so the naive width*height reshape
    shears the frame. Detect the true stride from the buffer size — the smallest px-aligned
    stride >= width that divides the buffer evenly — and crop back to the active width.
    Returns an (H, width) uint16 copy, or None if the buffer is too small."""
    n = int(a.size)
    if n == width * height:                       # 1.7: no line padding
        return a.reshape(height, width).copy()
    for align in (8, 16, 32, 64, 128, 256, 512, 1024):
        s = ((width + align - 1) // align) * align
        if s > width and n % s == 0:              # 2.0: padded stride (3872 for width 3856)
            h = n // s
            if h > 0:
                return a[:s * h].reshape(h, s)[:, :width].copy()
    if n >= width * height:                       # fallback: assume no pad
        return a[:width * height].reshape(height, width).copy()
    return None


def grab_raw16(n_frames=1, width=RAW_W, height=RAW_H, fps=RAW_FPS, camera=0,
               timeout_s=25, retries=1, exposure_ns=None, iso=None, shdr=False):
    """Capture native RAW16 Bayer frames via an isolated gst-launch subprocess.

    Returns (frames, meta): frames is a list of HxW uint16 arrays (RGGB, 12-bit,
    values 0..4095); meta carries the exact pipeline + per-frame raw byte size.
    The caller MUST have released the camera first (single-client). Raises
    RuntimeError with the gst stderr tail if nothing was captured.

    For characterization (OETF/PTC, dark, SNR-vs-gain) pass exposure_ns (manual
    exposure, disables AE) and/or iso (manual ISO/gain) -> qtiqmmfsrc runs 3A OFF and
    holds the requested values, so an exposure/gain sweep is repeatable. Leaving both
    None keeps 3A auto (a quick look/preview grab).

    shdr=True adds `vhdr=shdr-raw` -> qtiqmmfsrc requests the Raw SHDR (2-exposure)
    usecase so CamX acquires the IFE with is_shdr=1 and demuxes the sensor's two VCs.
    REQUIRED for Clear HDR / DOL capture: without it the IFE acquires a single RDI port
    (is_shdr=0) and no buffers flow even when the sensor is in SHDR mode. shdr-raw is
    LINE-INTERLEAVED (the two legs alternate lines) -> demux with dcg_demux interleaved.

    Why RAW16 (not RAW10/12): RAW10's advertised max is 3840x2160 < 3856x2180 so
    CamX rejects it at CheckValidStreamConfig; RAW12 destabilises cam-server. RAW16
    validates at native res and carries the 12-bit data in a 16-bit container.
    """
    caps = ("video/x-bayer,format=rggb,bpp=(string)16,"
            "width=%d,height=%d,framerate=%d/1" % (width, height, fps))
    # manual 3A for characterization (disable AE/AWB drift; hold exposure/gain)
    props = []
    if shdr:
        props += ["vhdr=shdr-raw"]      # Raw SHDR: IFE acquires is_shdr=1, demuxes the 2 VCs
    if exposure_ns is not None or iso is not None:
        props += ["control-mode=off"]
    if exposure_ns is not None:
        props += ["exposure-mode=off", "manual-exposure-time=%d" % int(exposure_ns)]
    if iso is not None:
        # manual-iso-value ONLY takes effect with iso-mode=manual; range 100..3200.
        # (Setting the value without iso-mode=manual left an inconsistent 3A state.)
        props += ["iso-mode=manual", "manual-iso-value=%d" % max(100, min(3200, int(iso)))]
    last_err = ""
    for _attempt in range(retries + 1):
        tmp = tempfile.mkdtemp(prefix="iq9raw_")
        pat = os.path.join(tmp, "f_%03d.bin")
        cam = [] if camera == 0 else ["camera=%d" % camera]
        # eos-after must be n_frames+1: `identity eos-after=N` fires EOS as the Nth buffer
        # passes, and that EOS races the sink -> the Nth buffer is often torn down unwritten.
        # Grabbing one extra buffer guarantees n_frames complete files (verified: eos-after=1
        # yields 0 bytes; eos-after=2 yields a full frame).
        argv = (["gst-launch-1.0", "-e", "qtiqmmfsrc"] + cam + props +
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
            f = _destride_raw16(a, width, height)     # 1.7 & 2.0 (stride-padded) safe
            if f is not None:
                frames.append(f)
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
                            "n": len(frames), "bit_depth": 12, "cfa": "RGGB",
                            "exposure_ns": exposure_ns, "iso": iso,
                            "ae": (exposure_ns is None and iso is None)}
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
    def __init__(self, width=1920, height=1080, fps=30, mode="nv12", camera=0,
                 exposure_ns=None, iso=None):
        _ensure_gst()
        self.w, self.h, self.fps, self.mode = width, height, fps, mode
        self.exposure_ns, self.iso = exposure_ns, iso
        # qtiqmmfsrc element props built at CONSTRUCTION. Manual exposure/gain is set here
        # (a fresh pipeline already in control-mode=off) rather than toggled live -- the live
        # 3A-mode transition is what tripped the IFE SMMU fault on this stack; starting in
        # manual avoids that transition. props must end with a space (it precedes '!').
        props = "" if camera == 0 else ("camera=%d " % camera)
        if exposure_ns is not None or iso is not None:
            props += "control-mode=off "
        if exposure_ns is not None:
            props += "exposure-mode=off manual-exposure-time=%d " % int(exposure_ns)
        if iso is not None:
            props += "iso-mode=manual manual-iso-value=%d " % max(100, min(3200, int(iso)))
        if mode == "nv12":
            desc = ("qtiqmmfsrc name=c %s! video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 "
                    "! videoconvert ! video/x-raw,format=BGRx "
                    "! appsink name=s max-buffers=2 drop=true sync=false"
                    % (props, width, height, fps))
        elif mode == "bayer":
            # RAW16 Bayer -> uint16. bpp MUST be a caps string; RAW16 (not RAW10/12).
            # Prefer grab_raw16() (isolated subprocess); this in-process path is for tools
            # that own the camera exclusively.
            desc = ("qtiqmmfsrc name=c %s! video/x-bayer,format=rggb,bpp=(string)16,"
                    "width=%d,height=%d,framerate=%d/1 "
                    "! appsink name=s max-buffers=2 drop=true sync=false"
                    % (props, width, height, fps))
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
            # bayer (RAW16): uint16 LE. 1.7 stride == width; 2.0 pads the stride -> de-pad.
            a = np.frombuffer(mi.data, dtype="<u2")
            return _destride_raw16(a, w, h)
        finally:
            buf.unmap(mi)
