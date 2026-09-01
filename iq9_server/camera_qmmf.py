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
import struct
import subprocess
import tempfile

import numpy as np

try:
    import cam_meta                        # pure-Python camera_metadata_t parser (diagnostics)
except Exception:
    cam_meta = None

# Sensor-native RAW readout (Leopard IMX678 on this EVK): full RGGB, 12-bit.
# CAPS geometry the qtiqmmfsrc bayer stream accepts is 3856 x 2180 (the driver
# rejects 2176). The DECODED active image, however, is 3856 x 2176 (datasheet).
RAW_W, RAW_H, RAW_FPS = 3856, 2180, 30   # width, CAPS height, fps
RAW_H_ACTIVE = 2176                       # real active rows in the buffer


def _destride_raw16(a, width, height):
    """Reshape a flat RAW16 (uint16 LE) 'bayer' buffer to a (RAW_H_ACTIVE, width) RGGB array.

    The qtiqmmfsrc bayer(bpp=16) buffer is a PLAIN LINEAR raster whose pixel stride
    equals the active width (3856 px / 7712 B) with NO per-row padding; the active image
    is the first width*RAW_H_ACTIVE samples and the remainder is a trailing padding/
    metadata block. NOTE: on QLI 2.0 GstVideoMeta MISREPORTS the layout (stride 7728 B =
    3864 px, height 2180) — trusting it (or stride = buffer//height) shears the frame by
    ~15 px/row. Adjacent-row cross-correlation is flat only at stride == width, and the
    result matches the ColorChecker with no shear and correct RGGB order. The `height`
    arg (CAPS height, 2180) is ignored for the pixel layout. Verified on 2.0 (Aug 2026)."""
    n = int(a.size)
    rows = min(RAW_H_ACTIVE, n // width)
    if rows <= 0:
        return None
    return a[:width * rows].reshape(rows, width).copy()


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


# --- CamX result-metadata extraction (best-effort, never fatal) -------------------------
# qtiqmmfsrc emits the `result-metadata` signal per frame with a gpointer to a
# qmmf::CameraMetadata (wraps Android camera_metadata_t). There is NO Python binding / header
# for it, so we reach the raw serialized buffer via ctypes: call the (non-static) method
# qmmf::CameraMetadata::getbuffer() -> camera_metadata_t*, read its first 8 bytes (`size`),
# and copy that many bytes. cam_meta.parse() (pure Python) decodes it. If ANY step fails we
# just return None and the daemon keeps publishing pixels + PTS.
import ctypes                                                       # noqa: E402

_CAM_GETBUF = None
_CAM_GETBUF_TRIED = False


def _diag_write(path, text, append=False):
    """Best-effort diagnostic write (never raises)."""
    try:
        with open(path, "a" if append else "w") as f:
            f.write(text)
    except Exception:
        pass


# CONFIRMED PyGPointer layout on this build (GStreamer 1.28 / PyGObject): a raw G_TYPE_POINTER
# signal arg marshals to a `GPointer` wrapper whose int() raises. The wrapped C pointer sits at
# offset 16 in the CPython object (offset 24 holds the gtype = 0x44 = G_TYPE_POINTER, which
# confirmed offset 16 is the qmmf::CameraMetadata*). Feeding a WRONG `this` to getbuffer segfaults
# the whole process (uncatchable), so we ONLY read offset 16 and guard it looks like a userspace VA.
_CAM_META_PTR_OFF = 16


def _plausible_ptr(v):
    """A value that looks like a userspace VA (excludes small ints / obvious non-pointers)."""
    return isinstance(v, int) and 0x10000 <= v < (1 << 48)


def _extract_cam(ptr):
    """Serialized camera_metadata_t bytes from a result-metadata GPointer arg, or None."""
    if ptr is None:
        return None
    if isinstance(ptr, int):
        return _cam_getbuffer(ptr) if _plausible_ptr(ptr) else None
    try:
        cand = ctypes.c_void_p.from_address(id(ptr) + _CAM_META_PTR_OFF).value or 0
    except Exception:
        return None
    return _cam_getbuffer(cand) if _plausible_ptr(cand) else None


_GB_DIAG = False
CAM_GB_DBG = "/dev/shm/iq9_meta_gb.dbg"


def _cam_getbuffer(addr):
    """ctypes qmmf::CameraMetadata::getbuffer(this=addr) -> serialized camera_metadata_t bytes,
    or None. `addr` MUST be a valid CameraMetadata* (a wrong `this` segfaults the process). Dumps
    the returned pointer + header once to CAM_GB_DBG so the size-field type can be confirmed."""
    global _CAM_GETBUF, _CAM_GETBUF_TRIED, _GB_DIAG
    try:
        if not addr:
            return None
        if _CAM_GETBUF is None:
            if _CAM_GETBUF_TRIED:
                return None
            _CAM_GETBUF_TRIED = True
            lib = ctypes.CDLL("libqmmf_camera_metadata.so.1")
            fn = lib._ZN4qmmf14CameraMetadata9getbufferEv      # mangled: getbuffer()
            fn.restype = ctypes.c_void_p
            fn.argtypes = [ctypes.c_void_p]
            _CAM_GETBUF = fn
        raw_ptr = _CAM_GETBUF(ctypes.c_void_p(int(addr)))
        if not raw_ptr:
            if not _GB_DIAG:
                _GB_DIAG = True
                _diag_write(CAM_GB_DBG, "getbuffer(this=0x%x) -> NULL\n" % addr)
            return None
        head = ctypes.string_at(raw_ptr, 16)
        s32 = struct.unpack_from("<I", head, 0)[0]
        s64 = struct.unpack_from("<Q", head, 0)[0]
        if not _GB_DIAG:
            _GB_DIAG = True
            _diag_write(CAM_GB_DBG, "getbuffer(this=0x%x) -> raw_ptr=0x%x head16=%s s32=%d s64=%d\n"
                        % (addr, raw_ptr, head.hex(), s32, s64))
        for size in (s64, s32):                            # accept whichever size field is sane
            if 48 <= size <= (64 << 20):
                return ctypes.string_at(raw_ptr, int(size))
        return None
    except Exception as e:
        if not _GB_DIAG:
            _GB_DIAG = True
            _diag_write(CAM_GB_DBG, "getbuffer exc: %r\n" % (e,))
        return None


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


class DualCapture:
    """ONE qtiqmmfsrc with TWO request video pads -- NV12 (live view / field-map) AND bayer
    RAW16 (characterization) -- from a SINGLE persistent camera session. This is the fix for the
    QLI 2.0 CCI/I2C wedge: the sensor faults on any open-after-close (REG_bank unlock / CCI not
    enabled / i2c -22), so we never close -- both consumers pull from one session and the RAW path
    no longer has to kill the NV12 worker. Constraint: ONE qtiqmmfsrc per process (qmmfsrc_init
    asserts context != NULL), but that one element holds both pads. Verified: NV12 + RAW deliver
    simultaneously (2026-08-31)."""

    def __init__(self, nv_w=1920, nv_h=1080, raw_w=RAW_W, raw_h=RAW_H, fps=30, camera=0,
                 exposure_ns=None, iso=None, attach_meta=True):
        _ensure_gst()
        self.nv_w, self.nv_h, self.raw_w, self.raw_h = nv_w, nv_h, raw_w, raw_h
        self._cam_raw = None            # latest serialized camera_metadata_t bytes (from signal)
        self._cam_seq = 0               # increments each result-metadata callback
        props = "" if camera == 0 else ("camera=%d " % camera)
        # NOTE: attach-cam-meta is a PAD property (GstQmmfSrcVideoPad), NOT an element property
        # -- putting it on qtiqmmfsrc makes parse_launch fail. It's set on the pads below. The
        # per-frame CamX metadata is read from the `result-metadata` element signal regardless.
        if exposure_ns is not None or iso is not None:
            props += "control-mode=off "
        if exposure_ns is not None:
            props += "exposure-mode=off manual-exposure-time=%d " % int(exposure_ns)
        if iso is not None:
            props += "iso-mode=manual manual-iso-value=%d " % max(100, min(3200, int(iso)))
        desc = ("qtiqmmfsrc name=c %s"
                "c.video_0 ! video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 "
                "! videoconvert ! video/x-raw,format=BGRx "
                "! appsink name=nv max-buffers=2 drop=true sync=false "
                "c.video_1 ! video/x-bayer,format=rggb,bpp=(string)16,width=%d,height=%d,framerate=%d/1 "
                "! appsink name=raw max-buffers=2 drop=true sync=false"
                % (props, nv_w, nv_h, fps, raw_w, raw_h, fps))
        self.desc = desc
        self.pipe = Gst.parse_launch(desc)
        self.nv_sink = self.pipe.get_by_name("nv")
        self.raw_sink = self.pipe.get_by_name("raw")
        self.src = self.pipe.get_by_name("c")
        if attach_meta and self.src is not None:
            # Enable per-frame CamX result metadata: (1) connect the element `result-metadata`
            # signal (our read path -> _on_result_meta -> _extract_cam), (2) best-effort set the
            # PAD property attach-cam-meta=true on each video src pad. Both are non-fatal.
            try:
                self.src.connect("result-metadata", self._on_result_meta)
            except Exception:
                pass
            try:
                it = self.src.iterate_pads()
                while True:
                    res, pad = it.next()
                    if res != Gst.IteratorResult.OK:
                        break
                    try:
                        if pad.get_direction() == Gst.PadDirection.SRC and \
                           pad.find_property("attach-cam-meta") is not None:
                            pad.set_property("attach-cam-meta", True)
                    except Exception:
                        pass
            except Exception:
                pass

    def _on_result_meta(self, element, ptr, *user):
        """result-metadata signal: stash the latest serialized camera_metadata_t bytes.
        Runs on the qmmf callback thread; keep it minimal (copy bytes, return)."""
        try:
            self._cam_seq += 1
            raw = _extract_cam(ptr)
            if raw:
                self._cam_raw = raw
        except Exception:
            pass

    def latest_cam_raw(self):
        """Most recent serialized camera_metadata_t bytes (or None), and its callback seq."""
        return self._cam_raw, self._cam_seq

    def start(self):
        self.pipe.set_state(Gst.State.PLAYING)
        return self

    def stop(self):
        self.pipe.set_state(Gst.State.NULL)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    @staticmethod
    def _pts_ns(buf):
        try:
            pts = buf.pts
            return int(pts) if pts != Gst.CLOCK_TIME_NONE else None
        except Exception:
            return None

    def nv12_frame(self, timeout_s=5.0):
        """Latest NV12 frame as (BGR HxWx3 uint8, pts_ns), or (None, None)."""
        samp = self.nv_sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if samp is None:
            return None, None
        buf = samp.get_buffer()
        st = samp.get_caps().get_structure(0)
        w = st.get_value("width"); h = st.get_value("height")
        pts = self._pts_ns(buf)
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None, pts
        try:
            stride = mi.size // h                              # BGRx = 4 B/px, de-pad stride
            a = np.frombuffer(mi.data, np.uint8, count=stride * h).reshape(h, stride)
            return a[:, :w * 4].reshape(h, w, 4)[:, :, :3].copy(), pts
        finally:
            buf.unmap(mi)

    def raw_frame(self, timeout_s=5.0):
        """Latest RAW16 Bayer frame as ((H, W) uint16 de-strided, pts_ns), or (None, None)."""
        samp = self.raw_sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if samp is None:
            return None, None
        buf = samp.get_buffer()
        st = samp.get_caps().get_structure(0)
        w = st.get_value("width"); h = st.get_value("height")
        pts = self._pts_ns(buf)
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None, pts
        try:
            a = np.frombuffer(mi.data, dtype="<u2")
            return _destride_raw16(a, w, h), pts
        finally:
            buf.unmap(mi)

    def set_prop(self, name, value):
        try:
            if self.src is not None and self.src.find_property(name) is not None:
                self.src.set_property(name, value)
                return True
        except Exception:
            pass
        return False
