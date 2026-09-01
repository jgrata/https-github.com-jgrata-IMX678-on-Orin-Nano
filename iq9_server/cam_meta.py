"""Parse the serialized CamX ``camera_metadata_t`` blob into a Python dict.

The IQ9 ``qtiqmmfsrc`` delivers per-frame CamX result metadata as a ``qmmf::CameraMetadata``
(wraps Android's ``camera_metadata_t``). The daemon (camera_qmmf) extracts the raw serialized
buffer via ctypes ``getbuffer()`` and hands the bytes here. This module is PURE PYTHON for the
PARSING (no gi, no camera) so it runs on the board AND on a dev PC against a captured blob --
the parser is iterated with NO daemon reboot. Tag NAMES are resolved authoritatively at runtime
from the board's ``libcamera_metadata`` (falls back to a built-in map of the tags we surface).

Serialized ``camera_metadata_t`` layout on this build (QCS9075 / lemans) -- VERIFIED against a
real 41512-byte blob: an all-uint32 header (48 bytes), an entry table (16 B each), then a data
blob. Header:
    off  type   field           (example)
    0    u32    size             41512  (== len(buf))
    4    u32    version          1
    8    u32    flags            1
    12   u32    entry_count      152
    16   u32    entry_capacity   152
    20   u32    entries_start    48     (entry table begins here)
    24   u32    data_count       39032
    28   u32    data_capacity    39032
    32   u32    data_start       2480   (data blob begins here)
    36   u32    padding          0
    40   u64    vendor_id        0xffffffffffffffff (none)
Entry (camera_metadata_buffer_entry_t, 16 B): tag u32@0, count u32@4, data u32@8 (offset into the
data blob, OR up to 4 inline bytes when count*elem_size <= 4), type u8@12, reserved[3]@13.
type: 0 byte(1) 1 int32(4) 2 float(4) 3 int64(8) 4 double(8) 5 rational(8 = 2×int32).
"""
import ctypes
import struct

_TYPE_SIZE = {0: 1, 1: 4, 2: 4, 3: 8, 4: 8, 5: 8}
_TYPE_NAME = {0: "byte", 1: "int32", 2: "float", 3: "int64", 4: "double", 5: "rational"}

HEADER_SIZE = 48

# Tags we surface as friendly 'actual' fields. Numbers are the stable Android tag ids (section base
# + index), CONFIRMED via libcamera_metadata_lemans on this board. Vendor tags (0x8xxxxxxx) are left
# raw. Also used as the offline fallback name map.
T_CCM              = 0x00000001   # color_correction.transform  rational[9]
T_AWB_GAINS        = 0x00000002   # color_correction.gains       float[4]  (R,Gr,Gb,B)
T_CC_MODE          = 0x00000000   # color_correction.mode        byte
T_CONTROL_MODE     = 0x00010000   # control.mode                 byte
T_AE_MODE          = 0x00010001   # control.aeMode
T_AWB_MODE         = 0x00010013   # control.awbMode (resolved at runtime when present)
T_CAPTURE_INTENT   = 0x0001000D   # control.captureIntent        byte
T_CROP_REGION      = 0x000D0000   # scaler.cropRegion            int32[4]
T_FRAME_COUNT      = 0x000C0000   # request.frameCount           int32
T_EXPOSURE_TIME    = 0x000E0000   # sensor.exposureTime          int64 ns
T_FRAME_DURATION   = 0x000E0001   # sensor.frameDuration         int64 ns
T_SENSITIVITY      = 0x000E0002   # sensor.sensitivity           int32 (ISO)
T_TIMESTAMP        = 0x000E0010   # sensor.timestamp             int64 ns (SOF)
T_ROLLING_SKEW     = 0x000E001A   # sensor.rollingShutterSkew    int64 ns
T_DYN_BLACK_LEVEL  = 0x000E001C   # sensor.dynamicBlackLevel     float[4]
T_DYN_WHITE_LEVEL  = 0x000E001D   # sensor.dynamicWhiteLevel     int32

_FALLBACK_NAMES = {
    T_CCM: "color_correction.transform", T_AWB_GAINS: "color_correction.gains",
    T_CC_MODE: "color_correction.mode", T_CONTROL_MODE: "control.mode",
    T_AE_MODE: "control.aeMode", T_CAPTURE_INTENT: "control.captureIntent",
    T_CROP_REGION: "scaler.cropRegion", T_FRAME_COUNT: "request.frameCount",
    T_EXPOSURE_TIME: "sensor.exposureTime", T_FRAME_DURATION: "sensor.frameDuration",
    T_SENSITIVITY: "sensor.sensitivity", T_TIMESTAMP: "sensor.timestamp",
    T_ROLLING_SKEW: "sensor.rollingShutterSkew", T_DYN_BLACK_LEVEL: "sensor.dynamicBlackLevel",
    T_DYN_WHITE_LEVEL: "sensor.dynamicWhiteLevel",
}

# ---- authoritative tag-name resolver (board libcamera_metadata) ------------------------
_NAME_FN = None
_NAME_TRIED = False


def _name_fn():
    """Cached ctypes get_camera_metadata_tag_name(uint) -> const char*, or None if unavailable.
    QCS9075 is the 'lemans' family; try that first, then generic names."""
    global _NAME_FN, _NAME_TRIED
    if _NAME_TRIED:
        return _NAME_FN
    _NAME_TRIED = True
    for lib in ("libcamera_metadata_lemans.so.0", "libcamera_metadata.so.0",
                "libcamera_metadata.so"):
        try:
            L = ctypes.CDLL(lib)
            fn = L.get_camera_metadata_tag_name
            fn.restype = ctypes.c_char_p
            fn.argtypes = [ctypes.c_uint]
            _NAME_FN = fn
            return fn
        except Exception:
            continue
    return None


def tag_name(tag):
    """Human name for a tag: authoritative (board lib) if possible, else the built-in fallback."""
    fn = _name_fn()
    if fn is not None:
        try:
            n = fn(tag)
            if n:
                return n.decode("ascii", "replace")
        except Exception:
            pass
    return _FALLBACK_NAMES.get(tag)


def parse(buf):
    """Parse a serialized camera_metadata_t. Returns a dict; never raises.
    { _layout_ok, _size, _version, _entry_count, _data_count, _vendor_id,
      by_tag: { tag: {name, type, type_name, count, values} },
      <friendly names>: value }"""
    out = {"_layout_ok": False, "by_tag": {}}
    try:
        if buf is None or len(buf) < HEADER_SIZE:
            out["_error"] = "buf too small (%s)" % (None if buf is None else len(buf))
            return out
        (size, version, flags, entry_count, entry_capacity, entries_start,
         data_count, data_capacity, data_start, padding) = struct.unpack_from("<10I", buf, 0)
        vendor_id = struct.unpack_from("<Q", buf, 40)[0]
        out.update({"_size": size, "_version": version, "_entry_count": entry_count,
                    "_data_count": data_count, "_vendor_id": vendor_id,
                    "_entries_start": entries_start, "_data_start": data_start})
        if size != len(buf) or entry_count > 8192 or entries_start + entry_count * 16 > len(buf):
            out["_error"] = "layout mismatch size=%d len=%d ec=%d es=%d" % (
                size, len(buf), entry_count, entries_start)
            return out
        for i in range(entry_count):
            eoff = entries_start + i * 16
            tag, count = struct.unpack_from("<II", buf, eoff)
            etype = buf[eoff + 12]
            elem = _TYPE_SIZE.get(etype, 0)
            vals = []
            if elem:
                total = count * elem
                doff = (eoff + 8) if total <= 4 else (data_start + struct.unpack_from("<I", buf, eoff + 8)[0])
                if 0 <= doff and doff + total <= len(buf):
                    vals = _read_values(buf, doff, etype, count)
            ent = {"name": tag_name(tag), "type": etype, "type_name": _TYPE_NAME.get(etype, "?"),
                   "count": count, "values": vals}
            out["by_tag"][tag] = ent
        out["_layout_ok"] = True
    except Exception as e:
        out["_error"] = "parse exception: %r" % (e,)
    return out


def _read_values(buf, off, etype, count):
    fmt = {0: "b", 1: "i", 2: "f", 3: "q", 4: "d"}.get(etype)
    if fmt is not None:
        return list(struct.unpack_from("<" + fmt * count, buf, off))
    if etype == 5:                                       # rational num/den (2×int32)
        r = struct.unpack_from("<" + "i" * (2 * count), buf, off)
        return [(r[2 * k], r[2 * k + 1]) for k in range(count)]
    return []


def _v(md, tag, scalar=True):
    e = md.get("by_tag", {}).get(tag)
    if not e or not e["values"]:
        return None
    return e["values"][0] if (scalar and len(e["values"]) == 1) else e["values"]


def decode(md):
    """Friendly 'actual' view of the CamX result (achieved sensor/ISP values). Never raises."""
    d = {}
    try:
        if not md.get("_layout_ok"):
            return {"_available": False, "_error": md.get("_error")}
        d["_available"] = True
        exp = _v(md, T_EXPOSURE_TIME)
        if exp is not None:
            d["exposure_ns"] = exp
        fd = _v(md, T_FRAME_DURATION)
        if fd is not None:
            d["frame_duration_ns"] = fd
        iso = _v(md, T_SENSITIVITY)
        if iso is not None:
            d["iso"] = iso
        ts = _v(md, T_TIMESTAMP)
        if ts is not None:
            d["sensor_timestamp_ns"] = ts
        sk = _v(md, T_ROLLING_SKEW)
        if sk is not None:
            d["rolling_shutter_skew_ns"] = sk
        bl = _v(md, T_DYN_BLACK_LEVEL, scalar=False)
        if bl is not None:
            d["dynamic_black_level"] = bl
        wl = _v(md, T_DYN_WHITE_LEVEL)
        if wl is not None:
            d["dynamic_white_level"] = wl
        crop = _v(md, T_CROP_REGION, scalar=False)
        if crop is not None:
            d["crop_region"] = crop
        fc = _v(md, T_FRAME_COUNT)
        if fc is not None:
            d["frame_count"] = fc
        ci = _v(md, T_CAPTURE_INTENT)
        if ci is not None:
            d["capture_intent"] = ci
        cm = _v(md, T_CONTROL_MODE)
        if cm is not None:
            d["control_mode"] = cm
        ccm = _v(md, T_CCM, scalar=False)
        if ccm and isinstance(ccm, list):
            d["ccm"] = [round(n / de, 6) if de else 0.0 for (n, de) in ccm]   # 9 rationals -> floats
        gains = _v(md, T_AWB_GAINS, scalar=False)
        if gains:
            d["awb_gains"] = [round(g, 6) for g in gains]
    except Exception as e:
        d["_error"] = repr(e)
    return d
