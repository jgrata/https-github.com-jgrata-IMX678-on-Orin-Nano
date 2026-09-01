"""IQ9 dual-pad camera daemon -- ONE qtiqmmfsrc, NV12 + bayer RAW16 from a single session.

QLI 2.0 wedges the sensor on any camera open/close (CCI/I2C fault: REG_bank unlock / CCI not
enabled / i2c -22 @ slave 0x9e). This daemon holds ONE persistent qtiqmmfsrc with two request
pads so NV12 (live view / field-map) and RAW16 (characterization) coexist with NO reopen -- the
RAW path no longer has to kill the NV12 worker, so the wedge can't trigger.

Publishes latest frames to POSIX shm (atomic rename -> no torn frames):
  /dev/shm/iq9_nv12   BGR    : magic 'IQ9N' + u32 w + u32 h + u32 seq + BGR bytes (w*h*3)
  /dev/shm/iq9_raw    RAW16  : magic 'IQ9R' + u32 w + u32 h + u32 seq + u16 LE bytes (w*h*2)
RAW is published ONLY while /dev/shm/iq9_raw.on exists (the webui touches it for a capture burst),
so setup/live view doesn't churn 16 MB/frame. Polls /dev/shm/iq9_nv12.ctl for exposure-comp.

Env: IQ9_W IQ9_H IQ9_FPS IQ9_CAM IQ9_SHM IQ9_RAW_SHM IQ9_EXP_NS IQ9_ISO
"""
import json
import os
import struct
import tempfile
import time

import numpy as np

import camera_qmmf
import cam_meta

W = int(os.environ.get("IQ9_W", "1920"))
H = int(os.environ.get("IQ9_H", "1080"))
FPS = int(os.environ.get("IQ9_FPS", "30"))
CAM = int(os.environ.get("IQ9_CAM", "0"))
NV_SHM = os.environ.get("IQ9_SHM", "/dev/shm/iq9_nv12")
RAW_SHM = os.environ.get("IQ9_RAW_SHM", "/dev/shm/iq9_raw")
RAW_ON = RAW_SHM + ".on"
CTL = NV_SHM + ".ctl"
NV_MAGIC = b"IQ9N"
RAW_MAGIC = b"IQ9R"
EXP_NS = os.environ.get("IQ9_EXP_NS")
ISO = os.environ.get("IQ9_ISO")

# RTSP: HW-encoded streams teed off the SAME camera session (see camera_qmmf.DualCapture). Gated so
# a bad encode config can't permanently break the core camera (set IQ9_RTSP=0 + reboot to recover).
# IQ9_RTSP=1 -> H264+H265 1080p off video_0 (no extra camera stream). IQ9_RTSP_4K=1 -> add a
# dedicated 4k H264 stream on video_2 (an EXTRA camera stream -- verify CamX/Venus capacity).
RTSP = os.environ.get("IQ9_RTSP", "1") not in ("0", "false", "False")
RTSP_4K = os.environ.get("IQ9_RTSP_4K", "0") == "1"
_ENC_IO = "capture-io-mode=dmabuf output-io-mode=dmabuf-import"    # zero-copy import of the ISP dmabuf


def _rtsp_config():
    streams, s4k = [], None
    if RTSP:
        streams = [
            {"enc": "v4l2h264enc " + _ENC_IO, "parse": "h264parse", "port": 8554, "mpoint": "/h264-1080"},
            {"enc": "v4l2h265enc " + _ENC_IO, "parse": "h265parse", "port": 8555, "mpoint": "/h265-1080"},
        ]
        if RTSP_4K:
            s4k = {"enc": "v4l2h264enc " + _ENC_IO, "parse": "h264parse", "port": 8556,
                   "mpoint": "/h264-4k", "width": 3840, "height": 2160}
    return streams, s4k

NV_META = NV_SHM + ".meta"                # per-frame provenance sidecar (JSON): seq, pts, actual
RAW_META = RAW_SHM + ".meta"
CAM_BIN = "/dev/shm/iq9_cammeta.bin"       # latest serialized camera_metadata_t (webui may re-parse)
CAM_DBG = "/dev/shm/iq9_cammeta.dbg"       # one-time probe report (validate the ctypes chain)


def _publish(path, magic, arr, w, h, seq):
    hdr = magic + struct.pack("<III", w, h, seq)
    d = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(dir=d, prefix=".pub_")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(hdr)
            f.write(arr.tobytes())
        os.replace(tmp, path)                          # atomic on tmpfs
    except Exception:
        try:
            os.remove(tmp)
        except OSError:
            pass


def _write_atomic(path, data):
    """Atomic write of bytes (or str) to a tmpfs path. Never raises."""
    try:
        if isinstance(data, str):
            data = data.encode("utf-8", "replace")
        d = os.path.dirname(path) or "."
        fd, tmp = tempfile.mkstemp(dir=d, prefix=".pub_")
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        os.replace(tmp, path)
    except Exception:
        try:
            os.remove(tmp)
        except Exception:
            pass


def _publish_meta(path, seq, pts_ns, cam_raw):
    """Write the per-frame provenance sidecar: sensor timestamp + parsed CamX 'actual' values.
    Parsing is best-effort (cam_meta never raises); pts_ns is always present."""
    rec = {"seq": seq, "pts_ns": pts_ns, "host_ns": time.monotonic_ns(),
           "host_epoch_ns": time.time_ns()}
    try:
        if cam_raw:
            md = cam_meta.parse(cam_raw)
            rec["cam"] = cam_meta.decode(md)
            rec["cam"]["_layout_ok"] = md.get("_layout_ok", False)
    except Exception:
        pass
    _write_atomic(path, json.dumps(rec))


def _probe_dump(cam_raw, pts_ns):
    """One-time diagnostic: confirm the ctypes getbuffer chain + camera_metadata_t layout and
    list every entry (tag/type/count/value) so the SENSOR-section tag indices can be verified
    against real data before the webui parser trusts the name map. Written once to CAM_DBG."""
    lines = ["IQ9 cam-meta probe", "pts_ns=%s" % pts_ns,
             "cam_raw=%s bytes" % (len(cam_raw) if cam_raw else None)]
    try:
        if cam_raw:
            lines.append("header hex: " + cam_raw[:64].hex())
            md = cam_meta.parse(cam_raw)
            for k in ("_layout_ok", "_error", "_size", "_version", "_entry_count",
                      "_data_count", "_vendor_id", "_entries_start", "_data_start"):
                if k in md:
                    lines.append("%s=%s" % (k, md[k]))
            lines.append("--- entries (tag_hex type count -> values[:6]) ---")
            for tag, e in sorted(md.get("by_tag", {}).items()):
                nm = e.get("name") or ""
                lines.append("0x%08x %-8s n=%-3d %-28s %s" % (
                    tag, e["type_name"], e["count"], nm, e["values"][:6]))
            lines.append("--- decoded ---")
            lines.append(json.dumps(cam_meta.decode(md), indent=2))
    except Exception as e:
        lines.append("probe exception: %r" % (e,))
    _write_atomic(CAM_DBG, "\n".join(lines) + "\n")


def main():
    rtsp_streams, rtsp_4k = _rtsp_config()
    cam = camera_qmmf.DualCapture(
        nv_w=W, nv_h=H, fps=FPS, camera=CAM,
        exposure_ns=int(EXP_NS) if EXP_NS else None,
        iso=int(ISO) if ISO else None,
        rtsp_streams=rtsp_streams, rtsp_4k=rtsp_4k).start()
    _write_atomic("/dev/shm/iq9_rtsp.json", json.dumps({
        "enabled": bool(rtsp_streams), "streams": [
            {"codec": s["parse"].replace("parse", ""), "port": s["port"], "mpoint": s["mpoint"],
             "resolution": ("3840x2160" if s is rtsp_4k else "%dx%d" % (W, H))}
            for s in (rtsp_streams + ([rtsp_4k] if rtsp_4k else []))]}))
    # qtirtspbin's embedded RTSP server services client requests from GLib main-loop callbacks; our
    # capture loop below uses BLOCKING appsink pulls (no main loop), so the server would accept the
    # TCP connection but never answer OPTIONS/DESCRIBE. Run a GLib main loop in a daemon thread so
    # the RTSP server (and any other GSource) is dispatched.
    if rtsp_streams or rtsp_4k:
        try:
            import threading
            from gi.repository import GLib
            threading.Thread(target=GLib.MainLoop().run, daemon=True).start()
        except Exception:
            pass
    time.sleep(1.2)                                    # 3A settle
    nseq = 0
    rseq = 0
    applied_ec = None
    last_ctl = 0.0
    probed = False
    try:
        while True:
            bgr, pts = cam.nv12_frame(timeout_s=3.0)
            if bgr is not None:
                nseq += 1
                _publish(NV_SHM, NV_MAGIC, bgr, bgr.shape[1], bgr.shape[0], nseq)
                cam_raw, _ = cam.latest_cam_raw()
                _publish_meta(NV_META, nseq, pts, cam_raw)
                if cam_raw:
                    _write_atomic(CAM_BIN, cam_raw)    # latest raw meta (webui may re-parse)
                if not probed and nseq >= 3:           # one-time layout/chain validation
                    probed = True
                    _probe_dump(cam_raw, pts)
            if os.path.exists(RAW_ON):                 # RAW only during a capture burst
                raw, rpts = cam.raw_frame(timeout_s=1.0)
                if raw is not None:
                    rseq += 1
                    _publish(RAW_SHM, RAW_MAGIC, raw, raw.shape[1], raw.shape[0], rseq)
                    cam_raw, _ = cam.latest_cam_raw()
                    _publish_meta(RAW_META, rseq, rpts, cam_raw)
            now = time.monotonic()
            if now - last_ctl > 1.0:
                last_ctl = now
                try:
                    if os.path.exists(CTL):
                        c = json.load(open(CTL))
                        ec = c.get("exposure_compensation")
                        if ec is not None and ec != applied_ec:
                            if cam.set_prop("exposure-compensation", int(ec)):
                                applied_ec = ec
                except Exception:
                    pass
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
