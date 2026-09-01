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


def main():
    cam = camera_qmmf.DualCapture(
        nv_w=W, nv_h=H, fps=FPS, camera=CAM,
        exposure_ns=int(EXP_NS) if EXP_NS else None,
        iso=int(ISO) if ISO else None).start()
    time.sleep(1.2)                                    # 3A settle
    nseq = 0
    rseq = 0
    applied_ec = None
    last_ctl = 0.0
    try:
        while True:
            bgr = cam.nv12_frame(timeout_s=3.0)
            if bgr is not None:
                nseq += 1
                _publish(NV_SHM, NV_MAGIC, bgr, bgr.shape[1], bgr.shape[0], nseq)
            if os.path.exists(RAW_ON):                 # RAW only during a capture burst
                raw = cam.raw_frame(timeout_s=1.0)
                if raw is not None:
                    rseq += 1
                    _publish(RAW_SHM, RAW_MAGIC, raw, raw.shape[1], raw.shape[0], rseq)
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
