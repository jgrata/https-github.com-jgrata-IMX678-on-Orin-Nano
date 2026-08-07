"""IQ9 NV12 live-view worker — owns qtiqmmfsrc in ITS OWN process.

Why a separate process: an in-process `Gst NULL` does NOT fully disconnect the
cam-server recorder client, so the camera stays claimed and a subsequent RAW/RDI
capture gets no frame. Fully KILLING this process releases the camera cleanly (the
only path that reliably yields RAW frames). The server therefore runs NV12 here and
SIGKILLs this worker for the RAW window, then respawns it.

Publishes the latest frame as raw BGR to POSIX shm (/dev/shm) via atomic rename, and
polls a small JSON control file for live tweaks (exposure-compensation). BGR (not
JPEG) so the MTF slanted-edge and ColorChecker ΔE keep full fidelity.

Env: IQ9_W IQ9_H IQ9_FPS IQ9_CAM IQ9_SHM (default /dev/shm/iq9_nv12)
Frame file layout: magic 'IQ9N' + u32 width + u32 height + u32 seq + BGR bytes (w*h*3).
"""
import json
import os
import struct
import tempfile
import time

import camera_qmmf

W = int(os.environ.get("IQ9_W", "1920"))
H = int(os.environ.get("IQ9_H", "1080"))
FPS = int(os.environ.get("IQ9_FPS", "30"))
CAM = int(os.environ.get("IQ9_CAM", "0"))
SHM = os.environ.get("IQ9_SHM", "/dev/shm/iq9_nv12")
CTL = SHM + ".ctl"
MAGIC = b"IQ9N"
# Optional construction-time manual exposure/gain for the NV12 live path (None -> 3A auto).
EXP_NS = os.environ.get("IQ9_EXP_NS")   # manual exposure, nanoseconds
ISO = os.environ.get("IQ9_ISO")         # manual ISO/gain, 100..3200


def _publish(bgr, seq):
    h, w = bgr.shape[:2]
    hdr = MAGIC + struct.pack("<III", w, h, seq)
    d = os.path.dirname(SHM) or "."
    fd, tmp = tempfile.mkstemp(dir=d, prefix=".nv12_")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(hdr)
            f.write(bgr.tobytes())
        os.replace(tmp, SHM)                       # atomic on tmpfs -> server never sees a torn frame
    except Exception:
        try:
            os.remove(tmp)
        except OSError:
            pass


def main():
    cam = camera_qmmf.QmmfCapture(
        W, H, FPS, mode="nv12", camera=CAM,
        exposure_ns=int(EXP_NS) if EXP_NS else None,
        iso=int(ISO) if ISO else None).start()
    time.sleep(1.2)                                # 3A settle
    seq = 0
    applied_ec = None
    last_ctl = 0.0
    try:
        while True:
            bgr = cam.frame(timeout_s=3.0)
            if bgr is not None:
                seq += 1
                _publish(bgr, seq)
            now = time.monotonic()
            if now - last_ctl > 1.0:               # poll control file ~1 Hz
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
