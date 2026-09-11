"""Local zero-copy reader for raw_capture's shared-memory frame ring.

raw_capture publishes each decoded uint16 Bayer frame into /dev/shm/metro_raw
(see ShmPublisher in raw_capture.cpp). LOCAL consumers on the Jetson read frames
straight from the mmap -- no RAW10 pack, no localhost TCP, no NumPy unpack -- which
is the only way to a full-rate 4K RAW path here (the 1 GbE port caps remote RAW).

Layout (little-endian, packed), matching the C++ structs exactly:
  ShmHeader @0 (64 B reserved): magic,ver,nslots,slot_stride,max_w,max_h,data_off,
                                _pad (8x u32), latest_seq (u64), latest_slot (u32), _pad2
  slot @ 64 + i*slot_stride: ShmSlotMeta{seq u64; w,h,bpp,_pad u32; exp_ns,sof_ns u64;
                                         gain,capture_time f64} then pixels @ +data_off
A published `latest_seq` + 4 slots give a reader ~4 frame-periods to copy a slot.
"""
import mmap
import os
import struct

import numpy as np

SHM_PATH = os.environ.get("METRO_SHM_PATH", "/dev/shm/metro_raw")
_MAGIC = 0x5741524D
_HDR = struct.Struct("<8IQII")        # 48 B used; slots start at HDR_SZ=64
_META = struct.Struct("<Q4IQQdd")     # 56 B
_HDR_SZ = 64


class ShmUnavailable(RuntimeError):
    pass


class ShmReader:
    def __init__(self, path=SHM_PATH):
        if not os.path.exists(path):
            raise ShmUnavailable("shm not present: %s (is raw_capture running?)" % path)
        self.path = path
        self._f = open(path, "rb")
        self._mm = mmap.mmap(self._f.fileno(), 0, prot=mmap.PROT_READ)
        magic = _HDR.unpack_from(self._mm, 0)[0]
        if magic != _MAGIC:
            self.close()
            raise ShmUnavailable("bad shm magic 0x%08x" % magic)

    def close(self):
        try:
            self._mm.close()
        finally:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _hdr(self):
        (magic, ver, nslots, stride, maxw, maxh, data_off, _pad,
         seq, slot, _pad2) = _HDR.unpack_from(self._mm, 0)
        return nslots, stride, data_off, seq, slot

    def latest(self, copy=True, retries=5):
        """Latest complete frame as a dict (frame = HxW uint16 Bayer), or None.
        copy=False returns a live view into the mmap (zero-copy, but only valid
        until the writer laps this slot -- copy before any slow work)."""
        for _ in range(retries):
            nslots, stride, data_off, seq, slot = self._hdr()
            if seq == 0:
                return None
            soff = _HDR_SZ + slot * stride
            mseq, w, h, bpp, _p, exp_ns, sof_ns, gain, ctime = _META.unpack_from(self._mm, soff)
            n = w * h
            arr = np.frombuffer(self._mm, np.uint16, count=n, offset=soff + data_off).reshape(h, w)
            seq2 = self._hdr()[3]
            if mseq == seq == seq2:                    # no tear
                return {"seq": seq, "w": w, "h": h, "bpp": bpp, "exp_ns": exp_ns,
                        "sof_ns": sof_ns, "gain": gain, "capture_time": ctime,
                        "frame": arr.copy() if copy else arr,
                        "maxv": float((1 << bpp) - 1)}
        return None


def sensor_timing(duration_s=2.0, sleep_s=0.0):
    """Measure true sensor frame timing from the shm ring. Samples latest_seq +
    sof_ns as fast as possible; because we track BOTH the seq delta and the sensor
    timestamp delta, the per-frame period is exact even if we sample sparsely (or
    the capture loop drops frames). time.monotonic is fine here (FastAPI process)."""
    import time
    r = ShmReader()
    try:
        samples = []                                   # (seq, sof_ns)
        t_end = time.monotonic() + duration_s
        last_seq = None
        while time.monotonic() < t_end:
            f = r.latest(copy=False)
            if f is not None and f["seq"] != last_seq:
                samples.append((f["seq"], f["sof_ns"]))
                last_seq = f["seq"]
            if sleep_s:
                time.sleep(sleep_s)
    finally:
        r.close()
    if len(samples) < 3:
        return {"error": "too few frames (%d) -- is capture running?" % len(samples)}
    seqs = np.array([s for s, _ in samples], np.int64)
    sof = np.array([t for _, t in samples], np.int64)
    dseq = np.diff(seqs)
    dsof = np.diff(sof).astype(np.float64)             # ns
    ok = dseq > 0
    per_frame_ns = dsof[ok] / dseq[ok]                 # true sensor period (handles skips)
    per_ms = per_frame_ns / 1e6
    span = int(seqs[-1] - seqs[0])
    seen = len(samples)
    return {
        "frames_seen": seen,
        "seq_span": span,
        "dropped_by_reader": int(span - (seen - 1)),   # seq advanced more than we sampled
        "sensor_period_ms_median": round(float(np.median(per_ms)), 4),
        "sensor_period_ms_std": round(float(np.std(per_ms)), 4),
        "sensor_fps": round(1000.0 / float(np.median(per_ms)), 2) if np.median(per_ms) > 0 else None,
        "jitter_ms_p2p": round(float(per_ms.max() - per_ms.min()), 4),
    }


_MEAS = {"seq": None, "t": None, "fps": 0.0}

def measured_fps():
    """Delivered fps: rate the shm frame-seq counter advances, measured across
    calls (>=0.25 s window). Reflects the frames actually reaching consumers."""
    import time as _t
    try:
        with ShmReader() as r:
            seq = r._hdr()[3]
    except Exception:
        return _MEAS["fps"]
    now = _t.monotonic(); m = _MEAS
    if m["seq"] is None:
        m["seq"] = seq; m["t"] = now
    else:
        dt = now - m["t"]
        if dt >= 0.25:
            dseq = seq - m["seq"]
            if dseq >= 0:
                m["fps"] = dseq / dt
            m["seq"] = seq; m["t"] = now
    return round(m["fps"], 2)


if __name__ == "__main__":
    import time
    r = ShmReader()
    f = r.latest()
    if f is None:
        print("no frame yet")
    else:
        fr = f["frame"]
        print("frame seq=%d %dx%d bpp=%d exp=%.2fms gain=%.2f sof=%d  min/mean/max=%d/%.1f/%d"
              % (f["seq"], f["w"], f["h"], f["bpp"], f["exp_ns"] / 1e6, f["gain"],
                 f["sof_ns"], fr.min(), fr.mean(), fr.max()))
    print("timing:", sensor_timing(2.0))
    r.close()
