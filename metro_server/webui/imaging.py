"""Frame -> preview JPEG + histogram. Reuses the server's hdr.tonemap_rgb so the
web preview matches the rest of the pipeline. Fast path = 2x2 RGGB bin (half-res)
+ gray-world WB + Reinhard tonemap; that's plenty for a live focus/expose view and
keeps per-frame cost low on the Orin."""
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # metro_server/
import hdr  # noqa: E402  (server-side tonemap/demosaic)


def default_black_level(maxv):
    """Sensor pedestal (vendor optical black): ~200 DN @12-bit, ~50 @10-bit.
    Single source of truth for the web pipeline (preview / colorchecker / mtf)."""
    return 200.0 if maxv > 2000 else 50.0


def fast_preview(frame, maxv, out_width=960, black_level=None):
    """2x2 RGGB bin -> black-level subtract -> WB -> Reinhard tonemap -> BGR uint8."""
    if black_level is None:
        black_level = default_black_level(maxv)
    H, W = frame.shape
    He, We = (H // 2) * 2, (W // 2) * 2
    f = np.clip(frame[:He, :We].astype(np.float32) - black_level, 0, None)
    R = f[0::2, 0::2]
    Gr = f[0::2, 1::2]
    Gb = f[1::2, 0::2]
    B = f[1::2, 1::2]
    rgb = np.dstack([R, 0.5 * (Gr + Gb), B]) / (maxv - black_level)
    mR, mG, mB = (rgb[..., 0].mean(), rgb[..., 1].mean(), rgb[..., 2].mean())
    rgb[..., 0] *= mG / max(mR, 1e-6)
    rgb[..., 2] *= mG / max(mB, 1e-6)
    bgr = cv2.cvtColor(hdr.tonemap_rgb(rgb), cv2.COLOR_RGB2BGR)
    if out_width and bgr.shape[1] != out_width:
        h2 = max(1, int(round(bgr.shape[0] * out_width / bgr.shape[1])))
        bgr = cv2.resize(bgr, (out_width, h2), interpolation=cv2.INTER_AREA)
    return bgr


def encode_jpeg(bgr, quality=85):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


def histogram(frame, maxv, nbins=128):
    """Histogram of the raw Bayer frame (subsampled for speed)."""
    s = frame[::4, ::4]
    counts, _ = np.histogram(s, bins=nbins, range=(0, maxv))
    clip_hi = float((frame >= maxv).mean())      # fraction at full-scale (clipping)
    return {
        "counts": counts.astype(int).tolist(),
        "nbins": int(nbins),
        "maxv": float(maxv),
        "clip_frac": clip_hi,
        "p50": float(np.percentile(s, 50)),
        "p99": float(np.percentile(s, 99)),
    }
