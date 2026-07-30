"""RAW Bayer characterization + preview for the IQ9 IMX678 (12-bit RGGB, 0..4095).

Operates on native RAW16 frames from camera_qmmf.grab_raw16() -- HxW uint16, RGGB
CFA (R at (0,0), B at (1,1)), 12-bit data in a 16-bit container. These are the
linear-RAW primitives the colour-science needs (black level, saturation, per-channel
response) that the ISP/NV12 path hides.
"""
import numpy as np

MAXV = 4095            # 12-bit full scale
SAT = 4095             # saturation code


def split_rggb(raw):
    """Half-resolution channel planes from an RGGB mosaic: (R, G1, G2, B)."""
    return (raw[0::2, 0::2], raw[0::2, 1::2], raw[1::2, 0::2], raw[1::2, 1::2])


def characterize(raw):
    """Per-channel + global RAW statistics for sensor characterization."""
    r, g1, g2, b = (p.astype(np.float64) for p in split_rggb(raw))
    def st(p):
        return {"mean": float(p.mean()), "std": float(p.std()),
                "min": int(p.min()), "max": int(p.max()),
                "sat_frac": float((p >= SAT).mean())}
    chans = {"R": st(r), "G1": st(g1), "G2": st(g2), "B": st(b)}
    flat = raw.reshape(-1)
    # black-level estimate: mean of the darkest 0.5% of pixels per channel
    def blk(p):
        f = np.sort(p.reshape(-1))
        k = max(1, int(f.size * 0.005))
        return float(f[:k].mean())
    black = {"R": blk(r), "G1": blk(g1), "G2": blk(g2), "B": blk(b)}
    hist, edges = np.histogram(flat, bins=128, range=(0, MAXV + 1))
    return {
        "channels": chans,
        "black_level": black,
        "black_level_mean": float(np.mean(list(black.values()))),
        "green_balance": float(abs(g1.mean() - g2.mean())),   # G1~G2 sanity (fixed-pattern)
        "global": {"min": int(flat.min()), "max": int(flat.max()),
                   "mean": float(flat.mean()),
                   "sat_frac": float((flat >= SAT).mean()),
                   "black_clip_frac": float((flat <= 0).mean())},
        "maxv": MAXV,
        "hist": {"counts": hist.tolist(), "edges": edges.astype(int).tolist()},
        "shape": [int(raw.shape[0]), int(raw.shape[1])],
    }


def preview_bgr8(raw, out_w=960, wb=True, gamma=2.2, black=None):
    """Fast display preview: 2x2-bin the mosaic to half-res RGB (no demosaic-convention
    ambiguity), optional gray-world WB, robust normalize + gamma -> 8-bit BGR."""
    r, g1, g2, b = (p.astype(np.float32) for p in split_rggb(raw))
    g = 0.5 * (g1 + g2)
    if black is not None:
        r = np.clip(r - black.get("R", 0), 0, None)
        g = np.clip(g - 0.5 * (black.get("G1", 0) + black.get("G2", 0)), 0, None)
        b = np.clip(b - black.get("B", 0), 0, None)
    if wb:
        gm = g.mean() + 1e-6
        r *= gm / (r.mean() + 1e-6)
        b *= gm / (b.mean() + 1e-6)
    rgb = np.dstack([r, g, b])                       # half-res, linear
    norm = np.percentile(rgb, 99.5) or (rgb.max() + 1e-6)
    rgb = np.clip(rgb / norm, 0, 1) ** (1.0 / gamma)
    bgr = (rgb[:, :, ::-1] * 255).astype(np.uint8)   # RGB->BGR for cv2
    if out_w and bgr.shape[1] > out_w:
        import cv2
        bgr = cv2.resize(bgr, (out_w, max(1, int(bgr.shape[0] * out_w / bgr.shape[1]))),
                         interpolation=cv2.INTER_AREA)
    return bgr
