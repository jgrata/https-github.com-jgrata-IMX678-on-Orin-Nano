"""ColorChecker detect + measure + CCM derive for the web UI. Reuses the validated
spike pipeline and hdr.py, with the two learnings baked in:
  1) detect on a Reinhard-tonemapped image (robust to bright in-scene sources), and
  2) sample BLACK-LEVEL-SUBTRACTED linear data (pedestal desaturates otherwise).

analyze(frame, maxv) -> dict with dE (vendor vs derived), CCM, illuminant, clip
fraction, and base64 PNGs (detection overlay + labeled measured/reference swatches).
"""
import base64
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # metro_server/
import hdr  # noqa: E402

# X-Rite ColorChecker Classic patch names, row-major (matches hdr.COLORCHECKER_SRGB)
PATCH_NAMES = [
    "dark skin", "light skin", "blue sky", "foliage", "blue flower", "bluish green",
    "orange", "purplish blue", "moderate red", "purple", "yellow green", "orange yellow",
    "blue", "green", "red", "yellow", "magenta", "cyan",
    "white", "neutral 8", "neutral 6.5", "neutral 5", "neutral 3.5", "black",
]

# nearest-standard-illuminant table for a rough label from CCT
STD_ILLUM = [("A", 2856), ("D50", 5003), ("D55", 5503), ("D65", 6504), ("D75", 7504)]

_G = np.arange(24).reshape(4, 6)
_PERMS = {"id": _G.flatten(), "lr": np.fliplr(_G).flatten(),
          "ud": np.flipud(_G).flatten(), "rot180": np.rot90(_G, 2).flatten()}


def _default_black_level(maxv):
    return 200.0 if maxv > 2000 else 50.0        # ~200 DN @12-bit, ~50 @10-bit


def _bin_rggb(frame, maxv, black_level):
    H, W = frame.shape
    He, We = (H // 2) * 2, (W // 2) * 2
    f = np.clip(frame[:He, :We].astype(np.float32) - black_level, 0, None)
    R = f[0::2, 0::2]; Gr = f[0::2, 1::2]; Gb = f[1::2, 0::2]; B = f[1::2, 1::2]
    return np.dstack([R, 0.5 * (Gr + Gb), B]) / (maxv - black_level)


def _detect_img(rgb_lin):
    wb = rgb_lin.copy()
    mR, mG, mB = (wb[..., 0].mean(), wb[..., 1].mean(), wb[..., 2].mean())
    wb[..., 0] *= mG / max(mR, 1e-6)
    wb[..., 2] *= mG / max(mB, 1e-6)
    return cv2.cvtColor(hdr.tonemap_rgb(wb), cv2.COLOR_RGB2BGR)


def _sort_corners(box):
    p = np.asarray(box, np.float64); s = p[:, 0] + p[:, 1]; d = p[:, 1] - p[:, 0]
    TL = p[np.argmin(s)]; BR = p[np.argmax(s)]; TR = p[np.argmin(d)]; BL = p[np.argmax(d)]
    return np.array([[TL[1], TL[0]], [TR[1], TR[0]], [BR[1], BR[0]], [BL[1], BL[0]]])


def _sample_linear(rgb, corners, rows=4, cols=6, frac=0.5):
    TL, TR, BR, BL = [np.asarray(c, np.float64) for c in corners]
    cell_r = (np.linalg.norm(BL - TL) + np.linalg.norm(BR - TR)) / 2 / rows
    cell_c = (np.linalg.norm(TR - TL) + np.linalg.norm(BR - BL)) / 2 / cols
    hh = max(2, int(frac * cell_r / 2)); hw = max(2, int(frac * cell_c / 2))
    out = np.zeros((24, 3)); ctrs = []
    for i in range(rows):
        for j in range(cols):
            u = (j + 0.5) / cols; v = (i + 0.5) / rows
            top = TL * (1 - u) + TR * u; bot = BL * (1 - u) + BR * u
            ctr = top * (1 - v) + bot * v
            r, c = int(round(ctr[0])), int(round(ctr[1])); ctrs.append((r, c))
            out[i * cols + j] = np.median(
                rgb[max(0, r - hh):r + hh, max(0, c - hw):c + hw].reshape(-1, 3), 0)
    return out, ctrs, (hh, hw)


def _lin2lab(rgb):
    rgb = np.clip(np.asarray(rgb, np.float64), 0, 1)
    srgb = np.where(rgb <= 0.0031308, 12.92 * rgb, 1.055 * np.power(rgb, 1 / 2.4) - 0.055)
    lab = cv2.cvtColor(np.clip(srgb, 0, 1).astype(np.float32).reshape(-1, 1, 3),
                       cv2.COLOR_RGB2Lab)
    return lab.reshape(-1, 3).astype(np.float64)


def _lin2srgb8(vals):
    srgb = np.where(vals <= 0.0031308, 12.92 * vals,
                    1.055 * np.power(np.clip(vals, 0, 1), 1 / 2.4) - 0.055)
    return (np.clip(srgb, 0, 1) * 255 + 0.5).astype(np.uint8)


def _mccamy_cct(x, y):
    n = (x - 0.3320) / (0.1858 - y)
    return 449 * n ** 3 + 3525 * n ** 2 + 6823.3 * n + 5520.33


def _illuminant(neutral_lin):
    srgb_lin = np.asarray(neutral_lin) @ np.asarray(hdr.VENDOR_CCM).T
    M = np.array([[0.4124, 0.3576, 0.1805],
                  [0.2126, 0.7152, 0.0722],
                  [0.0193, 0.1192, 0.9505]])
    XYZ = np.clip(srgb_lin, 0, None) @ M.T
    if XYZ.sum() <= 0:
        return float("nan"), "unknown"
    x, y = XYZ[0] / XYZ.sum(), XYZ[1] / XYZ.sum()
    cct = _mccamy_cct(x, y)
    name = min(STD_ILLUM, key=lambda t: abs(t[1] - cct))[0]
    return float(cct), name


def _b64png(bgr):
    ok, buf = cv2.imencode(".png", bgr)
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()


def _swatch_image(after_lin, ref_lin, cell=54, gap=3):
    """4x6 chart layout; each cell split: LEFT = measured+CCM, RIGHT = reference."""
    after8 = _lin2srgb8(after_lin); ref8 = _lin2srgb8(ref_lin)
    H = 4 * cell + 5 * gap; W = 6 * cell + 5 * gap
    img = np.full((H, W, 3), 30, np.uint8)
    for p in range(24):
        i, j = p // 6, p % 6
        y0 = gap + i * (cell + gap); x0 = gap + j * (cell + gap)
        half = cell // 2
        img[y0:y0 + cell, x0:x0 + half] = after8[p][::-1]        # left = measured (BGR)
        img[y0:y0 + cell, x0 + half:x0 + cell] = ref8[p][::-1]   # right = reference
    return _b64png(img)


def analyze(frame, maxv, black_level=None):
    if black_level is None:
        black_level = _default_black_level(maxv)
    rgb_lin = _bin_rggb(frame, maxv, black_level)
    bgr = _detect_img(rgb_lin)
    det = cv2.mcc.CCheckerDetector_create()
    if not det.process(bgr, cv2.mcc.MCC24):
        return {"detected": False, "black_level": black_level,
                "clip_frac": float((frame >= maxv).mean())}
    cc = det.getListColorChecker()[0]
    corners = _sort_corners(cc.getBox())
    chart, ctrs, (hh, hw) = _sample_linear(rgb_lin, corners)
    ref = hdr.colorchecker_linear(); ref_lab = _lin2lab(ref)

    best = None
    for name, perm in _PERMS.items():
        X = chart[perm]; sc = ref.mean() / max(X.mean(), 1e-9)
        A, *_ = np.linalg.lstsq(X * sc, ref, rcond=None)
        r = float(np.sqrt(np.mean((X * sc @ A - ref) ** 2)))
        if best is None or r < best[1]:
            best = (name, r, perm)
    orient, _, perm = best
    order = perm.tolist()
    chart = chart[perm]

    sc = ref.mean() / max(chart.mean(), 1e-9)
    A, *_ = np.linalg.lstsq(chart * sc, ref, rcond=None)
    M = sc * A.T
    resid = float(np.sqrt(np.mean((chart * sc @ A - ref) ** 2)))
    after = chart @ np.asarray(M).T
    dE_derived = np.linalg.norm(_lin2lab(after) - ref_lab, axis=1)

    s3 = ref.mean(0) / np.maximum(chart.mean(0), 1e-9)
    vend = np.asarray(chart * s3) @ np.asarray(hdr.VENDOR_CCM).T
    dE_vendor = np.linalg.norm(_lin2lab(vend) - ref_lab, axis=1)

    cct, illum = _illuminant(np.mean(chart[18:21], axis=0))

    # detection overlay (green sample boxes) -- ctrs are in ORIGINAL (unpermuted) order
    ov = bgr.copy()
    for (r, c) in ctrs:
        cv2.rectangle(ov, (c - hw, r - hh), (c + hw, r + hh), (0, 255, 0), 2)

    return {
        "detected": True,
        "black_level": float(black_level),
        "cost": float(cc.getCost()),
        "orientation": orient,
        "resid": resid,
        "clip_frac": float((frame >= maxv).mean()),
        "illum_cct": cct,
        "illum_name": illum,
        "dE_vendor_mean": float(dE_vendor.mean()),
        "dE_vendor_max": float(dE_vendor.max()),
        "dE_derived_mean": float(dE_derived.mean()),
        "dE_derived_max": float(dE_derived.max()),
        "dE_derived": dE_derived.round(3).tolist(),
        "dE_vendor": dE_vendor.round(3).tolist(),
        "patch_names": PATCH_NAMES,
        "ccm": np.asarray(M).round(5).tolist(),
        "white_black_ratio": float(chart[18].max() / max(chart[23].max(), 1e-6)),
        "overlay_png": _b64png(ov),
        "swatch_png": _swatch_image(after, ref),
    }
