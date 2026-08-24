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


def _illuminant(neutral_lin, ccm=None):
    """Rough CCT of the scene illuminant from the neutral patches: map neutral raw through
    a FIXED raw->sRGB matrix (illuminant-INDEPENDENT) -> XYZ -> xy -> McCamy CCT.
    NOTE: this MUST use a fixed sensor matrix, never the shot's own derived CCM -- the derived
    CCM is fit to THIS illuminant and white-balances the neutrals to ~D65, so feeding it back
    would always report ~6500K. The default (hdr.VENDOR_CCM, the eCAM/Jetson tuning) is only
    approximate on the IQ9, so treat the CCT as a rough label; wb_gains is the calibration-free
    cast indicator to trust instead."""
    ccm = hdr.VENDOR_CCM if ccm is None else ccm
    srgb_lin = np.asarray(neutral_lin) @ np.asarray(ccm).T
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


def _wb_gains(neutral_rgb):
    """Calibration-free illuminant cast fingerprint: the per-channel gains that neutralize
    the measured (black-level-subtracted) neutral patches, normalized to green (G=1).
    g_R>1 & g_B<1 => warm source (tungsten); g_R<1 & g_B>1 => cool (daylight). Independent
    of any CCM, so unlike the CCT label it stays meaningful even without a calibrated matrix."""
    m = np.clip(np.asarray(neutral_rgb, float).mean(axis=0), 1e-6, None)
    g = m[1] / m                                 # gains that pull each channel up to green
    return [float(g[0]), 1.0, float(g[2])]


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


def _de2000(lab1, lab2):
    """CIEDE2000 colour difference (kL=kC=kH=1), vectorized over N patches.
    Perceptually uniform -- the metric to trust over plain ΔE76, which over-weights
    chroma/blue error and misrepresents perceived accuracy."""
    lab1 = np.asarray(lab1, float); lab2 = np.asarray(lab2, float)
    L1, a1, b1 = lab1[:, 0], lab1[:, 1], lab1[:, 2]
    L2, a2, b2 = lab2[:, 0], lab2[:, 1], lab2[:, 2]
    C1 = np.hypot(a1, b1); C2 = np.hypot(a2, b2)
    Cbar = (C1 + C2) / 2.0
    G = 0.5 * (1 - np.sqrt(Cbar ** 7 / (Cbar ** 7 + 25.0 ** 7)))
    a1p = (1 + G) * a1; a2p = (1 + G) * a2
    C1p = np.hypot(a1p, b1); C2p = np.hypot(a2p, b2)
    h1p = np.degrees(np.arctan2(b1, a1p)) % 360.0
    h2p = np.degrees(np.arctan2(b2, a2p)) % 360.0
    dLp = L2 - L1
    dCp = C2p - C1p
    dhp = h2p - h1p
    dhp = np.where(dhp > 180, dhp - 360, dhp)
    dhp = np.where(dhp < -180, dhp + 360, dhp)
    dhp = np.where(C1p * C2p == 0, 0.0, dhp)
    dHp = 2 * np.sqrt(C1p * C2p) * np.sin(np.radians(dhp) / 2.0)
    Lbarp = (L1 + L2) / 2.0
    Cbarp = (C1p + C2p) / 2.0
    hsum = h1p + h2p; hdiff = np.abs(h1p - h2p)
    hbarp = np.where(C1p * C2p == 0, hsum,
             np.where(hdiff <= 180, hsum / 2.0,
              np.where(hsum < 360, (hsum + 360) / 2.0, (hsum - 360) / 2.0)))
    T = (1 - 0.17 * np.cos(np.radians(hbarp - 30))
         + 0.24 * np.cos(np.radians(2 * hbarp))
         + 0.32 * np.cos(np.radians(3 * hbarp + 6))
         - 0.20 * np.cos(np.radians(4 * hbarp - 63)))
    dTheta = 30 * np.exp(-(((hbarp - 275) / 25.0) ** 2))
    RC = 2 * np.sqrt(Cbarp ** 7 / (Cbarp ** 7 + 25.0 ** 7))
    SL = 1 + (0.015 * (Lbarp - 50) ** 2) / np.sqrt(20 + (Lbarp - 50) ** 2)
    SC = 1 + 0.045 * Cbarp
    SH = 1 + 0.015 * Cbarp * T
    RT = -np.sin(np.radians(2 * dTheta)) * RC
    return np.sqrt((dLp / SL) ** 2 + (dCp / SC) ** 2 + (dHp / SH) ** 2
                   + RT * (dCp / SC) * (dHp / SH))


def _fit_ccm(chart, ref):
    """Scale-conditioned least-squares raw-linear -> linear-sRGB CCM. Returns M
    with apply = rgb @ M.T."""
    sc = ref.mean() / max(chart.mean(), 1e-9)
    A, *_ = np.linalg.lstsq(chart * sc, ref, rcond=None)
    return sc * A.T


def _loo_de2000(chart, ref, ref_lab):
    """Leave-one-out cross-validated ΔE00 for the derived CCM: each patch is
    predicted by a CCM fit on the OTHER 23, so it measures generalization, not
    self-fit. This is the honest number to compare against vendor (which never
    saw the data). Resubstitution ΔE (fit==test) is always optimistic."""
    n = len(chart)
    de = np.zeros(n)
    keep = np.arange(n)
    for i in range(n):
        m = keep != i
        M = _fit_ccm(chart[m], ref[m])
        pred = chart[i] @ np.asarray(M).T
        de[i] = _de2000(_lin2lab(pred[None, :]), ref_lab[i:i + 1])[0]
    return de


def _rootpoly_features(rgb, degree):
    """Finlayson root-polynomial features (2015). Each term is homogeneous of
    degree 1 in RGB, so the mapping stays EXPOSURE-INVARIANT (unlike ordinary
    polynomials) -- the property a CCM must keep. deg1=3 terms (=linear),
    deg2=6, deg3=13. Inputs are non-negative linear signals."""
    R = np.clip(rgb[:, 0], 0, None); G = np.clip(rgb[:, 1], 0, None); B = np.clip(rgb[:, 2], 0, None)
    feats = [R, G, B]
    if degree >= 2:
        feats += [np.sqrt(R * G), np.sqrt(G * B), np.sqrt(R * B)]
    if degree >= 3:
        feats += [np.cbrt(R * G * G), np.cbrt(R * R * G), np.cbrt(G * B * B),
                  np.cbrt(G * G * B), np.cbrt(R * B * B), np.cbrt(R * R * B), np.cbrt(R * G * B)]
    return np.column_stack(feats)


def _fit_rootpoly(chart, ref, degree):
    sc = ref.mean() / max(chart.mean(), 1e-9)
    Phi = _rootpoly_features(chart * sc, degree)
    A, *_ = np.linalg.lstsq(Phi, ref, rcond=None)      # [Nterms x 3]
    return (sc, A, degree)


def _apply_rootpoly(rgb, model):
    sc, A, degree = model
    return _rootpoly_features(np.atleast_2d(rgb) * sc, degree) @ A


def _loo_rootpoly_de2000(chart, ref, ref_lab, degree):
    n = len(chart); de = np.zeros(n); keep = np.arange(n)
    for i in range(n):
        m = keep != i
        model = _fit_rootpoly(chart[m], ref[m], degree)
        pred = _apply_rootpoly(chart[i], model)
        de[i] = _de2000(_lin2lab(pred), ref_lab[i:i + 1])[0]
    return de


def meter_levels(frame, maxv, black_level=None):
    """Detect the chart and return the brightest patch-channel and darkest patch,
    as fractions of full signal (black-level subtracted). Orientation-independent:
    'hi' = the channel most at risk of clipping (max over all patches/channels),
    'lo' = the darkest patch (min patch-mean) — the SNR-limited end. Used to meter
    an HDR bracket that keeps the whole chart unclipped with good shadow SNR."""
    if black_level is None:
        black_level = _default_black_level(maxv)
    rgb_lin = _bin_rggb(frame, maxv, black_level)          # signal / (maxv - bl), in [0,1]
    bgr = _detect_img(rgb_lin)
    det = cv2.mcc.CCheckerDetector_create()
    if not det.process(bgr, cv2.mcc.MCC24):
        return {"detected": False, "clip_frac": float((frame >= maxv).mean())}
    cc = det.getListColorChecker()[0]
    chart, _, _ = _sample_linear(rgb_lin, _sort_corners(cc.getBox()))
    hi = float(chart.max())                                # brightest single channel (clip risk)
    lo = float(chart.mean(axis=1).min())                   # darkest patch mean (SNR limit)
    return {
        "detected": True,
        "hi": hi, "lo": lo,
        "hi_clip": hi >= 0.97,                              # brightest patch channel at/above saturation
        "chart_dr": (hi / lo) if lo > 1e-6 else float("inf"),
        "black_level": float(black_level),
        "clip_frac": float((frame >= maxv).mean()),
    }


def analyze(frame, maxv, black_level=None, rootpoly_degree=2):
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

    # derived CCM fit on all 24 patches (resubstitution) -> optimistic ΔE00
    M = _fit_ccm(chart, ref)
    after = chart @ np.asarray(M).T
    dE_derived = _de2000(_lin2lab(after), ref_lab)
    resid = float(np.sqrt(np.mean((np.clip(after, 0, 1) - ref) ** 2)))   # clip-consistent
    # leave-one-out cross-validated ΔE00 -> the HONEST generalization number
    dE_xval = _loo_de2000(chart, ref, ref_lab)
    # vendor CCM (no fit -> unbiased); gray-world scale to compare on equal footing
    s3 = ref.mean(0) / np.maximum(chart.mean(0), 1e-9)
    vend = np.asarray(chart * s3) @ np.asarray(hdr.VENDOR_CCM).T
    dE_vendor = _de2000(_lin2lab(vend), ref_lab)

    # root-polynomial CCM (ANALYSIS ONLY -- the exported/applied CCM stays the 3x3;
    # root-poly isn't a 3x3 the ISP can consume). Shows the headroom beyond a 3x3;
    # the fit-vs-xval gap is the overfit guardrail (more DOF -> watch the gap).
    rp_deg = int(rootpoly_degree)
    rp_model = _fit_rootpoly(chart, ref, rp_deg)
    dE_rp_fit = _de2000(_lin2lab(np.clip(_apply_rootpoly(chart, rp_model), 0, 1)), ref_lab)
    dE_rp_xval = _loo_rootpoly_de2000(chart, ref, ref_lab, rp_deg)
    rp_terms = int(_rootpoly_features(chart[:1], rp_deg).shape[1])

    neutral = chart[18:21]                              # white, neutral8, neutral6.5 (bright greys)
    cct, illum = _illuminant(np.mean(neutral, axis=0))  # via a FIXED matrix (approx) -- see _illuminant
    wb_gains = _wb_gains(neutral)                        # calibration-free cast indicator
    white_level = float(chart[18].max())                # brightest neutral channel, frac of full-scale

    # detection overlay (green sample boxes) -- ctrs are in ORIGINAL (unpermuted) order
    ov = bgr.copy()
    for (r, c) in ctrs:
        cv2.rectangle(ov, (c - hw, r - hh), (c + hw, r + hh), (0, 255, 0), 2)

    return {
        "detected": True,
        "metric": "CIEDE2000",
        "black_level": float(black_level),
        "cost": float(cc.getCost()),
        "orientation": orient,
        "resid": resid,
        "clip_frac": float((frame >= maxv).mean()),
        "illum_cct": cct,
        "illum_name": illum,
        "illum_approx": True,                             # CCT via a fixed (uncalibrated) matrix
        "wb_gains": [round(g, 4) for g in wb_gains],       # [R,G,B] gains to neutral (G=1); cast fingerprint
        "white_level": round(white_level, 4),             # white-patch brightest channel, frac of full-scale
        "white_clip": bool(white_level >= 0.97),          # white patch at/near saturation -> corrupts CCM
        # vendor CCM (unbiased) vs derived: report BOTH the optimistic self-fit and
        # the honest cross-validated ΔE00. Compare vendor vs xval to judge "does ours win".
        "dE_vendor_mean": float(dE_vendor.mean()),
        "dE_vendor_max": float(dE_vendor.max()),
        "dE_derived_mean": float(dE_derived.mean()),      # resubstitution (fit==test, optimistic)
        "dE_derived_max": float(dE_derived.max()),
        "dE_xval_mean": float(dE_xval.mean()),            # leave-one-out (honest)
        "dE_xval_max": float(dE_xval.max()),
        "derived_beats_vendor": bool(dE_xval.mean() < dE_vendor.mean()),
        # root-polynomial (analysis) -- can a higher-DOF model beat the 3x3?
        "rootpoly_degree": rp_deg,
        "rootpoly_terms": rp_terms,
        "dE_rootpoly_fit_mean": float(dE_rp_fit.mean()),
        "dE_rootpoly_xval_mean": float(dE_rp_xval.mean()),
        "dE_rootpoly_xval_max": float(dE_rp_xval.max()),
        "rootpoly_beats_linear": bool(dE_rp_xval.mean() < dE_xval.mean()),
        "dE_derived": dE_xval.round(3).tolist(),          # per-patch = cross-validated
        "dE_vendor": dE_vendor.round(3).tolist(),
        "patch_names": PATCH_NAMES,
        "ccm": np.asarray(M).round(5).tolist(),
        "white_black_ratio": float(chart[18].max() / max(chart[23].max(), 1e-6)),
        "overlay_png": _b64png(ov),
        "swatch_png": _swatch_image(after, ref),
    }


# ── Vendor-ISP colour eval (processed frames: Jetson ISP / IQ9 NV12) ──────────
# No CCM fit -- the ISP already colour-corrects (its tuning carries the CCMs), so we
# measure ΔE00 of ITS displayed output vs the ColorChecker reference. The input is a
# display-ready sRGB BGR frame, so we detect on it directly (no tonemap) and sample
# in sRGB (no black-level / linearisation of raw).
def _srgb2lab(srgb01):
    """sRGB (gamma-encoded RGB in [0,1]) -> Lab (D65). OpenCV RGB2Lab assumes sRGB."""
    lab = cv2.cvtColor(np.clip(srgb01, 0, 1).astype(np.float32).reshape(-1, 1, 3),
                       cv2.COLOR_RGB2Lab)
    return lab.reshape(-1, 3).astype(np.float64)


def _swatch_image8(meas8, ref8, cell=54, gap=3):
    """4x6 swatch; each cell LEFT = measured (ISP), RIGHT = reference. 8-bit RGB in."""
    H = 4 * cell + 5 * gap; W = 6 * cell + 5 * gap
    img = np.full((H, W, 3), 30, np.uint8)
    for p in range(24):
        i, j = p // 6, p % 6
        y0 = gap + i * (cell + gap); x0 = gap + j * (cell + gap); half = cell // 2
        img[y0:y0 + cell, x0:x0 + half] = meas8[p][::-1]         # left = measured (to BGR)
        img[y0:y0 + cell, x0 + half:x0 + cell] = ref8[p][::-1]   # right = reference
    return _b64png(img)


def analyze_processed(bgr):
    """Vendor-ISP colour eval on an ISP-processed BGR frame (e.g. IQ9 NV12).
    Returns per-patch + summary ΔE00 of the ISP output vs the ColorChecker reference,
    the neutral-patch colour cast, a detection overlay, and a measured-vs-ref swatch."""
    det = cv2.mcc.CCheckerDetector_create()
    if not det.process(bgr, cv2.mcc.MCC24):
        return {"detected": False, "clip_frac": float((bgr >= 254).mean())}
    cc = det.getListColorChecker()[0]
    corners = _sort_corners(cc.getBox())
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float64)
    meas, ctrs, (hh, hw) = _sample_linear(rgb, corners)          # 24x3 in 0..255
    meas01 = np.clip(meas / 255.0, 0.0, 1.0)

    ref = np.asarray(hdr.colorchecker_linear()); ref_lab = _lin2lab(ref)
    best = None                                                  # orient by min mean ΔE00
    for name, perm in _PERMS.items():
        de = _de2000(_srgb2lab(meas01[perm]), ref_lab)
        if best is None or de.mean() < best[1]:
            best = (name, float(de.mean()), perm)
    orient, _, perm = best
    meas01 = meas01[perm]; ctrs = [ctrs[k] for k in perm.tolist()]
    meas_lab = _srgb2lab(meas01)
    de = _de2000(meas_lab, ref_lab)

    neutral = meas_lab[18:24]                                    # gray ramp: residual cast
    cast_a = float(np.mean(np.abs(neutral[:, 1]))); cast_b = float(np.mean(np.abs(neutral[:, 2])))
    worst = np.argsort(-de)[:3]

    ov = bgr.copy()
    for (r, c) in ctrs:
        cv2.rectangle(ov, (c - hw, r - hh), (c + hw, r + hh), (0, 255, 0), 2)
    meas8 = (meas01 * 255 + 0.5).astype(np.uint8)

    return {
        "detected": True, "mode": "vendor-isp-eval", "metric": "CIEDE2000",
        "orientation": orient, "cost": float(cc.getCost()),
        "clip_frac": float((bgr >= 254).mean()),
        "dE_mean": float(de.mean()), "dE_max": float(de.max()), "dE_median": float(np.median(de)),
        "dE": de.round(3).tolist(), "patch_names": PATCH_NAMES,
        "worst": [{"name": PATCH_NAMES[i], "de": round(float(de[i]), 2)} for i in worst],
        "neutral_cast_a": round(cast_a, 2), "neutral_cast_b": round(cast_b, 2),
        "overlay_png": _b64png(ov), "swatch_png": _swatch_image8(meas8, _lin2srgb8(ref)),
    }
