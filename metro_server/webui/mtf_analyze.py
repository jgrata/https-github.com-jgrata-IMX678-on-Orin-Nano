"""Slanted-edge MTF (focus assist) for the web UI. Ports the MATLAB GUI flow:
find squares (cv2.MSER -> convex hull -> cv2.minAreaRect Feret box), extract an
ROI straddling each edge, run the validated mtf.jslantedge port.

Black level is subtracted for a clean ESF, though MTF itself is DC-invariant
(the LSF is gradient(esf), so a constant pedestal cancels).

analyze(frame, maxv, params) -> squares/edges with MTF curves, MTF50, MTF@Nyquist,
units (cyc/mm or cyc/deg), and an annotated overlay PNG.
"""
import base64
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))               # webui/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # metro_server/
import mtf  # noqa: E402  (validated jslantedge port)
from imaging import default_black_level  # noqa: E402


def _bin_luma(frame, maxv, black_level):
    """2x2 RGGB bin -> luma [0..1], half-res (matches the GUI's LastLuma)."""
    H, W = frame.shape
    He, We = (H // 2) * 2, (W // 2) * 2
    f = np.clip(frame[:He, :We].astype(np.float32) - black_level, 0, None)
    R = f[0::2, 0::2]; Gr = f[0::2, 1::2]; Gb = f[1::2, 0::2]; B = f[1::2, 1::2]
    G = 0.5 * (Gr + Gb)
    luma = 0.25 * R + 0.5 * G + 0.25 * B
    return luma / (maxv - black_level)


def _find_squares(luma, p):
    npx = luma.size
    minA = max(int(p["min_pct"] / 100 * npx), 30)
    maxA = max(int(p["max_pct"] / 100 * npx), minA + 1)
    I8 = cv2.normalize(luma, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    try:
        I8 = cv2.createCLAHE(2.0, (8, 8)).apply(I8)          # aids MSER (per mtfgui)
    except Exception:
        pass
    mser = cv2.MSER_create()
    mser.setDelta(int(max(p["delta"], 1)))
    mser.setMinArea(minA); mser.setMaxArea(maxA)
    for setter, val in (("setMaxVariation", p["max_var"]),
                        ("setMinDiversity", p["min_div"])):
        try:
            getattr(mser, setter)(float(val))
        except Exception:
            pass
    regions, _ = mser.detectRegions(I8)
    boxes = []
    for pts in regions:
        if len(pts) < 3:
            continue
        hull = cv2.convexHull(pts.reshape(-1, 1, 2).astype(np.int32))
        (cx, cy), (w, h), ang = cv2.minAreaRect(hull)        # Feret box
        if max(w, h) <= 0 or min(w, h) / max(w, h) < p["min_ar"]:
            continue
        boxes.append({"verts": cv2.boxPoints(((cx, cy), (w, h), ang)),
                      "center": (cx, cy), "wh": (w, h), "area": w * h})
    boxes.sort(key=lambda b: -b["area"])                     # merge near-duplicates
    kept = []
    for b in boxes:
        if any(np.hypot(b["center"][0] - k["center"][0], b["center"][1] - k["center"][1])
               < p["merge_frac"] * max(k["wh"]) for k in kept):
            continue
        kept.append(b)
    return kept


def _edge_roi_corners(V, e, along, across):
    A = V[e]; B = V[(e + 1) % 4]; d = B - A; L = float(np.hypot(d[0], d[1]))
    if L < 1e-9:
        return np.repeat(A[None], 4, 0)
    u = d / L; nrm = np.array([-u[1], u[0]]); M = (A + B) / 2
    ha = along * L / 2; hc = across * L / 2
    return np.array([M - ha * u - hc * nrm, M + ha * u - hc * nrm,
                     M + ha * u + hc * nrm, M - ha * u + hc * nrm])


def _extract_roi(src, R):
    x0 = max(int(np.floor(R[:, 0].min())), 0); x1 = min(int(np.ceil(R[:, 0].max())), src.shape[1])
    y0 = max(int(np.floor(R[:, 1].min())), 0); y1 = min(int(np.ceil(R[:, 1].max())), src.shape[0])
    roi = src[y0:y1, x0:x1]
    adir = R[1] - R[0]
    if abs(adir[0]) > abs(adir[1]):        # along-edge ~horizontal -> transpose to vertical
        roi = roi.T
    return roi


def _mtf50(freq, m):
    for i in range(1, len(m)):
        if m[i - 1] >= 0.5 > m[i]:
            t = (0.5 - m[i - 1]) / (m[i] - m[i - 1])
            return float(freq[i - 1] + t * (freq[i] - freq[i - 1]))
    return float("nan")


def _b64png(bgr):
    ok, buf = cv2.imencode(".png", bgr)
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()


_DEFAULTS = dict(min_pct=0.3, max_pct=8.0, delta=5, max_var=0.25, min_div=0.2,
                 min_ar=0.6, merge_frac=0.5, along=0.7, across=0.3, osf=4,
                 pitch_um=2.0, efl_mm=8.0, units="mm")


def analyze(frame, maxv, params=None):
    p = dict(_DEFAULTS)
    if params:
        p.update({k: params[k] for k in params if k in _DEFAULTS})
    bl = params.get("black_level") if params else None
    if bl is None:
        bl = default_black_level(maxv)
    luma = _bin_luma(frame, maxv, bl)

    osf = int(p["osf"])
    pitch_um = float(p["pitch_um"]) * 2.0        # half-res preview -> pitch doubles
    if p["units"] == "deg":
        pixel = float(np.degrees(np.arctan((pitch_um / 1000.0) / max(float(p["efl_mm"]), 1e-9))))
        ustr = "cyc/deg"
    else:
        pixel = pitch_um / 1000.0                 # mm/pixel -> cyc/mm
        ustr = "cyc/mm"
    nyq = 1.0 / (2.0 * pixel)
    fq = np.linspace(0, nyq, 80)

    sq = _find_squares(luma, p)
    disp = cv2.cvtColor(
        (np.clip(luma / max(np.percentile(luma, 99), 1e-6), 0, 1) ** (1 / 2.2) * 255).astype(np.uint8),
        cv2.COLOR_GRAY2BGR)
    edges = []
    for si, b in enumerate(sq):
        V = b["verts"]
        cv2.polylines(disp, [V.astype(np.int32)], True, (255, 180, 80), 1)
        for e in range(4):
            R = _edge_roi_corners(V, e, p["along"], p["across"])
            roi = _extract_roi(luma, R)
            if roi.size < 64:
                continue
            try:
                ff, mm, *_ = mtf.jslantedge(roi, osf, pixel)
            except Exception:
                continue
            m50 = _mtf50(ff, mm)
            mq = np.interp(fq, ff, mm, left=float(mm[0]), right=0.0)
            m_nyq = float(np.interp(nyq, ff, mm, left=float(mm[0]), right=0.0))
            edges.append({
                "square": si, "edge": e,
                "mtf50": None if np.isnan(m50) else round(m50, 4),
                "mtf_nyq": round(m_nyq, 4),
                "freq": fq.round(4).tolist(), "mtf": mq.round(4).tolist(),
            })
            x0, y0 = int(R[:, 0].min()), int(R[:, 1].min())
            x1, y1 = int(R[:, 0].max()), int(R[:, 1].max())
            cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 1)
            Mc = R.mean(0)
            lbl = "--" if np.isnan(m50) else "%.2f" % m50
            cv2.putText(disp, lbl, (int(Mc[0]) - 12, int(Mc[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

    return {
        "n_squares": len(sq), "n_edges": len(edges),
        "units": ustr, "nyquist": round(nyq, 4), "pixel": pixel, "osf": osf,
        "black_level": float(bl), "clip_frac": float((frame >= maxv).mean()),
        "edges": edges, "overlay_png": _b64png(disp),
    }
