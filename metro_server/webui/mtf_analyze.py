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


def _bin_channel(frame, maxv, black_level, chan="R"):
    """4:1 Bayer bin to one half-res plane, black-subtracted, [0..1].
    chan 'R'/'B' = a single strided plane (cheap; a high-contrast B&W target has
    signal in every channel, so R or B is ideal and skips the luma weighting).
    'Y' = 0.25R+0.5G+0.25B (needed for ABSOLUTE MTF; relative-focus MTF on a single
    channel is fine). Only 'Y' pays the full-frame cost."""
    H, W = frame.shape
    He, We = (H // 2) * 2, (W // 2) * 2
    fr = frame[:He, :We]
    if chan == "R":
        plane = fr[0::2, 0::2].astype(np.float32)
    elif chan == "B":
        plane = fr[1::2, 1::2].astype(np.float32)
    else:  # Y luma
        R = fr[0::2, 0::2].astype(np.float32); Gr = fr[0::2, 1::2].astype(np.float32)
        Gb = fr[1::2, 0::2].astype(np.float32); B = fr[1::2, 1::2].astype(np.float32)
        plane = 0.25 * R + 0.25 * (Gr + Gb) + 0.25 * B
    plane = np.maximum(plane - black_level, 0.0)
    return plane / (maxv - black_level)


def _find_squares(luma, p, det_width=480):
    # MSER + the per-region hull/minAreaRect loop are run on a DOWNSAMPLED image
    # (default ~640 px wide): MSER at 2 MP spawns a huge number of nested regions
    # so the loop dominates (~1.5 s); at 640 px it's ~10x faster and finds the same
    # squares. Box vertices are scaled back to full luma coords for the SFR (which
    # still runs on the full-res edge). area% is of the downsampled image (== of the
    # frame, so thresholds are unchanged).
    Hf, Wf = luma.shape
    s = min(1.0, det_width / float(Wf))
    small = cv2.resize(luma, (int(round(Wf * s)), int(round(Hf * s))),
                       interpolation=cv2.INTER_AREA) if s < 1.0 else luma
    npx = small.size
    minA = max(int(p["min_pct"] / 100 * npx), 12)
    maxA = max(int(p["max_pct"] / 100 * npx), minA + 1)
    I8 = cv2.normalize(small, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
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
    inv = 1.0 / s
    boxes = []
    for pts in regions:
        if len(pts) < 3:
            continue
        hull = cv2.convexHull(pts.reshape(-1, 1, 2).astype(np.int32))
        (cx, cy), (w, h), ang = cv2.minAreaRect(hull)        # Feret box (downsampled coords)
        if max(w, h) <= 0 or min(w, h) / max(w, h) < p["min_ar"]:
            continue
        boxes.append({"verts": cv2.boxPoints(((cx, cy), (w, h), ang)) * inv,   # -> full luma coords
                      "center": (cx * inv, cy * inv), "wh": (w * inv, h * inv), "area": w * h * inv * inv})
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


def _b64jpg(bgr, width=900, quality=80):
    h, w = bgr.shape[:2]
    if w > width:
        bgr = cv2.resize(bgr, (width, max(1, int(h * width / w))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()


_DEFAULTS = dict(min_pct=0.3, max_pct=8.0, delta=5, max_var=0.25, min_div=0.2,
                 min_ar=0.6, merge_frac=0.5, along=0.7, across=0.3, osf=4,
                 pitch_um=2.0, efl_mm=8.0, units="mm", chan="R")


def analyze(frame, maxv, params=None):
    p = dict(_DEFAULTS)
    if params:
        p.update({k: params[k] for k in params if k in _DEFAULTS})
    bl = params.get("black_level") if params else None
    if bl is None:
        bl = default_black_level(maxv)
    # Locked ROIs: reuse caller-supplied boxes (skip MSER) for a fast live loop.
    boxes_in = params.get("boxes") if params else None
    luma = _bin_channel(frame, maxv, bl, p["chan"])   # R/B (fast, B&W target) or Y (absolute)

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

    if boxes_in:
        sq = [{"verts": np.asarray(b, np.float64)} for b in boxes_in if len(b) == 4]
        found = False
    else:
        sq = _find_squares(luma, p)
        found = True
    disp = cv2.cvtColor(
        (np.clip(luma / max(np.percentile(luma, 99), 1e-6), 0, 1) ** (1 / 2.2) * 255).astype(np.uint8),
        cv2.COLOR_GRAY2BGR)
    # Collect every edge ROI (draw square outlines now), then run the SFR serially.
    # (Profiled: ThreadPool is SLOWER here -- jslantedge is GIL-bound on small ROIs,
    # so pool overhead dominates. 8 edges ~30 ms serial.)
    tasks = []
    for si, b in enumerate(sq):
        V = b["verts"]
        cv2.polylines(disp, [V.astype(np.int32)], True, (255, 180, 80), 1)
        for e in range(4):
            R = _edge_roi_corners(V, e, p["along"], p["across"])
            roi = _extract_roi(luma, R)
            if roi.size >= 64:
                tasks.append((si, e, R, roi))

    def _one(task):
        si, e, R, roi = task
        try:
            ff, mm, *_ = mtf.jslantedge(roi, osf, pixel)
        except Exception:
            return None
        m50 = _mtf50(ff, mm)
        mq = np.interp(fq, ff, mm, left=float(mm[0]), right=0.0)
        m_nyq = float(np.interp(nyq, ff, mm, left=float(mm[0]), right=0.0))
        return {"square": si, "edge": e, "R": R,
                "mtf50": None if np.isnan(m50) else round(m50, 4),
                "mtf_nyq": round(m_nyq, 4),
                "freq": fq.round(4).tolist(), "mtf": mq.round(4).tolist()}

    edges = []
    for t in tasks:
        r = _one(t)
        if r is None:
            continue
        R = r.pop("R")
        edges.append(r)
        x0, y0 = int(R[:, 0].min()), int(R[:, 1].min())
        x1, y1 = int(R[:, 0].max()), int(R[:, 1].max())
        cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 1)
        Mc = R.mean(0)
        lbl = "--" if r["mtf50"] is None else "%.2f" % r["mtf50"]
        cv2.putText(disp, lbl, (int(Mc[0]) - 12, int(Mc[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

    # per-square size (% of the detect image) + aspect, so the client can
    # auto-tighten the finder params (size is stable under focus drift).
    npx = float(luma.size)
    area_pct, ars = [], []
    for b in sq:
        V = b["verts"]; x = V[:, 0]; y = V[:, 1]
        A = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
        s1 = float(np.hypot(*(V[1] - V[0]))); s2 = float(np.hypot(*(V[2] - V[1])))
        area_pct.append(round(100.0 * A / npx, 3))
        ars.append(round(min(s1, s2) / max(s1, s2, 1e-9), 3))
    return {
        "n_squares": len(sq), "n_edges": len(edges), "found": found, "chan": p["chan"],
        "units": ustr, "nyquist": round(nyq, 4), "pixel": pixel, "osf": osf,
        "black_level": float(bl), "clip_frac": float((frame >= maxv).mean()),
        "edges": edges, "overlay_png": _b64jpg(disp),
        "boxes": [b["verts"].round(1).tolist() for b in sq],   # for lock/reuse
        "sq_area_pct": area_pct, "sq_ar": ars,                  # for auto-tighten
    }
