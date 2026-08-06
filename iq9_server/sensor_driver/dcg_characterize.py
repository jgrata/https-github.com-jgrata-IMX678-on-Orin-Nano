#!/usr/bin/env python3
"""Per-leg static characterization for IMX678 DCG (Clear HDR) captures.

Splits each stacked Clear HDR RAW16 frame into HG (HCG) and LG (LCG) legs (via dcg_demux) and,
per leg and per RGGB channel, derives:
  - pedestal + read noise (DN and e-) from a dark point,
  - photon-transfer curve -> conversion gain K (e-/DN) and full-well (e-),
  - SNR vs signal and the sensor-referred SNR=1 point (electrons),
  - conversion-gain ratio Rcg from BOTH the PTC slopes (K_LG/K_HG) and the net-signal ratio.

Photon transfer (two-frame difference removes fixed-pattern noise):
    var_DN = (1/K) * signal_DN + read_DN**2      -> slope = 1/K, intercept = read_DN**2
    K [e-/DN] = 1/slope ; read_e = K*read_DN ; full_well_e = K*signal_sat_DN
HCG has higher conversion gain -> smaller K, steeper slope; Rcg = K_LCG/K_HCG = slope_HG/slope_LG.

Input: sweep points (each = one light level with >=2 raw frames) + a dark point, plus the demux
layout. Board-independent; pairs with iq9_client.sweep() output. SNR=1 here is sensor-referred
(electrons) - a lux-referred SNR1s needs calibrated illumination at the sensor (see
imx678-lowlight-013lx.md retraction). Requires numpy.
"""
from __future__ import annotations
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dcg_demux  # noqa: E402

LEG_ROLE = {"HG": "HCG", "LG": "LCG"}


def load_frame(path, width=None, dtype="<u2"):
    """Load a stacked DCG frame as 2-D uint16. Supports .npy (board format) and raw .bin."""
    if path.endswith(".npy"):
        a = np.load(path)
        if a.ndim == 1:
            if not width:
                raise ValueError("1-D .npy needs --width")
            a = a.reshape(-1, width)
        return np.ascontiguousarray(a)
    a = np.fromfile(path, dtype=np.dtype(dtype))
    if not width:
        raise ValueError("raw frame needs --width")
    return a.reshape(-1, width)


def _channels(leg):
    """RGGB channel planes of a leg as float64."""
    return {n: leg[pr::2, pc::2].astype(np.float64) for n, (pr, pc) in dcg_demux._PHASE.items()}


def measure(frames, layout):
    """Per-leg, per-channel mean (DN) and temporal variance (DN**2) for a set of frames.

    >=2 frames -> variance from the frame difference (fixed-pattern-noise free);
     1 frame    -> spatial variance fallback (includes FPN; flagged via nframes).
    """
    legs = {"HG": [], "LG": []}
    for f in frames:
        hg, lg = dcg_demux.demux(f, **layout)
        legs["HG"].append(_channels(hg))
        legs["LG"].append(_channels(lg))
    out = {}
    for leg, planes_list in legs.items():
        out[leg] = {}
        for name in dcg_demux.BAYER:
            planes = [c[name] for c in planes_list]
            mean = float(np.mean([p.mean() for p in planes]))
            if len(planes) >= 2:
                var = float(np.var(planes[0] - planes[1]) / 2.0)
            else:
                var = float(planes[0].var())
            out[leg][name] = {"mean": mean, "var": var, "nframes": len(planes)}
    return out


def ptc_fit(signal_dn, var_dn):
    """Linear photon-transfer fit over the rising (unsaturated) region.

    Returns K (e-/DN), slope, read_DN (from intercept), read_e, and the fit width.
    """
    s = np.asarray(signal_dn, float)
    v = np.asarray(var_dn, float)
    order = np.argsort(s)
    s, v = s[order], v[order]
    peak = int(np.argmax(v))              # variance turns over at saturation
    lin = slice(0, max(peak + 1, 2))
    sl, vl = s[lin], v[lin]
    A = np.vstack([sl, np.ones_like(sl)]).T
    slope, intercept = np.linalg.lstsq(A, vl, rcond=None)[0]
    K = 1.0 / slope if slope > 0 else float("nan")
    read_dn = math.sqrt(intercept) if intercept > 0 else 0.0
    return {
        "K_e_per_DN": float(K),
        "slope": float(slope),
        "read_DN_ptc": float(read_dn),
        "read_e_ptc": float(read_dn * K) if slope > 0 else float("nan"),
        "n_ptc": int(sl.size),
        "sat_signal_DN": float(s[peak]),
    }


def _snr1_electrons(read_e):
    """Signal (e-) at SNR=1 for shot+read noise: S = (1 + sqrt(1 + 4*read_e**2)) / 2."""
    return float((1.0 + math.sqrt(1.0 + 4.0 * read_e * read_e)) / 2.0)


def characterize(points, dark, ref_level=None):
    """Full per-leg report. points: [{'level': L, 'meas': measure(...)}]; dark: measure(...)."""
    legs_out = {}
    for leg in ("HG", "LG"):
        chans = {}
        for ch in dcg_demux.BAYER:
            ped = dark[leg][ch]["mean"]
            read_dn = math.sqrt(max(dark[leg][ch]["var"], 0.0))
            sig = [p["meas"][leg][ch]["mean"] - ped for p in points]
            var = [p["meas"][leg][ch]["var"] for p in points]
            fit = ptc_fit(sig, var)
            K = fit["K_e_per_DN"]
            read_e = read_dn * K if math.isfinite(K) else float("nan")
            snr = [s / math.sqrt(v) if v > 0 else float("nan") for s, v in zip(sig, var)]
            chans[ch] = {
                "pedestal_DN": float(ped),
                "read_DN": float(read_dn),
                "read_e": float(read_e),
                "K_e_per_DN": K,
                "slope": fit["slope"],
                "full_well_e": float(K * fit["sat_signal_DN"]) if math.isfinite(K) else float("nan"),
                "max_signal_DN": float(max(sig)) if sig else 0.0,
                "max_SNR": float(np.nanmax(snr)) if snr else float("nan"),
                "signal_at_SNR1_e": _snr1_electrons(read_e) if math.isfinite(read_e) else float("nan"),
                "read_DN_ptc": fit["read_DN_ptc"],
                "n_ptc": fit["n_ptc"],
            }
        legs_out[leg] = {"role": LEG_ROLE[leg], "channels": chans}

    # conversion-gain ratio, two independent ways
    if ref_level is None and points:
        ref_level = points[len(points) // 2]["level"]
    ref = next((p for p in points if p["level"] == ref_level), points[-1] if points else None)
    rcg_slope, rcg_sig = {}, {}
    for ch in dcg_demux.BAYER:
        kh = legs_out["HG"]["channels"][ch]["K_e_per_DN"]
        kl = legs_out["LG"]["channels"][ch]["K_e_per_DN"]
        rcg_slope[ch] = float(kl / kh) if kh else float("nan")
        if ref is not None:
            nh = ref["meas"]["HG"][ch]["mean"] - dark["HG"][ch]["mean"]
            nl = ref["meas"]["LG"][ch]["mean"] - dark["LG"][ch]["mean"]
            rcg_sig[ch] = float(nh / nl) if nl else float("nan")

    return {
        "legs": legs_out,
        "Rcg": {"from_ptc_slopes_KLG_over_KHG": rcg_slope,
                "from_signal_ratio_HG_over_LG": rcg_sig, "ref_level": ref_level},
        "notes": [
            "HG leg = HCG, LG leg = LCG (valid when EXP_GAIN=0 so legs differ by conversion gain only).",
            "Rcg from PTC slopes and from the signal ratio should agree (~2.4; datasheet 2.4-2.9).",
            "signal_at_SNR1_e is sensor-referred (electrons); lux-referred SNR1s needs calibrated "
            "illumination at the sensor.",
        ],
    }


def _load_manifest(path):
    """Manifest: {width?, dtype?, layout{hg_rows,lg_rows,ob_top?,gap?,hg_first?},
                  dark:[files], points:[{level, frames:[files]}]}."""
    with open(path, encoding="utf-8") as f:
        m = json.load(f)
    w, dt = m.get("width"), m.get("dtype", "<u2")
    layout = m["layout"]
    dark = measure([load_frame(p, w, dt) for p in m["dark"]], layout)
    points = [{"level": pt["level"],
               "meas": measure([load_frame(p, w, dt) for p in pt["frames"]], layout)}
              for pt in m["points"]]
    return points, dark, m.get("ref_level")


def main(argv=None):
    ap = argparse.ArgumentParser(description="DCG per-leg static characterization (HCG/LCG)")
    ap.add_argument("manifest", help="JSON manifest of dark + sweep points (see module docstring)")
    ap.add_argument("--out", default=None, help="write the report JSON here")
    args = ap.parse_args(argv)
    points, dark, ref = _load_manifest(args.manifest)
    report = characterize(points, dark, ref_level=ref)
    text = json.dumps(report, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
        print("wrote", args.out)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
