#!/usr/bin/env python3
"""Per-leg static characterization for IMX678 DCG (Clear HDR) captures.

Splits each stacked Clear HDR RAW16 frame into HG (HCG) and LG (LCG) legs (via dcg_demux) and,
per leg and per RGGB channel, derives:
  - pedestal + read noise from per-frame optical-black (masked) rows -- preferred -- or a dark point,
  - photon-transfer curve -> conversion gain K (e-/DN) and full-well (e-),
  - SNR vs signal and the sensor-referred SNR=1 point (electrons),
  - conversion-gain ratio Rcg from BOTH the PTC slopes (K_LG/K_HG) and the net-signal ratio,
  - dynamic range: per-leg DR, combined DCG DR (LG full-well over HG read floor), the HG:LG ratio,
    and the hand-off SNR -- i.e. whether the two legs fill the device DR contiguously.

Optical-black per-frame correction: give the layout `ob_hg`/`ob_lg` (masked row spans per leg,
absolute in the stacked frame). Each leg has its own OB because HCG and LCG sit at different black
levels; the OB mean is the per-frame pedestal (tracks dark drift) and the OB temporal variance is
the read noise -- no separate dark capture needed. Falls back to a `dark` point if OB is absent.

Photon transfer (two-frame difference removes fixed-pattern noise):
    var_DN = (1/K) * signal_DN + read_DN**2      -> slope = 1/K, intercept = read_DN**2
    K [e-/DN] = 1/slope ; read_e = K*read_DN ; full_well_e = K*signal_sat_DN
HCG has higher conversion gain -> smaller K, steeper slope; Rcg = K_LCG/K_HCG = slope_HG/slope_LG.

DR note: DCG (EXP_GAIN=0) lowers the shadow floor by ~Rcg over a single LCG read (gain_over_LCG_dB
~ 20*log10(Rcg) ~ 7.6 dB for Rcg=2.4). Dual-exposure (DExp/DOL, e-con used 16x = ~24 dB) extends the
top instead; DCG+DExp multiplies the two. Sensor-referred SNR=1 (electrons) is not lux-referred SNR1s
(needs calibrated illumination at the sensor -- see imx678-lowlight-013lx.md). Requires numpy.
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
DEMUX_KEYS = ("hg_rows", "lg_rows", "ob_top", "gap", "hg_first")
DOL_16X_DR_DB = 20.0 * math.log10(16.0)  # e-con DOL reference (~24.1 dB) for comparison


def load_frames(path, width=None, dtype="<u2"):
    """List of 2-D uint16 frames. .npy (N,H,W)->N frames (board save format), (H,W)->1, (K,)->reshape."""
    if path.endswith(".npy"):
        a = np.load(path)
        if a.ndim == 3:
            return [np.ascontiguousarray(a[i]) for i in range(a.shape[0])]
        if a.ndim == 2:
            return [np.ascontiguousarray(a)]
        if a.ndim == 1:
            if not width:
                raise ValueError("1-D .npy needs width")
            return [a.reshape(-1, width)]
        raise ValueError("unexpected .npy ndim %d" % a.ndim)
    if not width:
        raise ValueError("raw frame needs width")
    return [np.fromfile(path, dtype=np.dtype(dtype)).reshape(-1, width)]


def _channels(leg):
    return {n: leg[pr::2, pc::2].astype(np.float64) for n, (pr, pc) in dcg_demux._PHASE.items()}


def _ob_planes(frame, ranges):
    """Per-RGGB-channel planes of the optical-black rows (or None if no ranges)."""
    ranges = dcg_demux._as_ranges(ranges)
    if ranges is None:
        return None
    rows = np.concatenate([np.asarray(frame[a:b]) for a, b in ranges], axis=0).astype(np.float64)
    return {n: rows[pr::2, pc::2] for n, (pr, pc) in dcg_demux._PHASE.items()}


def _temporal_var(planes):
    """FPN-free temporal variance from a 2-frame difference; spatial fallback for a single frame."""
    if len(planes) >= 2:
        return float(np.var(planes[0] - planes[1]) / 2.0)
    return float(planes[0].var())


def measure(frames, layout):
    """Per-leg, per-channel mean/var (+ OB pedestal & OB read noise if layout has ob_hg/ob_lg)."""
    dk = {k: layout[k] for k in DEMUX_KEYS if k in layout}
    ob_ranges = {"HG": layout.get("ob_hg"), "LG": layout.get("ob_lg")}
    act = {"HG": [], "LG": []}
    obp = {"HG": [], "LG": []}
    for f in frames:
        hg, lg = dcg_demux.demux(f, **dk)
        act["HG"].append(_channels(hg))
        act["LG"].append(_channels(lg))
        obp["HG"].append(_ob_planes(f, ob_ranges["HG"]))
        obp["LG"].append(_ob_planes(f, ob_ranges["LG"]))
    out = {}
    for leg in ("HG", "LG"):
        out[leg] = {}
        has_ob = obp[leg][0] is not None
        for name in dcg_demux.BAYER:
            planes = [c[name] for c in act[leg]]
            rec = {"mean": float(np.mean([p.mean() for p in planes])),
                   "var": _temporal_var(planes), "nframes": len(planes)}
            if has_ob:
                obpl = [o[name] for o in obp[leg]]
                rec["ob_mean"] = float(np.mean([p.mean() for p in obpl]))
                rec["ob_var"] = _temporal_var(obpl)
                rec["net"] = rec["mean"] - rec["ob_mean"]
            out[leg][name] = rec
    return out


def ptc_fit(signal_dn, var_dn):
    """Linear photon-transfer fit over the rising (unsaturated) region -> K, read, sat signal."""
    s = np.asarray(signal_dn, float)
    v = np.asarray(var_dn, float)
    order = np.argsort(s)
    s, v = s[order], v[order]
    peak = int(np.argmax(v))
    sl, vl = s[: max(peak + 1, 2)], v[: max(peak + 1, 2)]
    slope, intercept = np.linalg.lstsq(np.vstack([sl, np.ones_like(sl)]).T, vl, rcond=None)[0]
    K = 1.0 / slope if slope > 0 else float("nan")
    read_dn = math.sqrt(intercept) if intercept > 0 else 0.0
    return {"K_e_per_DN": float(K), "slope": float(slope), "read_DN_ptc": float(read_dn),
            "read_e_ptc": float(read_dn * K) if slope > 0 else float("nan"),
            "n_ptc": int(sl.size), "sat_signal_DN": float(s[peak])}


def _snr1_electrons(read_e):
    return float((1.0 + math.sqrt(1.0 + 4.0 * read_e * read_e)) / 2.0)


def _leg_channel(points, dark, leg, ch):
    """Resolve pedestal + read noise (OB-preferred), net signal series, and the PTC fit."""
    p0 = points[0]["meas"][leg][ch]
    if "net" in p0:  # per-frame optical-black correction
        ped = float(np.mean([p["meas"][leg][ch]["ob_mean"] for p in points]))
        read_dn = math.sqrt(max(float(np.mean([p["meas"][leg][ch]["ob_var"] for p in points])), 0.0))
        sig = [p["meas"][leg][ch]["net"] for p in points]
        src = "optical-black (per-frame)"
    elif dark is not None:
        ped = dark[leg][ch]["mean"]
        read_dn = math.sqrt(max(dark[leg][ch]["var"], 0.0))
        sig = [p["meas"][leg][ch]["mean"] - ped for p in points]
        src = "dark point"
    else:
        raise ValueError("need layout ob_hg/ob_lg (masked rows) or a dark point for the pedestal")
    var = [p["meas"][leg][ch]["var"] for p in points]
    return ped, read_dn, src, sig, var, ptc_fit(sig, var)


def characterize(points, dark=None, ref_level=None, exp_gain_db=0.0):
    """Full per-leg report + Rcg + dynamic range. points: [{'level': L, 'meas': measure(...)}]."""
    legs_out = {}
    for leg in ("HG", "LG"):
        chans = {}
        for ch in dcg_demux.BAYER:
            ped, read_dn, src, sig, var, fit = _leg_channel(points, dark, leg, ch)
            K = fit["K_e_per_DN"]
            read_e = read_dn * K if math.isfinite(K) else float("nan")
            snr = [s / math.sqrt(v) if v > 0 else float("nan") for s, v in zip(sig, var)]
            chans[ch] = {
                "pedestal_DN": float(ped),
                "read_DN": float(read_dn),
                "read_e": float(read_e),
                "read_noise_source": src,
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

    if ref_level is None and points:
        ref_level = points[len(points) // 2]["level"]
    ref = next((p for p in points if p["level"] == ref_level), points[-1] if points else None)
    rcg_slope, rcg_sig = {}, {}
    for ch in dcg_demux.BAYER:
        kh = legs_out["HG"]["channels"][ch]["K_e_per_DN"]
        kl = legs_out["LG"]["channels"][ch]["K_e_per_DN"]
        rcg_slope[ch] = float(kl / kh) if kh else float("nan")
        if ref is not None:
            mh, ml = ref["meas"]["HG"][ch], ref["meas"]["LG"][ch]
            nh = mh.get("net", mh["mean"] - legs_out["HG"]["channels"][ch]["pedestal_DN"])
            nl = ml.get("net", ml["mean"] - legs_out["LG"]["channels"][ch]["pedestal_DN"])
            rcg_sig[ch] = float(nh / nl) if nl else float("nan")

    return {
        "legs": legs_out,
        "Rcg": {"from_ptc_slopes_KLG_over_KHG": rcg_slope,
                "from_signal_ratio_HG_over_LG": rcg_sig, "ref_level": ref_level},
        "dynamic_range": dynamic_range(legs_out, exp_gain_db),
        "notes": [
            "HG=HCG, LG=LCG (valid when EXP_GAIN=0 so legs differ by conversion gain only).",
            "Rcg from PTC slopes and from the signal ratio should agree (~2.4; datasheet 2.4-2.9).",
            "DR_combined uses the LG full-well (highlights) over the HG read floor (shadows). "
            "gain_over_LCG_dB is the DCG shadow-floor gain vs a single LCG read (~20*log10(Rcg)). "
            "handoff_SNR is the LG SNR where HG saturates -- >>1 means the legs fill DR contiguously.",
            "DExp/DOL reference (e-con used 16x) ~= %.1f dB and extends the top; DCG+DExp multiplies "
            "the two." % DOL_16X_DR_DB,
            "signal_at_SNR1_e is sensor-referred (electrons); lux-referred SNR1s needs calibrated "
            "illumination at the sensor.",
        ],
    }


def dynamic_range(legs_out, exp_gain_db=0.0):
    """Per-leg + combined DCG dynamic range and the HG:LG ratio, per channel."""
    exp_factor = 10.0 ** (exp_gain_db / 20.0)

    def db(x):
        return float(20.0 * math.log10(x)) if (x and x > 0 and math.isfinite(x)) else float("nan")

    out = {}
    for ch in dcg_demux.BAYER:
        hg = legs_out["HG"]["channels"][ch]
        lg = legs_out["LG"]["channels"][ch]
        fw_hg, fw_lg = hg["full_well_e"], lg["full_well_e"]
        rn_hg, rn_lg = hg["read_e"], lg["read_e"]
        kh, kl = hg["K_e_per_DN"], lg["K_e_per_DN"]
        lh = (kl / kh) * exp_factor if kh else float("nan")
        out[ch] = {
            "DR_HG_dB": db(fw_hg / rn_hg) if rn_hg else float("nan"),
            "DR_LG_dB": db(fw_lg / rn_lg) if rn_lg else float("nan"),
            "DR_combined_dB": db(fw_lg / rn_hg) if rn_hg else float("nan"),
            "gain_over_LCG_dB": db(rn_lg / rn_hg) if rn_hg else float("nan"),
            "LH_ratio": float(lh),
            "LH_ratio_stops": float(math.log2(lh)) if lh and lh > 0 else float("nan"),
            "handoff_SNR": float(fw_hg / math.sqrt(fw_hg + rn_lg * rn_lg)) if fw_hg > 0 else float("nan"),
            "vs_DOL16x_dB": db(fw_lg / rn_hg) - DOL_16X_DR_DB if rn_hg else float("nan"),
        }
    return out


def _load_manifest(path):
    """Manifest: {width?, dtype?, exp_gain_db?, layout{hg_rows,lg_rows,ob_top?,gap?,hg_first?,
                  ob_hg?,ob_lg?}, dark?:[files], points:[{level, frames:[files]}], ref_level?}."""
    with open(path, encoding="utf-8") as f:
        m = json.load(f)
    w, dt = m.get("width"), m.get("dtype", "<u2")
    layout = m["layout"]

    def load_pt(files):
        frames = []
        for p in files:
            frames.extend(load_frames(p, w, dt))
        return measure(frames, layout)

    dark = load_pt(m["dark"]) if m.get("dark") else None
    points = [{"level": pt["level"], "meas": load_pt(pt["frames"])} for pt in m["points"]]
    return points, dark, m.get("ref_level"), float(m.get("exp_gain_db", 0.0))


def main(argv=None):
    ap = argparse.ArgumentParser(description="DCG per-leg static characterization (HCG/LCG) + DR")
    ap.add_argument("manifest", help="JSON manifest of dark/OB + sweep points (see module docstring)")
    ap.add_argument("--out", default=None, help="write the report JSON here")
    args = ap.parse_args(argv)
    points, dark, ref, exp_gain_db = _load_manifest(args.manifest)
    report = characterize(points, dark, ref_level=ref, exp_gain_db=exp_gain_db)
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
