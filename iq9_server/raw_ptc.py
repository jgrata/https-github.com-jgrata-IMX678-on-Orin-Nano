#!/usr/bin/env python3
"""Single-plane RAW PTC / OETF / SNR characterization for the IMX678 (normal RAW, non-DCG).

Per RGGB channel, from a light sweep (>= 2 frames/level; the two-frame difference removes
fixed-pattern noise), derives:
  - pedestal + read noise (from a dark point/level, else the darkest sweep point),
  - photon-transfer curve -> conversion gain K (e-/DN), read noise (e-), full-well (e-),
  - SNR vs signal, max SNR, and the SNR=1 point (DN + sensor-referred electrons),
  - OETF linearity (signal vs light): R^2 and max deviation from a line, saturation knee,
  - per-channel dynamic range (full-well / read floor).

Photon transfer:  var_DN = (1/K) * signal_DN + read_DN^2   -> slope = 1/K, intercept = read_DN^2
  K [e-/DN] = 1/slope ; read_e = K*read_DN ; full_well_e = K*signal_sat_DN.
K / read / full-well / SNR are SIGNAL-based -> robust to the light source's own nonlinearity.
The OETF (signal-vs-light) DOES need a linear light reference; a bare DMX-level sweep reflects
sensor x lamp, so read the lamp-independent linearity from the PTC's straightness and use the
OETF mainly for the saturation knee + gross shape. Sensor-referred SNR=1 (electrons) is NOT the
lux-referred SNR1s (that needs calibrated illumination at the sensor). Requires numpy.
"""
from __future__ import annotations
import argparse
import json
import math
import os

import numpy as np

_PHASE = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}   # RGGB (R at (0,0), B at (1,1))
BAYER = ("R", "G1", "G2", "B")


def split_rggb(frame):
    return {n: frame[pr::2, pc::2].astype(np.float64) for n, (pr, pc) in _PHASE.items()}


def _temporal_var(planes):
    """FPN-free temporal variance from frame differences, AVERAGED over all consecutive pairs
    (more pairs -> tighter estimate). Each difference is invariant to a spatially-uniform
    brightness change, so LED switching-PSU / DMX ripple that dims/brightens the whole frame
    cancels here and does NOT inflate the noise. Spatial fallback for a single frame."""
    if len(planes) >= 2:
        diffs = [np.var(planes[i] - planes[i + 1]) / 2.0 for i in range(len(planes) - 1)]
        return float(np.mean(diffs))
    return float(planes[0].var())


def measure(frames, roi=None):
    """Per-RGGB mean + temporal variance over an ROI. roi = (y0, y1, x0, x1) in FULL-frame px.
    Also returns mean_cv_pct: the frame-to-frame spread of the ROI MEAN (%), i.e. the
    spatially-uniform flicker amplitude — diagnostic only (the 2-frame-diff var above rejects it)."""
    planes = {ch: [] for ch in BAYER}
    for f in frames:
        g = f[roi[0]:roi[1], roi[2]:roi[3]] if roi else f
        for ch, (pr, pc) in _PHASE.items():
            planes[ch].append(g[pr::2, pc::2].astype(np.float64))
    out = {}
    for ch in BAYER:
        pmeans = [p.mean() for p in planes[ch]]
        mean = float(np.mean(pmeans))
        mean_cv = float(100.0 * np.std(pmeans) / max(mean, 1e-9)) if len(pmeans) > 1 else 0.0
        out[ch] = {"mean": mean, "var": _temporal_var(planes[ch]),
                   "n": len(planes[ch]), "mean_cv_pct": mean_cv}
    return out


def ptc_fit(signal_dn, var_dn):
    """Linear photon-transfer fit over the rising (unsaturated) region -> K, read, sat."""
    s = np.asarray(signal_dn, float)
    v = np.asarray(var_dn, float)
    order = np.argsort(s)
    s, v = s[order], v[order]
    peak = int(np.argmax(v))                       # variance peaks at ~full-well, then drops
    sl, vl = s[: max(peak + 1, 2)], v[: max(peak + 1, 2)]
    slope, intercept = np.linalg.lstsq(np.vstack([sl, np.ones_like(sl)]).T, vl, rcond=None)[0]
    K = 1.0 / slope if slope > 0 else float("nan")
    read_dn = math.sqrt(intercept) if intercept > 0 else 0.0
    return {"K_e_per_DN": float(K), "slope": float(slope), "read_DN_ptc": float(read_dn),
            "n_ptc": int(sl.size), "sat_signal_DN": float(s[peak])}


def _snr1_electrons(read_e):
    """Sensor-referred SNR=1 signal (electrons): signal / sqrt(signal + read^2) = 1."""
    return float((1.0 + math.sqrt(1.0 + 4.0 * read_e * read_e)) / 2.0)


def _snr1_dn(signal_dn, snr):
    """Measured SNR=1 crossing in DN (interpolated), or nan if the sweep doesn't span it."""
    s = np.asarray(signal_dn, float)
    r = np.asarray(snr, float)
    order = np.argsort(s)
    s, r = s[order], r[order]
    for i in range(1, len(r)):
        if (r[i - 1] - 1.0) <= 0 < (r[i] - 1.0) or (r[i - 1] - 1.0) < 0 <= (r[i] - 1.0):
            t = (1.0 - r[i - 1]) / (r[i] - r[i - 1]) if r[i] != r[i - 1] else 0.0
            return float(s[i - 1] + t * (s[i] - s[i - 1]))
    return float("nan")


def oetf_fit(light, signal):
    """Linearity of signal vs light over the unsaturated region: R^2 + max deviation %."""
    l = np.asarray(light, float)
    s = np.asarray(signal, float)
    order = np.argsort(l)
    l, s = l[order], s[order]
    peak = int(np.argmax(s))                       # saturation flattens the top
    lr, sr = (l[: peak + 1], s[: peak + 1]) if peak >= 1 else (l, s)
    if lr.size < 2:
        return {"r2": float("nan"), "max_dev_pct": float("nan"), "n_linear": int(lr.size)}
    (m, b), *_ = np.linalg.lstsq(np.vstack([lr, np.ones_like(lr)]).T, sr, rcond=None)
    pred = m * lr + b
    ss_res = float(np.sum((sr - pred) ** 2))
    ss_tot = float(np.sum((sr - sr.mean()) ** 2))
    return {"r2": float(1.0 - ss_res / ss_tot) if ss_tot > 0 else float("nan"),
            "max_dev_pct": float(np.max(np.abs(sr - pred)) / max(sr.max(), 1e-9) * 100.0),
            "slope_DN_per_light": float(m), "intercept_DN": float(b),
            "n_linear": int(lr.size), "sat_signal_DN": float(s[peak])}


def characterize(points, dark=None, black_level=None):
    """points: [{'light': L, 'meas': measure(...)}]. Returns per-channel PTC/OETF/SNR/DR."""
    out = {}
    for ch in BAYER:
        if dark is not None:                       # explicit dark frames (lights off)
            ped = dark[ch]["mean"]
            read_dn = math.sqrt(max(dark[ch]["var"], 0.0))
            src = "dark frames"
        else:                                      # pedestal/read from the darkest sweep point
            dk = min(points, key=lambda p: p["meas"][ch]["mean"])
            ped = black_level if black_level is not None else dk["meas"][ch]["mean"]
            read_dn = math.sqrt(max(dk["meas"][ch]["var"], 0.0))
            src = "darkest sweep point" if black_level is None else "fixed black_level + darkest var"
        sig = [p["meas"][ch]["mean"] - ped for p in points]
        var = [p["meas"][ch]["var"] for p in points]
        light = [p["light"] for p in points]
        fit = ptc_fit(sig, var)
        K = fit["K_e_per_DN"]
        read_e = read_dn * K if math.isfinite(K) else float("nan")
        snr = [s / math.sqrt(v) if v > 0 else float("nan") for s, v in zip(sig, var)]
        oetf = oetf_fit(light, [p["meas"][ch]["mean"] for p in points])
        fw_e = K * fit["sat_signal_DN"] if math.isfinite(K) else float("nan")
        out[ch] = {
            "pedestal_DN": float(ped),
            "read_DN": float(read_dn),
            "read_e": float(read_e),
            "read_noise_source": src,
            "K_e_per_DN": K,
            "gain_slope_var_per_DN": fit["slope"],
            "full_well_e": float(fw_e),
            "sat_signal_DN": fit["sat_signal_DN"],
            "max_SNR": float(np.nanmax(snr)) if snr else float("nan"),
            "SNR1_DN_measured": _snr1_dn(sig, snr),
            "SNR1_e_sensor": _snr1_electrons(read_e) if math.isfinite(read_e) else float("nan"),
            "DR_dB": float(20.0 * math.log10(fw_e / read_e)) if (math.isfinite(fw_e) and read_e > 0) else float("nan"),
            "oetf": oetf,
            "n_points": len(points),
        }
    notes = [
        "PTC K/read/full-well/SNR are signal-based (robust to lamp nonlinearity).",
        "oetf.r2/max_dev_pct measure signal-vs-LIGHT linearity: with a DMX-level sweep this is "
        "sensor x lamp -> for a pure sensor OETF use a linear light reference (photometer / ND).",
        "PTC-line straightness is the lamp-independent linearity check.",
        "SNR1_e_sensor is sensor-referred (electrons); lux-referred SNR1s needs calibrated illumination.",
    ]
    return {"channels": out, "notes": notes}


def lux_metrics(ch_report):
    """From a per-channel characterize() report whose OETF 'light' axis is LUX, derive the
    lux-referred metrics: responsivity (e-/lux) = K * (DN-per-lux slope), and lux-referred
    SNR1s = sensor-referred SNR=1 electrons / responsivity. Returns {} if inputs are unusable
    (no K, non-linear fit, or the sweep light axis was DMX level rather than lux)."""
    K = ch_report.get("K_e_per_DN")
    slope = ch_report.get("oetf", {}).get("slope_DN_per_light")
    snr1_e = ch_report.get("SNR1_e_sensor")
    if not (K and slope and math.isfinite(K) and math.isfinite(slope) and slope > 0):
        return {}
    resp = float(K * slope)
    out = {"responsivity_e_per_lux": resp}
    if snr1_e is not None and math.isfinite(snr1_e) and resp > 0:
        out["snr1s_lux"] = float(snr1_e / resp)
    return out


def spec_snr1s(snr1s, meas_fnum, spec_fnum, meas_integ_s, spec_integ_s,
               meas_refl=None, spec_refl=None):
    """Normalize a measured SNR1s (scene lux at SNR=1) to a spec test condition. Converts the
    REQUIRED scene illuminance from our optics/target to the spec's:

        SNR1s_spec = SNR1s_meas * (t_meas/t_spec) * (N_spec/N_meas)^2 * (rho_meas/rho_spec)

    - integration: a shorter spec exposure collects fewer e- -> needs MORE lux  (t_meas/t_spec)
    - aperture:    a faster spec f/# collects more light -> needs LESS lux      (N_spec/N_meas)^2
    - reflectance: a darker spec target reflects less to the sensor -> MORE lux (rho_meas/rho_spec)
                   applied only if BOTH reflectances are given (e.g. our white 0.9 -> Sony 0.18).
    Sony SNR1s call-out: 100 lux at the surface of an 18% gray target, 3200K, F1.4, 1/60s."""
    if snr1s is None or not math.isfinite(snr1s):
        return None
    f = (meas_integ_s / spec_integ_s) * (spec_fnum / meas_fnum) ** 2
    if meas_refl and spec_refl:
        f *= (meas_refl / spec_refl)
    return float(snr1s * f)


# ---- manifest CLI (mirrors dcg_characterize) --------------------------------
def _load_frames(path, width=None, dtype="<u2"):
    if path.endswith(".npy"):
        a = np.load(path)
        if a.ndim == 3:
            return [np.ascontiguousarray(a[i]) for i in range(a.shape[0])]
        if a.ndim == 2:
            return [np.ascontiguousarray(a)]
        raise ValueError("unexpected .npy ndim %d" % a.ndim)
    if not width:
        raise ValueError("raw frame needs width")
    return [np.fromfile(path, dtype=np.dtype(dtype)).reshape(-1, width)]


def _load_manifest(path):
    with open(path, encoding="utf-8") as f:
        m = json.load(f)
    w, dt, roi = m.get("width"), m.get("dtype", "<u2"), m.get("roi")

    def load_pt(files):
        frames = []
        for p in files:
            frames.extend(_load_frames(p, w, dt))
        return measure(frames, roi)

    dark = load_pt(m["dark"]) if m.get("dark") else None
    points = [{"light": pt["light"], "meas": load_pt(pt["frames"])} for pt in m["points"]]
    return points, dark, m.get("black_level")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Single-plane RAW PTC/OETF/SNR characterization")
    ap.add_argument("manifest")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    points, dark, bl = _load_manifest(args.manifest)
    report = characterize(points, dark, black_level=bl)
    text = json.dumps(report, indent=2)
    if args.out:
        open(args.out, "w", encoding="utf-8").write(text)
        print("wrote", args.out)
    else:
        print(text)
    return 0


# ---- synthetic self-test ----------------------------------------------------
def _selftest():
    rng = np.random.default_rng(0)
    K_true, read_e_true, ped, fw_dn = 1.6, 2.2, 200.0, 3600.0     # e-/DN, e-, DN, full-well DN
    read_dn = read_e_true / K_true
    H, W = 200, 300
    lights = [0.0, 5, 10, 20, 40, 80, 160, 320, 640, 1280]        # arbitrary linear light units
    e_per_light = 4.0                                             # e- per light unit per pixel
    points = []
    for L in lights:
        sig_e = L * e_per_light
        sig_dn = sig_e / K_true
        var_dn = sig_dn / K_true + read_dn ** 2                   # shot + read, in DN^2
        frames = []
        for _ in range(2):
            fr = ped + sig_dn + rng.normal(0, math.sqrt(var_dn), (H, W))
            frames.append(np.clip(fr, 0, ped + fw_dn).astype(np.uint16))
        points.append({"light": L, "meas": measure(frames)})
    rep = characterize(points)["channels"]["R"]
    lm = lux_metrics(rep)                          # light axis == e_per_light units -> responsivity ~= e_per_light
    sp = spec_snr1s(1.0, 2.0, 1.4, 0.020, 1.0 / 60.0)   # f/2->f/1.4, 20ms->1/60s: 1.20*0.49=0.588
    ok = (abs(rep["K_e_per_DN"] - K_true) < 0.15 and abs(rep["read_e"] - read_e_true) < 0.6
          and rep["oetf"]["r2"] > 0.999 and rep["full_well_e"] > 0
          and abs(lm.get("responsivity_e_per_lux", 0.0) - e_per_light) < 0.3
          and abs(sp - 0.588) < 0.005)
    print("SELFTEST %s: K=%.3f (true %.2f) read_e=%.2f (true %.2f) full_well_e=%.0f "
          "OETF r2=%.5f maxdev=%.2f%% maxSNR=%.1f SNR1_e=%.2f DR=%.1fdB resp=%.2f (true %.2f)"
          % ("PASS" if ok else "FAIL", rep["K_e_per_DN"], K_true, rep["read_e"], read_e_true,
             rep["full_well_e"], rep["oetf"]["r2"], rep["oetf"]["max_dev_pct"],
             rep["max_SNR"], rep["SNR1_e_sensor"], rep["DR_dB"],
             lm.get("responsivity_e_per_lux", float("nan")), e_per_light))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        raise SystemExit(_selftest())
    raise SystemExit(main())
