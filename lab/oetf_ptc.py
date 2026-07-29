"""OETF + Photon-Transfer-Curve characterization of the IMX678 (Jetson raw path),
using DMX-controlled LED flat-field illumination.

Make-shift setup: a large diffuse reflective target filling the FOV, lit by the
Waveform-3082 LED panels (DMX). Not an integrating sphere, but a good uniform field.

METHOD (classic exposure-sweep PTC):
  Fix a stable flat field (DMX), sweep CAMERA EXPOSURE (precisely linear), and at each
  exposure grab TWO frames. Per Bayer channel, over a central ROI:
    signal   = mean(f1)                       [DN, minus black for PTC]
    var_temporal = var(f1 - f2) / 2           [DN^2]  (difference removes fixed-pattern noise)
  OETF : signal vs exposure  -> linearity (R^2), black level (intercept), saturation knee.
  PTC  : var vs (signal-black) -> shot-noise slope m in the mid region:
           conversion gain  K = 1/m           [e-/DN]
           read noise       = sqrt(var@0) * K  [e- rms]
           full well        = (sat-black) * K  [e-]
  SNR1s: Sony low-light figure of merit = TARGET illuminance (lux) for SNR=1 at 1/60s.
           SNR=1 signal S1 solves S1^2 = m*S1 + var@0 (K cancels: SNR = sig/sqrt(var));
           map S1 through the OETF slope to an exposure, then via the DMX->lux LUT to a
           lux*s product, /(1/60 s) -> SNR1s. Normalizes to Sony's F1.4 / 18% grey with
           --fnum/--reflectance. Match Sony's 3200K source with --illum tungsten.

Runs on the PC: DMX is local (lab/dmx_lights), raw frames come from the Jetson
image_server over the wire (camera_client). Close QLC+ first (FTDI is exclusive).

Usage:
  python lab/oetf_ptc.py --check                 # just report flat-field uniformity
  python lab/oetf_ptc.py --illum d65 --level 48 --emin 0.4 --emax 64 --points 24 --force
  # SNR1s comparable to Sony's spec (3200K, F1.4, 18% grey, 1/60s):
  python lab/oetf_ptc.py --illum tungsten --level 48 --emin 0.4 --emax 64 \
      --fnum 1.8 --reflectance 0.18 --force
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metro_server", "webui"))   # camera_client
sys.path.insert(0, _HERE)                                                # dmx_lights
import camera_client
import dmx_lights as dl

JET_HOST, JET_PORT = "192.168.99.2", 9000
CHANNELS = {"R": (0, 0), "Gr": (0, 1), "Gb": (1, 0), "B": (1, 1)}        # (row,col) offset in the 2x2 CFA


def plane(frame, ch):
    r, c = CHANNELS[ch]
    return frame[r::2, c::2]


def roi_of(pl, frac=0.25):
    h, w = pl.shape
    rh, rw = int(h * frac / 2), int(w * frac / 2)
    return pl[h // 2 - rh:h // 2 + rh, w // 2 - rw:w // 2 + rw]


def lux_from_lut(illum, level):
    """Target illuminance (lux) at a DMX level, interpolated from the calibration
    LUT lab/dmx_lux_<illum>.json (built by dmx_lux_cal.py). None if no LUT exists.
    NB this is illuminance AT THE TARGET (includes ambient), so downstream SNR=1
    lux folds in target reflectance + lens f/# -- a system, not sensor, spec."""
    p = os.path.join(_HERE, "dmx_lux_%s.json" % illum)
    if not os.path.exists(p):
        return None
    with open(p) as fh:
        d = json.load(fh)
    return float(np.interp(level, d["dmx"], d["lux"]))


def uniformity(frame):
    """Central-ROI CV + 9-tile illumination spread on the green (Gr) plane."""
    g = plane(frame, "Gr")
    roi = roi_of(g, 0.25)
    gh, gw = g.shape
    tiles = np.array([[g[y * gh // 3:(y + 1) * gh // 3, x * gw // 3:(x + 1) * gw // 3].mean()
                       for x in range(3)] for y in range(3)]).ravel()
    return (roi.std() / max(roi.mean(), 1.0),
            (tiles.max() - tiles.min()) / max(tiles.mean(), 1.0),
            roi.mean())


def capture_frames(c, n, delay=0.08, flush=0):
    """Grab n independent frames; return (list of float64 arrays, white level).
    `flush` discards that many frames first -- the pipeline is ~2-3 frames deep
    (~0.5 s/frame), so after a big exposure/gain jump the first captures are still
    in-flight at the OLD setting; measuring them inflates the temporal variance and
    corrupts the mean. Flush clears the transition before we measure."""
    for _ in range(flush):
        c.capture()
    frames, maxv = [], None
    for i in range(n):
        f, mv = c.capture()
        frames.append(f.astype(np.float64))
        maxv = mv if maxv is None else maxv
        if i < n - 1:
            time.sleep(delay)                          # ensure an independent frame
    return frames, maxv


def channel_stats(frames, chn, roi_frac):
    """Robust central-ROI signal + temporal variance for one Bayer channel.

    signal = median of per-frame ROI means (rejects an occasional bad frame).
    var    = median of consecutive-pair difference variances var(f_k - f_{k+1})/2
             -- the pair difference removes fixed-pattern noise, and the median
             over pairs rejects a single torn/dropped frame (which corrupts <=2
             of the n-1 pairs). This is what killed the old 2-frame PTC.
    """
    rois = [roi_of(plane(f, chn), roi_frac) for f in frames]
    means = np.array([r.mean() for r in rois])
    diffs = [((rois[k] - rois[k + 1]) ** 2).mean() / 2.0 for k in range(len(rois) - 1)]
    return float(np.median(means)), float(np.median(diffs))


def run_sweep(c, exps_ms, frames, roi, settle, verbose=True, flush=2):
    """Set each exposure, grab `frames` frames, return (rows, white-level maxv).
    Shared by the exposure-sweep tool and the gain-sweep driver (one source of
    truth for how a PTC row is measured). `flush` drops in-flight frames after each
    exposure change (see capture_frames)."""
    rows, maxv = [], None
    for e_ms in exps_ms:
        c.set_params({"exposure_ns": int(e_ms * 1e6)}); time.sleep(settle)
        fs, maxv = capture_frames(c, frames, flush=flush)
        rec = {"exp_ms": round(float(e_ms), 4)}
        for chn in ("R", "Gr", "Gb", "B"):
            mean, var = channel_stats(fs, chn, roi)
            rec["%s_mean" % chn] = round(mean, 3)
            rec["%s_var" % chn] = round(var, 4)
        rows.append(rec)
        if verbose:
            print("  %.2f ms  Gr mean=%.1f var=%.1f%s" % (e_ms, rec["Gr_mean"], rec["Gr_var"],
                  "  (SAT)" if rec["Gr_mean"] >= 0.98 * maxv else ""))
    return rows, maxv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="report flat-field uniformity and exit")
    ap.add_argument("--illum", choices=["d65", "tungsten", "keep"], default="d65")
    ap.add_argument("--level", type=int, default=128, help="DMX level 0-255 for the chosen panel")
    ap.add_argument("--emin", type=float, default=0.3, help="min exposure ms")
    ap.add_argument("--emax", type=float, default=120.0, help="max exposure ms")
    ap.add_argument("--points", type=int, default=22)
    ap.add_argument("--frames", type=int, default=8, help="frames per exposure (>=2; median-of-pairs var)")
    ap.add_argument("--roi", type=float, default=0.25, help="central ROI fraction per plane")
    ap.add_argument("--settle", type=float, default=1.3, help="seconds after an exposure change")
    ap.add_argument("--out", default=os.path.join(_HERE, "ptc_result.csv"))
    ap.add_argument("--force", action="store_true", help="run even if the field looks non-uniform")
    # SNR1s (Sony low-light figure of merit): illuminance at the target for SNR=1.
    # Sony's reference conditions are 3200K, 18% grey, F1.4, 1/60s. Give --fnum and
    # --reflectance to also print the value normalized to F1.4 / 18% grey.
    ap.add_argument("--fnum", type=float, default=None, help="lens F-number used (for SNR1s F1.4 normalization)")
    ap.add_argument("--reflectance", type=float, default=None,
                    help="target reflectance 0-1 (e.g. 0.18 grey card; for SNR1s 18%% normalization)")
    a = ap.parse_args()

    # DMX flat field
    d = None
    if a.illum != "keep":
        d = dl.OpenDMX().start(rate_hz=11)
        ch = dl.CH_D65 if a.illum == "d65" else dl.CH_TUNGSTEN
        other = dl.CH_TUNGSTEN if a.illum == "d65" else dl.CH_D65
        d.set_many({ch: a.level, other: 0})
        print("DMX: %s(ch%d)=%d, other off; streaming @11Hz" % (a.illum, ch, a.level))
        time.sleep(1.5)

    c = camera_client.CameraClient(JET_HOST, JET_PORT, timeout=30)
    c.connect()
    orig = int(c.info().get("exposure_ns", 8_000_000))
    try:
        c.set_params({"exposure_ns": int(20 * 1e6)}); time.sleep(a.settle)
        f, _ = c.capture()
        cv, tile, mean = uniformity(f)
        print("flat-field @20ms: Gr ROI mean=%.0f  CV=%.4f  9-tile spread=%.1f%%" % (mean, cv, 100 * tile))
        if cv > 0.06 or tile > 0.15:
            print("  ** NOT uniform ** (want CV<~0.05, spread<~15%%). Fill the FOV with the lit "
                  "diffuse target / defocus slightly. Use --force to run anyway.")
            if a.check or not a.force:
                return
        else:
            print("  looks like a usable flat field.")
        if a.check:
            return

        exps = np.geomspace(a.emin, a.emax, a.points)
        print("\nexposure sweep (%d pts, %.2f-%.1f ms):" % (a.points, a.emin, a.emax))
        rows, maxv = run_sweep(c, exps, a.frames, a.roi, a.settle)
    finally:
        c.set_params({"exposure_ns": orig})
        c.close()
        if d:
            d.close()

    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("\nwrote %s (%d rows)" % (a.out, len(rows)))
    lux = lux_from_lut(a.illum, a.level) if a.illum != "keep" else None
    analyze(rows, maxv, lux, illum=a.illum, fnum=a.fnum, reflectance=a.reflectance)


SNR1S_EXP_S = 1.0 / 60.0        # Sony SNR1s reference exposure: 1/60 s
SNR1S_FNUM = 1.4                # Sony reference lens
SNR1S_REFL = 0.18              # Sony reference target (18% grey)


def snr1s_normalize(s1s, fnum, reflectance):
    """Scale a measured SNR1s (lux) to Sony's F1.4 / 18% grey reference. SNR1s is
    the illuminance for a fixed sensor signal, and signal ~ reflectance / F^2, so
      SNR1s_std = SNR1s_meas * (1.4/F)^2 * (reflectance/0.18).
    Lets a 97% white standard stand in for a (future) 18% grey card."""
    return s1s * (SNR1S_FNUM / fnum) ** 2 * (reflectance / SNR1S_REFL)


def compute_metrics(rows, maxv, lux=None):
    """Shared numeric core: per-Bayer-channel OETF + PTC + SNR1s from sweep rows.
    Returns {chn: {black, K, readN, fullwell, oetf_r2, oetf_slope, S1, snr1s}};
    snr1s is the raw (as-measured) SNR1s in lux at 1/60 s, nan if no lux. See
    analyze() for the physics."""
    out = {}
    have_lux = lux is not None
    for chn in ("R", "Gr", "Gb", "B"):
        m = np.array([r["%s_mean" % chn] for r in rows])
        v = np.array([r["%s_var" % chn] for r in rows])
        black = float(m.min())
        sig = m - black
        sat = 0.9 * (maxv - black)
        lin = (sig > 0.05 * sat) & (sig < 0.85 * sat)         # shot-noise region (avoid readnoise + rolloff)
        slope = readvar = K = readN = fullwell = float("nan")
        if lin.sum() >= 3:
            slope, inter = np.polyfit(sig[lin], v[lin], 1)     # var = slope*sig + inter
            K = 1.0 / slope if slope > 0 else float("nan")     # e-/DN
            readvar = max(inter, float(v[m <= black + 2].mean()) if (m <= black + 2).any() else inter)
            readN = (readvar ** 0.5) * K if K == K else float("nan")
            fullwell = (maxv - black) * K if K == K else float("nan")
        # SNR=1 signal (DN above black): S^2 = slope*S + readvar
        S1 = float("nan")
        if slope == slope and slope > 0 and readvar == readvar and readvar > 0:
            S1 = (slope + (slope * slope + 4 * readvar) ** 0.5) / 2.0
        # OETF linearity R^2 + slope (DN/ms) over the non-saturated range
        ok = sig < 0.9 * sat
        oetf_slope = oetf_r2 = float("nan")
        if ok.sum() >= 3:
            exps = np.array([r["exp_ms"] for r in rows])[ok]
            p = np.polyfit(exps, m[ok], 1); oetf_slope = float(p[0]); fit = np.polyval(p, exps)
            oetf_r2 = 1 - np.sum((m[ok] - fit) ** 2) / max(np.sum((m[ok] - m[ok].mean()) ** 2), 1e-9)
        # exposure at SNR=1 -> illuminance*time H1 = lux*t1 -> SNR1s = H1 / (1/60 s)
        s1s = float("nan")
        if S1 == S1 and oetf_slope == oetf_slope and oetf_slope > 0 and have_lux:
            s1s = (lux * (S1 / oetf_slope) / 1000.0) / SNR1S_EXP_S   # lux at 1/60 s
        out[chn] = dict(black=black, K=K, readN=readN, fullwell=fullwell,
                        oetf_r2=oetf_r2, oetf_slope=oetf_slope, S1=S1, snr1s=s1s)
    return out


def analyze(rows, maxv, lux=None, illum=None, fnum=None, reflectance=None):
    """Print per-channel OETF/PTC + SNR1s (Sony low-light FoM): TARGET illuminance
    (lux) for SNR=1 at 1/60 s, lower is better.

    SNR is K-independent: SNR = signal / sqrt(var), and the PTC gives
    var = slope*signal + readvar (slope = 1/K). SNR = 1 => signal^2 = var, i.e.
      S1 = (slope + sqrt(slope^2 + 4*readvar)) / 2      [DN above black]
    mapped through the OETF slope to an exposure then via the DMX->lux LUT to a
    lux*s product, /(1/60 s). Normalizes to Sony's F1.4 / 18% grey with
    --fnum/--reflectance (SNR1s ~ F^2/reflectance). Spectrum is NOT correctable
    -- match Sony's 3200K with --illum tungsten."""
    have_lux = lux is not None
    met = compute_metrics(rows, maxv, lux)
    print("\n=== OETF / PTC summary (per Bayer channel) ===")
    hdr = "  %-3s %8s %8s %10s %9s %10s %8s" % (
        "ch", "black", "K(e-/DN)", "readN(e-)", "fullwell", "OETF R^2", "S@SNR1")
    if have_lux:
        hdr += " %11s" % "SNR1s(lx)"
    print(hdr)
    for chn in ("R", "Gr", "Gb", "B"):
        x = met[chn]
        line = "  %-3s %8.1f %8.3f %10.1f %9.0f %10.5f %8.2f" % (
            chn, x["black"], x["K"], x["readN"], x["fullwell"], x["oetf_r2"], x["S1"])
        if have_lux:
            line += " %11.3f" % x["snr1s"]
        print(line)
    g = met["Gr"]["snr1s"]
    if have_lux and g == g:
        print("\n  SNR1s (green) = %.3f lux  @ 1/60s, %s%s, this gain" % (
            g, (illum or "?"), (" 3200K" if illum == "tungsten" else "")))
        if fnum and reflectance:
            std = snr1s_normalize(g, fnum, reflectance)
            print("  normalized to Sony conditions (F%.1f, %.0f%% grey): SNR1s = %.3f lux"
                  " [from F%.2f, %.0f%% refl]" % (SNR1S_FNUM, 100 * SNR1S_REFL, std, fnum, 100 * reflectance))
            if illum != "tungsten":
                print("  ** illuminant is %s, not 3200K -- spectrum mismatch NOT corrected; re-run --illum tungsten" % illum)
        else:
            print("  give --fnum and --reflectance to normalize to Sony's F1.4 / 18%% grey."
                  " Match 3200K with --illum tungsten.")


if __name__ == "__main__":
    main()
