"""Gain-swept SNR1s -- the low-light figure of merit for SENSOR DOWN-SELECT.

For each analog gain, auto-place the exposure sweep so the saturation knee sits
near the top, run the robust PTC (oetf_ptc.run_sweep), extract the green-channel
SNR1s (oetf_ptc.compute_metrics), and normalize to Sony's F1.4 / 18% grey. The
MINIMUM SNR1s over gain is the comparison figure (each sensor at its own best
gain); we report it and the gain it occurs at.

SNR1s is read-noise-limited, and input-referred read noise (e-) falls as analog
gain rises (toward the HCG floor), so SNR1s improves with gain until it plateaus.

Sony reference conditions: 3200K, 18% grey, F1.4, 1/60s -> use --illum tungsten.
The rig knobs that make different sensors comparable are --fnum (lens) and
--reflectance (target); a 97% white standard normalizes to 18% grey via the
reflectance term. Results (+ all conditions) are saved to JSON/CSV so IMX678 and
the IQ9 candidates compare apples-to-apples.

  python lab/snr1s_vs_gain.py --illum tungsten --level 48 --fnum 1.65 --reflectance 0.97 \
      --gains 1,2,4,8,16,32 --sensor IMX678 --force
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
sys.path.insert(0, _HERE)                                                # oetf_ptc, dmx_lights
import camera_client
import dmx_lights as dl
import oetf_ptc as o

JET_HOST, JET_PORT = "192.168.99.2", 9000


def wait_for_gain(c, g, tries=16, dt=0.4):
    """Set-then-confirm: poll actual_gain (from capture metadata, which lags the
    request by a few frames) until it tracks g, or plateaus (clamp). Returns the
    last actual gain seen. IMX678 analog gain tops out ~31.6x (30 dB)."""
    ag = 0.0
    for _ in range(tries):
        time.sleep(dt)
        ag = float(c.info().get("actual_gain") or 0.0)
        if ag and abs(ag - g) <= 0.06 * g:
            return ag
    return ag or float(g)                              # plateaued/clamped, or metadata absent


def green_at(c, e_ms, frames=4, settle=0.8, flush=3):
    """Median green-plane ROI mean at one exposure (for range planning). Flushes
    in-flight frames first -- range planning makes big exposure jumps, so stale
    frames would give a bogus mean (e.g. black~2172 at 0.05ms)."""
    c.set_params({"exposure_ns": int(e_ms * 1e6)}); time.sleep(settle)
    fs, maxv = o.capture_frames(c, frames, flush=flush)
    mean, _ = o.channel_stats(fs, "Gr", 0.25)
    return mean, maxv


def plan_range(c, black_g, maxv, sat_target, emax_cap, emin_floor):
    """Probe responsivity R [DN/ms] at the current gain and place [emin, emax] so
    the knee lands near sat_target*full-scale. Probe from long to short exposure,
    taking the first non-saturated point (works across the whole gain range)."""
    R = None
    for tp in (8.0, 2.0, 0.5, 0.2):
        mean, _ = green_at(c, tp)
        if mean < 0.7 * maxv:
            R = max((mean - black_g) / tp, 1e-6); break
    if R is None:                                     # even 0.2ms saturates -> very high gain
        R = max((mean - black_g) / 0.2, 1e-6)
    emax = min(emax_cap, sat_target * (maxv - black_g) / R)
    emin = max(emin_floor, emax / 150.0)
    return emin, emax, R


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--illum", choices=["d65", "tungsten", "keep"], default="tungsten")
    ap.add_argument("--level", type=int, default=48, help="DMX level for the flat field")
    ap.add_argument("--gains", default="1,2,4,8,16,32", help="comma-separated analog gains (x)")
    ap.add_argument("--fnum", type=float, default=1.65, help="lens F-number (SNR1s F1.4 normalization)")
    ap.add_argument("--reflectance", type=float, default=0.97, help="target reflectance 0-1 (->18%% grey)")
    ap.add_argument("--sensor", default="IMX678", help="sensor id for the saved comparison record")
    ap.add_argument("--points", type=int, default=20)
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--roi", type=float, default=0.25)
    ap.add_argument("--settle", type=float, default=1.2, help="seconds after an exposure change")
    ap.add_argument("--sat-target", type=float, default=0.95, dest="sat_target")
    ap.add_argument("--emax-cap", type=float, default=120.0, dest="emax_cap")
    ap.add_argument("--emin-floor", type=float, default=0.05, dest="emin_floor")
    ap.add_argument("--out", default=None, help="JSON path (default lab/snr1s_vs_gain_<illum>.json)")
    ap.add_argument("--force", action="store_true", help="run even if the field looks non-uniform")
    a = ap.parse_args()
    gains = [float(g) for g in a.gains.split(",") if g.strip()]
    out = a.out or os.path.join(_HERE, "snr1s_vs_gain_%s.json" % a.illum)
    lux = o.lux_from_lut(a.illum, a.level) if a.illum != "keep" else None
    if lux is None:
        print("no DMX->lux LUT for %s -- SNR1s needs a calibrated light level. Run dmx_lux_cal.py first." % a.illum)
        return

    # DMX flat field
    d = None
    if a.illum != "keep":
        d = dl.OpenDMX().start(rate_hz=11)
        ch = dl.CH_D65 if a.illum == "d65" else dl.CH_TUNGSTEN
        other = dl.CH_TUNGSTEN if a.illum == "d65" else dl.CH_D65
        d.set_many({ch: a.level, other: 0})
        print("DMX: %s(ch%d)=%d (%.0f lux at target), other off; @11Hz" % (a.illum, ch, a.level, lux))
        time.sleep(1.5)

    c = camera_client.CameraClient(JET_HOST, JET_PORT, timeout=30)
    c.connect()
    orig_exp = int(c.info().get("exposure_ns", 8_000_000))
    orig_gain = float(c.info().get("gain", 1.0)) or 1.0
    results = []
    try:
        # one-shot flat-field gate (central ROI); vignetting trips tile spread -> --force
        c.set_params({"gain": 1.0, "exposure_ns": int(20e6)}); time.sleep(a.settle)
        f, _ = c.capture()
        cv, tile, mean = o.uniformity(f)
        print("flat-field @20ms/g1: Gr ROI mean=%.0f CV=%.4f 9-tile=%.1f%%" % (mean, cv, 100 * tile))
        if (cv > 0.06 or tile > 0.15) and not a.force:
            print("  ** non-uniform (central CV<0.05 ok; tile spread is lens vignetting). Use --force."); return

        last_ag = None
        for g in gains:
            c.set_params({"gain": float(g)})
            ag = wait_for_gain(c, g)                    # confirm the sensor actually took it
            o.capture_frames(c, 1, flush=3)             # drop frames still at the old gain
            # genuine clamp: actual well below request AND unchanged from the previous point
            if last_ag is not None and ag < 0.9 * g and abs(ag - last_ag) < 0.03 * max(ag, 1.0):
                print("gain request %.1fx clamped at %.2fx (analog ceiling) -- stopping sweep." % (g, ag)); break
            last_ag = ag
            black_g, maxv = green_at(c, a.emin_floor)
            emin, emax, R = plan_range(c, black_g, maxv, a.sat_target, a.emax_cap, a.emin_floor)
            exps = np.geomspace(emin, emax, a.points)
            print("\ngain %.2fx: black~%.0f  R~%.1f DN/ms  sweep %.3f-%.2f ms x%d"
                  % (ag, black_g, R, emin, emax, a.points))
            rows, maxv = o.run_sweep(c, exps, a.frames, a.roi, a.settle, verbose=False)
            gm = o.compute_metrics(rows, maxv, lux)["Gr"]
            raw = gm["snr1s"]
            std = o.snr1s_normalize(raw, a.fnum, a.reflectance) if raw == raw else float("nan")
            results.append(dict(gain=round(ag, 3), snr1s_raw=raw, snr1s_std=std,
                                readN_e=gm["readN"], K=gm["K"], fullwell_e=gm["fullwell"],
                                oetf_r2=gm["oetf_r2"], emin_ms=round(emin, 4), emax_ms=round(emax, 3)))
            print("   readN %.2f e-  K %.4f e-/DN  fullwell %.0f e-  OETF R2 %.5f"
                  % (gm["readN"], gm["K"], gm["fullwell"], gm["oetf_r2"]))
            print("   SNR1s raw %.3f lux -> normalized %.3f lux (F%.2f,%.0f%%->F1.4,18%%)"
                  % (raw, std, a.fnum, 100 * a.reflectance))
    finally:
        c.set_params({"gain": orig_gain, "exposure_ns": orig_exp})
        c.close()
        if d:
            d.close()

    if not results:
        print("no results."); return
    valid = [r for r in results if r["snr1s_std"] == r["snr1s_std"]]
    best = min(valid, key=lambda r: r["snr1s_std"]) if valid else None

    print("\n=== SNR1s vs gain (%s, %s%s, F%.2f, %.0f%% refl -> Sony F1.4/18%% grey) ==="
          % (a.sensor, a.illum, " 3200K" if a.illum == "tungsten" else "", a.fnum, 100 * a.reflectance))
    print("  %6s %10s %10s %9s %9s" % ("gain", "readN(e-)", "SNR1s_raw", "SNR1s_std", "fullwell"))
    for r in results:
        print("  %5.1fx %10.2f %10.3f %9.3f %9.0f"
              % (r["gain"], r["readN_e"], r["snr1s_raw"], r["snr1s_std"], r["fullwell_e"]))
    if best:
        print("\n  >>> SNR1s (down-select FoM) = %.3f lux  @ gain %.1fx  <<<" % (best["snr1s_std"], best["gain"]))
        print("      (green, Sony-normalized F1.4/18%% grey, 3200K, 1/60s; lower is better)")

    record = dict(sensor=a.sensor, metric="SNR1s", units="lux",
                  conditions=dict(illuminant=a.illum, source_K=3200 if a.illum == "tungsten" else None,
                                  fnum=a.fnum, reflectance=a.reflectance, exposure_s=o.SNR1S_EXP_S,
                                  target_lux=lux, normalized_to="Sony F1.4 / 18% grey"),
                  best_snr1s_lux=(best["snr1s_std"] if best else None),
                  best_gain=(best["gain"] if best else None),
                  sweep=results)
    with open(out, "w") as fh:
        json.dump(record, fh, indent=1)
    csvp = os.path.splitext(out)[0] + ".csv"
    with open(csvp, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(results[0].keys())); w.writeheader(); w.writerows(results)
    print("\nsaved %s + %s" % (out, csvp))


if __name__ == "__main__":
    main()
