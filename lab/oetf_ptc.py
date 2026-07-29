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

Runs on the PC: DMX is local (lab/dmx_lights), raw frames come from the Jetson
image_server over the wire (camera_client). Close QLC+ first (FTDI is exclusive).

Usage:
  python lab/oetf_ptc.py --check                 # just report flat-field uniformity
  python lab/oetf_ptc.py --illum d65 --level 128 --emin 0.3 --emax 120 --points 22 --frames 2
"""
import argparse
import csv
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


def capture_pair(c, delay=0.12):
    f1, maxv = c.capture()
    time.sleep(delay)                                  # ensure an independent frame
    f2, _ = c.capture()
    return f1.astype(np.float64), f2.astype(np.float64), maxv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="report flat-field uniformity and exit")
    ap.add_argument("--illum", choices=["d65", "tungsten", "keep"], default="d65")
    ap.add_argument("--level", type=int, default=128, help="DMX level 0-255 for the chosen panel")
    ap.add_argument("--emin", type=float, default=0.3, help="min exposure ms")
    ap.add_argument("--emax", type=float, default=120.0, help="max exposure ms")
    ap.add_argument("--points", type=int, default=22)
    ap.add_argument("--frames", type=int, default=2, help="frames per exposure (>=2; pairs averaged)")
    ap.add_argument("--roi", type=float, default=0.25, help="central ROI fraction per plane")
    ap.add_argument("--settle", type=float, default=1.3, help="seconds after an exposure change")
    ap.add_argument("--out", default=os.path.join(_HERE, "ptc_result.csv"))
    ap.add_argument("--force", action="store_true", help="run even if the field looks non-uniform")
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
        rows = []
        print("\nexposure sweep (%d pts, %.2f-%.1f ms):" % (a.points, a.emin, a.emax))
        for e_ms in exps:
            c.set_params({"exposure_ns": int(e_ms * 1e6)}); time.sleep(a.settle)
            f1, f2, maxv = capture_pair(c)
            rec = {"exp_ms": round(float(e_ms), 4)}
            for chn in ("R", "Gr", "Gb", "B"):
                p1, p2 = roi_of(plane(f1, chn), a.roi), roi_of(plane(f2, chn), a.roi)
                mean = float(p1.mean())
                var = float(((p1 - p2) ** 2).mean() / 2.0)      # temporal var, FPN removed
                rec["%s_mean" % chn] = round(mean, 3)
                rec["%s_var" % chn] = round(var, 4)
            rows.append(rec)
            print("  %.2f ms  Gr mean=%.1f var=%.1f%s" % (e_ms, rec["Gr_mean"], rec["Gr_var"],
                  "  (SAT)" if rec["Gr_mean"] >= 0.98 * maxv else ""))
    finally:
        c.set_params({"exposure_ns": orig})
        c.close()
        if d:
            d.close()

    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("\nwrote %s (%d rows)" % (a.out, len(rows)))
    analyze(rows, maxv)


def analyze(rows, maxv):
    """Per-channel OETF linearity + PTC (gain, read noise, full well)."""
    print("\n=== OETF / PTC summary (per Bayer channel) ===")
    print("  %-3s %8s %8s %10s %9s %10s" % ("ch", "black", "K(e-/DN)", "readN(e-)", "fullwell", "OETF R^2"))
    for chn in ("R", "Gr", "Gb", "B"):
        m = np.array([r["%s_mean" % chn] for r in rows])
        v = np.array([r["%s_var" % chn] for r in rows])
        black = float(m.min())
        sig = m - black
        sat = 0.9 * (maxv - black)
        lin = (sig > 0.05 * sat) & (sig < 0.85 * sat)         # shot-noise region (avoid readnoise + rolloff)
        if lin.sum() >= 3:
            slope, inter = np.polyfit(sig[lin], v[lin], 1)     # var = slope*sig + inter
            K = 1.0 / slope if slope > 0 else float("nan")     # e-/DN
            readvar = max(inter, float(v[m <= black + 2].mean()) if (m <= black + 2).any() else inter)
            readN = (readvar ** 0.5) * K if K == K else float("nan")
            fullwell = (maxv - black) * K if K == K else float("nan")
        else:
            K = readN = fullwell = float("nan")
        # OETF linearity R^2 over the non-saturated range
        ok = sig < 0.9 * sat
        if ok.sum() >= 3:
            exps = np.array([r["exp_ms"] for r in rows])[ok]
            p = np.polyfit(exps, m[ok], 1); fit = np.polyval(p, exps)
            ss = 1 - np.sum((m[ok] - fit) ** 2) / max(np.sum((m[ok] - m[ok].mean()) ** 2), 1e-9)
        else:
            ss = float("nan")
        print("  %-3s %8.1f %8.3f %10.1f %9.0f %10.5f" % (chn, black, K, readN, fullwell, ss))


if __name__ == "__main__":
    main()
