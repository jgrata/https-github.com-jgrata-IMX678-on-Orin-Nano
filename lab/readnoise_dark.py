"""DARK read-noise vs analog gain (lens capped, lights off).

The true read noise for SNR1s -- the light-based PTC intercept is contaminated
by LED/source flicker (excess temporal noise under illumination), which inflates
it several-fold. This measures the temporal noise floor in the dark:
  readN_DN = sqrt( median_k var(f_k - f_{k+1})/2 )   over a central ROI
  readN_e  = readN_DN * K(gain)                        (K from the gain sweep)

CAP THE LENS before running. Feed the resulting readN into the SNR1s calc for
the true (flicker-free) low-light figure. Reusable across sensors for the
down-select (same method -> comparable read noise).

  python lab/readnoise_dark.py --gains 1,2,4,8,16,32 --k-from lab/snr1s_vs_gain_tungsten.json
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metro_server", "webui"))
sys.path.insert(0, _HERE)
import camera_client
import dmx_lights as dl
import oetf_ptc as o

JET_HOST, JET_PORT = "192.168.99.2", 9000


def wait_for_gain(c, g, tries=14, dt=0.4):
    ag = 0.0
    for _ in range(tries):
        time.sleep(dt)
        ag = float(c.info().get("actual_gain") or 0.0)
        if ag and abs(ag - g) <= 0.06 * g:
            return ag
    return ag or float(g)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gains", default="1,2,4,8,16,32")
    ap.add_argument("--exp-ms", type=float, default=2.0, dest="exp_ms")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--roi", type=float, default=0.25)
    ap.add_argument("--sensor", default="IMX678")
    ap.add_argument("--k-from", default=None, dest="k_from",
                    help="gain-sweep JSON to read K(e-/DN) per gain (else DN only)")
    ap.add_argument("--out", default=os.path.join(_HERE, "readnoise_dark.json"))
    ap.add_argument("--no-dmx", action="store_true", help="skip turning the DMX off")
    a = ap.parse_args()
    gains = [float(g) for g in a.gains.split(",") if g.strip()]

    kmap = {}
    if a.k_from and os.path.exists(a.k_from):
        rec = json.load(open(a.k_from))
        kmap = {round(s["gain"], 1): s["K"] for s in rec.get("sweep", []) if s.get("K") == s.get("K")}

    if not a.no_dmx:                                   # lights off; lens must ALSO be capped
        d = dl.OpenDMX().start(rate_hz=11)
        d.set_many({dl.CH_D65: 0, dl.CH_TUNGSTEN: 0}); time.sleep(1.0); d.close()

    c = camera_client.CameraClient(JET_HOST, JET_PORT, timeout=25)
    c.connect()
    oe = int(c.info().get("exposure_ns", 8_000_000)); og = float(c.info().get("gain", 1.0)) or 1.0
    rows = []
    print("DARK read noise (CAP THE LENS), exp=%.1fms, central ROI, green plane" % a.exp_ms)
    print("  %6s %8s %10s %9s %10s" % ("gain", "black", "readN_DN", "K", "readN_e"))
    try:
        for g in gains:
            c.set_params({"gain": float(g), "exposure_ns": int(a.exp_ms * 1e6)})
            ag = wait_for_gain(c, g)
            o.capture_frames(c, 1, flush=3)            # clear frames still at the old gain
            fs, _ = o.capture_frames(c, a.frames, flush=2)
            black, rv = o.channel_stats(fs, "Gr", a.roi)
            rn_dn = rv ** 0.5
            K = kmap.get(round(ag, 1), float("nan"))
            rn_e = rn_dn * K if K == K else float("nan")
            rows.append(dict(gain=round(ag, 3), black=round(black, 2),
                             readN_dn=round(rn_dn, 4), K=K, readN_e=(round(rn_e, 3) if rn_e == rn_e else None)))
            print("  %5.1fx %8.1f %10.3f %9s %10s"
                  % (ag, black, rn_dn, ("%.4f" % K if K == K else "n/a"),
                     ("%.2f" % rn_e if rn_e == rn_e else "n/a")))
    finally:
        c.set_params({"gain": og, "exposure_ns": oe}); c.close()

    rec = dict(sensor=a.sensor, metric="dark_read_noise", exp_ms=a.exp_ms, roi=a.roi, sweep=rows)
    with open(a.out, "w") as fh:
        json.dump(rec, fh, indent=1)
    csvp = os.path.splitext(a.out)[0] + ".csv"
    with open(csvp, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("\nsaved %s + %s" % (a.out, csvp))
    if any(r["readN_e"] for r in rows):
        mn = min((r for r in rows if r["readN_e"]), key=lambda r: r["readN_e"])
        print("floor: %.2f e- @ gain %.1fx" % (mn["readN_e"], mn["gain"]))


if __name__ == "__main__":
    main()
