#!/usr/bin/env python3
"""Capture a RAW light-sweep on the IQ9 and run PTC/OETF/SNR characterization (headless CLI).

Uses ONE persistent bayer stream for the whole sweep and grabs CONSECUTIVE frames per level
(the two-frame difference is then the true temporal noise). This replaces the earlier
open/close-per-frame path (grab_raw16 spawns a fresh gst-launch each call -> the frame lands
before sensor/3A steady-state, so consecutive grabs sit at slightly different levels -> ~6-7%
capture-to-capture spread that CORRUPTED the variance and K). The persistent stream is rock
stable (frame-mean spread ~0.0%), so K/read come out clean. See the webui /ptc page + the
/api/ptc/* endpoints for the interactive version; this CLI mirrors them for scripting.

  # dark first (CAP THE LENS), then the sweep (uncap, light on):
  python3 ptc_capture.py --capture-dark --nframes 8
  python3 ptc_capture.py --channel tungsten --levels 12,18,25,32,40,48,55,62 --nframes 3 \
      --lux 12:53,18:160,25:370,32:700     # optional lux per level -> responsivity + SNR1s(lux)

Sensor exposure/gain is STATIC in the .bin, so the LIGHT (DMX) is the only sweep lever. Needs a
FLAT field filling the ROI. Camera is single-client; this owns the camera for its run.
"""
import argparse
import glob
import json
import math
import os
import sys
import time
import urllib.request

for _p in ("/home/metro/iq9_server", "/home/metro/iq9_server/_shared"):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import numpy as np                # noqa: E402
import camera_qmmf               # noqa: E402
import raw_ptc                   # noqa: E402

DMX = os.environ.get("DMX_AGENT", "http://192.168.99.1:9200") + "/dmx"
OUT = "/var/volatile/ptc"


def set_dmx(**fwd):
    body = json.dumps({k: int(v) for k, v in fwd.items()}).encode()
    urllib.request.urlopen(urllib.request.Request(
        DMX, data=body, method="POST", headers={"Content-Type": "application/json"}), timeout=8).read()


def roi_even(H, W, frac):
    fh = int(H * frac); fw = int(W * frac)
    y0 = (H - fh) // 2; x0 = (W - fw) // 2
    y0 -= y0 % 2; x0 -= x0 % 2                     # even offsets keep RGGB phase
    return (y0, y0 + fh, x0, x0 + fw)


def consecutive(cap, n):
    frames = []
    for _ in range(max(2, int(n))):
        f = cap.frame(timeout_s=5.0)
        if f is not None:
            frames.append(f.copy())
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", choices=["d65", "tungsten"], default="tungsten")
    ap.add_argument("--levels", default="12,18,25,32,40,48,55,62")
    ap.add_argument("--nframes", type=int, default=3)
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--roi-frac", type=float, default=0.4)
    ap.add_argument("--lux", default="", help="comma list level:lux, e.g. 12:53,18:160 (optional)")
    ap.add_argument("--capture-dark", action="store_true", help="just grab a capped dark and save it")
    ap.add_argument("--out", default="/var/volatile/ptc_report.json")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    lux = {}
    for tok in args.lux.replace(" ", "").split(","):
        if ":" in tok:
            k, v = tok.split(":"); lux[str(int(k))] = float(v)

    cap = camera_qmmf.QmmfCapture(mode="bayer", width=3856, height=2180, fps=30).start()
    try:
        for _ in range(3):
            cap.frame(timeout_s=5.0)                       # warm up
        probe = cap.frame(timeout_s=5.0)
        H, W = probe.shape
        roi = roi_even(H, W, args.roi_frac)
        print("frame %dx%d  ROI(y0,y1,x0,x1)=%s" % (W, H, roi))

        if args.capture_dark:
            frames = consecutive(cap, args.nframes)
            for i, f in enumerate(frames):
                np.save(os.path.join(OUT, "dark_%02d.npy" % i), f)
            dk = raw_ptc.measure(frames, roi)
            print("dark saved (%d frames): R pedestal=%.1f read=%.2f DN" %
                  (len(frames), dk["R"]["mean"], math.sqrt(max(dk["R"]["var"], 0))))
            return 0

        # prefer a pre-captured CAPPED dark (true read noise)
        dfiles = sorted(glob.glob(os.path.join(OUT, "dark_*.npy")))
        dark = None
        if dfiles:
            dark = raw_ptc.measure([np.load(p) for p in dfiles], roi)
            print("dark: capped, %d frames, R pedestal=%.1f read=%.2f DN" %
                  (len(dfiles), dark["R"]["mean"], math.sqrt(max(dark["R"]["var"], 0))))
        else:
            print("no capped dark found -> using the darkest sweep level for read noise")

        levels = [int(x) for x in args.levels.replace(",", " ").split() if int(x) > 0]
        points = []
        for lvl in levels:
            set_dmx(**{args.channel: lvl})
            time.sleep(args.settle)
            frames = consecutive(cap, args.nframes)
            meas = raw_ptc.measure(frames, roi)
            sub = frames[0][roi[0]:roi[1], roi[2]:roi[3]]
            gp = sub[0::2, 1::2].astype(np.float64)
            light = lux.get(str(lvl), float(lvl))
            print("  %s=%3d  green mean=%.0f DN  nonunif=%.1f%%  G1.var=%.1f  clip=%.4f"
                  % (args.channel, lvl, gp.mean(), 100.0 * gp.std() / max(gp.mean(), 1e-6),
                     meas["G1"]["var"], float((sub >= 4095).mean())))
            points.append({"light": light, "meas": meas})
    finally:
        try:
            cap.stop()
        except Exception:
            pass

    rep = raw_ptc.characterize(points, dark)
    if lux:
        for ch in raw_ptc.BAYER:
            rep["channels"][ch].update(raw_ptc.lux_metrics(rep["channels"][ch]))
    open(args.out, "w").write(json.dumps({"roi": roi, "channel": args.channel,
                                          "levels": levels, "lux": lux, "report": rep}, indent=2))
    print("\n=== PTC/OETF/SNR (per channel) ===")
    for ch in raw_ptc.BAYER:
        c = rep["channels"][ch]
        extra = ("  resp=%.2f e-/lux  SNR1s=%.2f lux" % (c["responsivity_e_per_lux"], c["snr1s_lux"])
                 if "snr1s_lux" in c else "")
        print("  %-2s K=%.3f e-/DN  read=%.2f e-  full_well=%.0f e-  maxSNR=%.1f  SNR1=%.2f e-  "
              "DR=%.1f dB  OETF r2=%.5f%s"
              % (ch, c["K_e_per_DN"], c["read_e"], c["full_well_e"], c["max_SNR"],
                 c["SNR1_e_sensor"], c["DR_dB"], c["oetf"]["r2"], extra))
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
