"""Interactive DMX -> lux calibration for a Waveform-3082 panel.

Steps one channel through DMX levels, HOLDING each steady (11 Hz stream) while you
read a manual lux meter at the target surface and type the value. Saves a LUT
(dmx[], lux[]) + CSV, and reports how linear the panel is. The LUT lets us set the
panel to a known lux (calibrated source) and do a light-axis OETF cross-check.

Close QLC+ first and don't use the webui/GUI DMX during the run (COM5 is exclusive,
and this tool holds it for the whole session).

  python lab/dmx_lux_cal.py --channel d65
  python lab/dmx_lux_cal.py --channel tungsten --steps 0,16,32,64,96,128,160,192,224,255
"""
import argparse
import csv
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dmx_lights as dl

_HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", choices=["d65", "tungsten"], default="d65")
    ap.add_argument("--steps", default="0,16,32,48,64,96,128,160,192,224,255",
                    help="comma-separated DMX levels 0-255")
    ap.add_argument("--settle", type=float, default=0.8, help="seconds after setting a level")
    ap.add_argument("--out", default=None, help="LUT json path (default lab/dmx_lux_<channel>.json)")
    a = ap.parse_args()
    steps = [max(0, min(255, int(s))) for s in a.steps.split(",") if s.strip() != ""]
    out = a.out or os.path.join(_HERE, "dmx_lux_%s.json" % a.channel)
    ch = dl.CH_D65 if a.channel == "d65" else dl.CH_TUNGSTEN
    other = dl.CH_TUNGSTEN if a.channel == "d65" else dl.CH_D65

    print("DMX->lux calibration: %s (ch%d). Hold the meter at the target surface." % (a.channel, ch))
    print("At each prompt, read the lux and type it (or blank to skip, 'q' to finish early).\n")
    d = dl.OpenDMX().start(rate_hz=11)
    d.set(other, 0)
    pairs = []
    try:
        for lvl in steps:
            d.set(ch, lvl)
            time.sleep(a.settle)
            while True:
                s = input("  DMX %3d -> lux: " % lvl).strip().lower()
                if s == "q":
                    lvl = None
                    break
                if s == "":
                    break
                try:
                    pairs.append((lvl, float(s)))
                    break
                except ValueError:
                    print("    enter a number, blank to skip, or q to finish")
            if lvl is None:
                break
    except KeyboardInterrupt:
        print("\n(interrupted)")
    finally:
        d.set_many({ch: 0, other: 0})
        time.sleep(0.5)
        d.close()

    if len(pairs) < 2:
        print("need >=2 readings; nothing saved."); return
    pairs.sort()
    dmx = [p[0] for p in pairs]
    lux = [p[1] for p in pairs]
    # linearity of a simple line fit (how close is lux ~ a*dmx + b)
    n = len(dmx)
    sx = sum(dmx); sy = sum(lux); sxx = sum(x * x for x in dmx); sxy = sum(x * y for x, y in pairs)
    den = n * sxx - sx * sx
    slope = (n * sxy - sx * sy) / den if den else 0.0
    inter = (sy - slope * sx) / n
    ybar = sy / n
    ss_res = sum((y - (slope * x + inter)) ** 2 for x, y in pairs)
    ss_tot = sum((y - ybar) ** 2 for y in lux) or 1e-9
    r2 = 1 - ss_res / ss_tot

    with open(out, "w") as f:
        json.dump({"channel": a.channel, "dmx": dmx, "lux": lux,
                   "line_fit": {"slope_lux_per_dmx": slope, "intercept_lux": inter, "r2": r2}}, f, indent=1)
    csvp = os.path.splitext(out)[0] + ".csv"
    with open(csvp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["dmx", "lux"]); w.writerows(pairs)
    print("\n%d points  lux %.1f..%.1f  line R^2=%.4f (1.0 = perfectly linear)"
          % (n, min(lux), max(lux), r2))
    print("saved %s + %s" % (out, csvp))


if __name__ == "__main__":
    main()
