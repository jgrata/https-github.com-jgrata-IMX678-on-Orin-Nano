#!/usr/bin/env python3
"""Self-test for dcg_characterize: synthetic photon-transfer frames with known gains.

Generates stacked [OB][HG][LG] Clear HDR frames where both legs see the same photons but read
with different conversion gains (K_HG=0.5, K_LG=1.2 e-/DN -> Rcg=2.4), plus Gaussian read noise
(2 e-) and a pedestal. Checks that characterize() recovers K per leg, read noise, and Rcg two ways.
Run: python test_dcg_characterize.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dcg_characterize as c

K_HG, K_LG = 0.5, 1.2      # e-/DN (HCG has higher conversion gain -> smaller K)
READ_E, PED = 2.0, 200.0
RCG = K_LG / K_HG          # 2.4
W, OB, HG, LG = 256, 8, 200, 200  # large enough that the 2-frame variance estimate is tight
LAYOUT = dict(hg_rows=HG, lg_rows=LG, ob_top=OB, gap=0, hg_first=True)


def stacked(s_e, rng):
    """One stacked DCG frame at signal s_e electrons (same photons both legs)."""
    f = np.full((OB + HG + LG, W), PED, float)

    def block(k, shape):
        e = rng.poisson(s_e, shape).astype(float) + rng.normal(0, READ_E, shape)
        return e / k + PED

    f[OB:OB + HG] = block(K_HG, (HG, W))
    f[OB + HG:OB + HG + LG] = block(K_LG, (LG, W))
    return np.clip(f, 0, 65535).astype("<u2")


def main():
    rng = np.random.default_rng(1)
    levels = [200, 800, 2000, 5000, 12000, 25000]
    points = [{"level": L, "meas": c.measure([stacked(L, rng) for _ in range(2)], LAYOUT)}
              for L in levels]
    dark = c.measure([stacked(0, rng) for _ in range(2)], LAYOUT)

    rep = c.characterize(points, dark, ref_level=5000)
    hg = rep["legs"]["HG"]["channels"]
    lg = rep["legs"]["LG"]["channels"]

    for ch in c.dcg_demux.BAYER:
        assert abs(hg[ch]["K_e_per_DN"] - K_HG) < 0.05, (ch, "K_HG", hg[ch]["K_e_per_DN"])
        assert abs(lg[ch]["K_e_per_DN"] - K_LG) < 0.08, (ch, "K_LG", lg[ch]["K_e_per_DN"])
        assert abs(hg[ch]["read_e"] - READ_E) < 0.5, (ch, "read_e HG", hg[ch]["read_e"])
        assert abs(rep["Rcg"]["from_ptc_slopes_KLG_over_KHG"][ch] - RCG) < 0.15, ch
        assert abs(rep["Rcg"]["from_signal_ratio_HG_over_LG"][ch] - RCG) < 0.15, ch

    r = c.dcg_demux.BAYER[0]
    print("dcg_characterize self-test PASS: "
          "K_HG=%.3f (exp %.2f)  K_LG=%.3f (exp %.2f)  read_e=%.2f  "
          "Rcg_slopes=%.3f  Rcg_signal=%.3f (exp %.2f)"
          % (hg[r]["K_e_per_DN"], K_HG, lg[r]["K_e_per_DN"], K_LG, hg[r]["read_e"],
             rep["Rcg"]["from_ptc_slopes_KLG_over_KHG"][r],
             rep["Rcg"]["from_signal_ratio_HG_over_LG"][r], RCG))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
