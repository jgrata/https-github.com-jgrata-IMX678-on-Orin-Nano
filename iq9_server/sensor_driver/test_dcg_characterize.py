#!/usr/bin/env python3
"""Self-test for dcg_characterize: synthetic photon-transfer frames with known gains + OB + DR.

Generates stacked [HG_OB][HG][LG_OB][LG] Clear HDR frames where both legs see the same photons
but read with different conversion gains (K_HG=0.5, K_LG=1.2 e-/DN -> Rcg=2.4), a constant ADC
read noise in DN (so HCG has lower read noise in e-), per-leg black levels, and per-leg full-well
(LCG holds ~5x more). Checks: per-frame optical-black pedestal/read-noise, K per leg, read noise,
Rcg two ways, and the dynamic-range metrics (combined DR, gain-over-LCG, DR ordering, hand-off SNR).
Run: python test_dcg_characterize.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dcg_characterize as c

K_HG, K_LG = 0.5, 1.2            # e-/DN (HCG higher conversion gain -> smaller K)
READ_DN = 4.0                   # constant ADC read noise (DN) -> read_e = READ_DN*K, HCG lower
PED_HG, PED_LG = 200.0, 170.0   # per-leg black (different) -> exercises per-leg OB
COL_OFF = 50.0                  # Bayer odd-column offset (OB removes it per channel)
FW_HG_E, FW_LG_E = 8000.0, 40000.0   # LCG holds ~5x more -> DCG combined DR > single-leg
W, OBR, HG, LG = 256, 8, 200, 200
RCG = K_LG / K_HG               # 2.4
LAYOUT = dict(hg_rows=HG, lg_rows=LG, ob_top=OBR, gap=OBR, hg_first=True,
              ob_hg=(0, OBR), ob_lg=(OBR + HG, OBR + HG + OBR))


def stacked(s_e, rng):
    """One stacked DCG frame: [HG_OB][HG][LG_OB][LG] at signal s_e electrons (same photons)."""
    f = np.zeros((OBR + HG + OBR + LG, W))

    def fill(r0, r1, ped, k, fw_e, signal):
        n = r1 - r0
        blk = np.full((n, W), ped)
        blk[:, 1::2] += COL_OFF
        if signal and s_e > 0:
            e = np.minimum(rng.poisson(s_e, (n, W)).astype(float), fw_e)  # clip at full-well
            blk = blk + e / k
        blk = blk + rng.normal(0, READ_DN, (n, W))                        # read noise in DN
        f[r0:r1] = blk

    fill(0, OBR, PED_HG, K_HG, FW_HG_E, False)
    fill(OBR, OBR + HG, PED_HG, K_HG, FW_HG_E, True)
    fill(OBR + HG, OBR + HG + OBR, PED_LG, K_LG, FW_LG_E, False)
    fill(OBR + HG + OBR, OBR + HG + OBR + LG, PED_LG, K_LG, FW_LG_E, True)
    return np.clip(f, 0, 65535).astype("<u2")


def main():
    rng = np.random.default_rng(3)
    levels = [200, 500, 1500, 4000, 7000, 9000, 20000, 38000, 44000]
    points = [{"level": L, "meas": c.measure([stacked(L, rng) for _ in range(2)], LAYOUT)}
              for L in levels]
    # no dark point: the masked rows provide pedestal + read noise per frame
    rep = c.characterize(points, dark=None, ref_level=4000)
    hg = rep["legs"]["HG"]["channels"]
    lg = rep["legs"]["LG"]["channels"]
    dr = rep["dynamic_range"]
    expect_gain_db = 20.0 * math.log10(RCG)  # ~7.6 dB

    for ch in c.dcg_demux.BAYER:
        assert hg[ch]["read_noise_source"].startswith("optical-black"), hg[ch]["read_noise_source"]
        assert abs(hg[ch]["K_e_per_DN"] - K_HG) < 0.05, (ch, "K_HG", hg[ch]["K_e_per_DN"])
        assert abs(lg[ch]["K_e_per_DN"] - K_LG) < 0.10, (ch, "K_LG", lg[ch]["K_e_per_DN"])
        assert abs(hg[ch]["read_e"] - READ_DN * K_HG) < 0.6, (ch, "read_e HG", hg[ch]["read_e"])
        assert abs(lg[ch]["read_e"] - READ_DN * K_LG) < 1.0, (ch, "read_e LG", lg[ch]["read_e"])
        assert abs(rep["Rcg"]["from_ptc_slopes_KLG_over_KHG"][ch] - RCG) < 0.15, ch
        assert abs(rep["Rcg"]["from_signal_ratio_HG_over_LG"][ch] - RCG) < 0.15, ch
        assert abs(dr[ch]["gain_over_LCG_dB"] - expect_gain_db) < 0.7, (ch, dr[ch]["gain_over_LCG_dB"])
        assert dr[ch]["DR_combined_dB"] > dr[ch]["DR_LG_dB"] > dr[ch]["DR_HG_dB"], (ch, dr[ch])
        assert dr[ch]["handoff_SNR"] > 10, (ch, dr[ch]["handoff_SNR"])

    r = c.dcg_demux.BAYER[0]
    print("dcg_characterize self-test PASS: "
          "K_HG=%.3f K_LG=%.3f  read_e HG/LG=%.2f/%.2f  Rcg=%.3f/%.3f  "
          "DR HG/LG/comb=%.1f/%.1f/%.1f dB  gain_over_LCG=%.2f dB (exp %.2f)  handoff_SNR=%.0f"
          % (hg[r]["K_e_per_DN"], lg[r]["K_e_per_DN"], hg[r]["read_e"], lg[r]["read_e"],
             rep["Rcg"]["from_ptc_slopes_KLG_over_KHG"][r],
             rep["Rcg"]["from_signal_ratio_HG_over_LG"][r],
             dr[r]["DR_HG_dB"], dr[r]["DR_LG_dB"], dr[r]["DR_combined_dB"],
             dr[r]["gain_over_LCG_dB"], expect_gain_db, dr[r]["handoff_SNR"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
