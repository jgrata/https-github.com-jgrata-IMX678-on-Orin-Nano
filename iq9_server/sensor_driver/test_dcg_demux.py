#!/usr/bin/env python3
"""Self-test for dcg_demux on synthetic stacked Clear HDR frames (no board, tiny frames).

Builds a [OB][HG][LG] stack where HG (HCG) reads Rcg x brighter than LG (LCG) for the same
"scene", then checks that analyze() finds the leg boundary and that demux()+leg_stats()+rcg()
recover the conversion-gain ratio. Documents the layout the bench capture is expected to match.
Run: python test_dcg_demux.py
"""
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dcg_demux as d


def build(width=64, ob=4, hg=40, lg=40, ped=200.0, col_off=50.0, sig_hcg=400.0, rcg=2.4, seed=0):
    """Synthetic stacked DCG frame. HG signal = sig_hcg; LG signal = sig_hcg / rcg (same scene).

    A per-column offset (odd columns = Gr/B phases) gives guess_width a Bayer phase to lock onto
    and makes the per-channel pedestal non-uniform, so leg_stats/rcg are exercised phase-correctly.
    """
    rng = np.random.default_rng(seed)
    f = np.full((ob + hg + lg, width), ped)
    f[:, 1::2] += col_off  # odd columns (Gr, B) sit col_off above R, Gb
    f[ob:ob + hg] += sig_hcg + rng.normal(0, 2, (hg, width))
    f[ob + hg:ob + hg + lg] += sig_hcg / rcg + rng.normal(0, 2, (lg, width))
    black = {"R": ped, "Gr": ped + col_off, "Gb": ped, "B": ped + col_off}
    return np.clip(f, 0, 65535).astype("<u2"), dict(
        width=width, ob=ob, hg=hg, lg=lg, ped=ped, rcg=rcg, black=black)


def main():
    frame, m = build()

    # 1) analyze() locates the HG/LG boundary (largest row-mean step)
    a = d.analyze(frame)
    exp_boundary = m["ob"] + m["hg"]
    assert abs(a["est_leg_boundary_row"] - exp_boundary) <= 3, (a["est_leg_boundary_row"], exp_boundary)

    # 2) demux() + rcg() recover the conversion-gain ratio on net signal
    hg, lg = d.demux(frame, m["hg"], m["lg"], ob_top=m["ob"])
    hs, ls = d.leg_stats(hg, m["black"]), d.leg_stats(lg, m["black"])
    r = d.rcg(hs, ls)
    for ch, val in r.items():
        assert abs(val - m["rcg"]) < 0.1, (ch, val, m["rcg"])

    # 3) load_raw round-trips and guess_width returns the true stride as a candidate
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as tf:
        path = tf.name
    frame.tofile(path)
    reloaded = d.load_raw(path, m["width"])
    ranked = d.guess_width(path, [48, 64, 96])
    os.unlink(path)
    assert reloaded.shape == frame.shape, (reloaded.shape, frame.shape)
    assert any(w == m["width"] for w, _, _ in ranked), ranked

    print("dcg_demux self-test PASS: boundary=%d (exp %d)  Rcg=%.3f (exp %.2f)"
          % (a["est_leg_boundary_row"], exp_boundary, sum(r.values()) / len(r), m["rcg"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
