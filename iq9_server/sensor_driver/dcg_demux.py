#!/usr/bin/env python3
"""DCG (Clear HDR) frame demux + layout analyzer for the IMX678 — portable to other DCG sensors.

Sony Clear HDR emits two per-pixel readouts each frame: HG (High / High-Conversion-Gain) and
LG (Low / Low-Conversion-Gain), on two MIPI virtual channels. On the IQ9 RDI capture path they
arrive together in one RAW16 buffer with the HG and LG sub-frames stacked. This module:

  1. load_raw()   - load a raw capture into a 2-D uint16 image (given the container width),
  2. guess_width()- rank candidate strides when the padded width isn't known yet,
  3. analyze()    - infer leading OB rows + the HG/LG boundary from the row-mean profile
                    (bench-discovery aid / transport check; needs an illuminated flat frame),
  4. demux()      - split a stacked frame into (HG, LG) given a layout,
  5. leg_stats()  - per-Bayer-channel (RGGB) mean/std + optional net signal for each leg.

Near-term use: with EXP_GAIN=0 the two legs differ ONLY by conversion gain, so HG == HCG and
LG == LCG of the SAME exposure - a simultaneous static LCG/HCG comparator. mean_HG/mean_LG on
net signal is the conversion-gain ratio Rcg (datasheet 2.4-2.9; our standalone build measured 2.4x).

Feed-forward: the same demux + per-leg stats front-ends DCG HDR reconstruction (HG+LG merge on
the measured Rcg) and per-leg WB/CCM. Sensor-agnostic - pass the container bit depth, width, and
leg layout. No board dependency; runs on the analysis host. Requires numpy.
"""
from __future__ import annotations
import argparse
import json
import numpy as np

# RGGB Bayer phase: (row%2, col%2) -> channel
BAYER = ("R", "Gr", "Gb", "B")
_PHASE = {"R": (0, 0), "Gr": (0, 1), "Gb": (1, 0), "B": (1, 1)}


def load_raw(path, width, dtype="<u2"):
    """Load a raw capture as (H, width) little-endian uint16. Height inferred from file size."""
    buf = np.fromfile(path, dtype=np.dtype(dtype))
    if width <= 0 or buf.size % width:
        raise ValueError(f"{buf.size} samples not divisible by width {width} - wrong width/bpp?")
    return buf.reshape(buf.size // width, width)


def guess_width(path, candidates, dtype="<u2"):
    """Rank candidate strides by RGGB even/odd-column separation.

    A correct stride keeps the Bayer phase column-aligned, so the mean of even columns differs
    from odd columns (R/Gb vs Gr/B); a wrong stride scrambles the phase and the separation
    collapses. Returns [(width, height, score)] sorted best-first. Heuristic - confirm visually.
    """
    buf = np.fromfile(path, dtype=np.dtype(dtype)).astype(np.float64)
    out = []
    for w in candidates:
        if w <= 0 or buf.size % w:
            continue
        col = buf.reshape(-1, w).mean(axis=0)
        even, odd = col[0::2], col[1::2]
        n = min(even.size, odd.size)
        out.append((w, buf.size // w, float(np.abs(even[:n] - odd[:n]).mean())))
    out.sort(key=lambda t: -t[2])
    return out


def row_profile(frame):
    """Per-row mean - OB rows read low/flat; the HG/LG boundary shows a step (HG > LG)."""
    return frame.mean(axis=1).astype(np.float64)


def analyze(frame, ob_thresh_frac=0.15):
    """Infer the leg layout from the row-mean profile of an ILLUMINATED (ideally flat) frame.

    HG (HCG) reads ~Rcg x higher than LG (LCG) for the same scene, so a stacked [HG;LG] frame
    steps DOWN at the boundary. Returns leading-OB and boundary estimates; verify before trusting.
    """
    prof = row_profile(frame)
    h = prof.size
    lo, hi = np.percentile(prof, 5), np.percentile(prof, 95)
    ob_level = lo + ob_thresh_frac * (hi - lo)
    est_ob = int(np.count_nonzero(prof[: max(1, h // 8)] < ob_level))
    grad = np.abs(np.diff(prof))
    m0, m1 = h // 3, 2 * h // 3
    bidx = int(np.argmax(grad[m0:m1])) + m0
    return {
        "rows": int(h),
        "cols": int(frame.shape[1]),
        "row_mean_min": float(prof.min()),
        "row_mean_max": float(prof.max()),
        "est_leading_ob_rows": est_ob,
        "est_leg_boundary_row": int(bidx + 1),
        "boundary_step_dn": float(grad[bidx]),
        "note": "boundary = largest row-mean step in the middle third; needs an illuminated flat "
                "target (a dark frame has no step). Confirm OB rows against the SRM appnote.",
    }


def demux(frame, hg_rows, lg_rows, ob_top=0, gap=0, hg_first=True):
    """Split a stacked DCG frame into (HG, LG). Row order: [ob_top][hg_rows][gap][lg_rows]."""
    r = ob_top
    first = frame[r : r + hg_rows]
    r += hg_rows + gap
    second = frame[r : r + lg_rows]
    return (first, second) if hg_first else (second, first)


def leg_stats(leg, black=None):
    """Per-RGGB mean/std for a leg; if `black` (per-channel dict) given, add net signal."""
    out = {}
    for name in BAYER:
        pr, pc = _PHASE[name]
        sub = leg[pr::2, pc::2].astype(np.float64)
        out[name] = {"mean": float(sub.mean()), "std": float(sub.std())}
        if black is not None:
            out[name]["signal"] = out[name]["mean"] - float(black.get(name, 0.0))
    return out


def rcg(hg_stats, lg_stats, black_hg=None, black_lg=None):
    """Conversion-gain ratio per channel = net_HG / net_LG (falls back to raw mean if no black)."""
    r = {}
    for name in BAYER:
        hg = hg_stats[name].get("signal", hg_stats[name]["mean"])
        lg = lg_stats[name].get("signal", lg_stats[name]["mean"])
        r[name] = float(hg / lg) if lg else float("nan")
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description="DCG / Clear HDR demux + layout analyzer")
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("guess-width", help="rank candidate strides for a raw capture")
    g.add_argument("file")
    g.add_argument("--candidates", default="3856,3872,3968,4096,4608")

    a = sub.add_parser("analyze", help="infer OB rows + HG/LG boundary (illuminated flat frame)")
    a.add_argument("file")
    a.add_argument("--width", type=int, required=True)

    s = sub.add_parser("split", help="demux into HG/LG and print per-leg RGGB stats + Rcg")
    s.add_argument("file")
    s.add_argument("--width", type=int, required=True)
    s.add_argument("--hg-rows", type=int, required=True)
    s.add_argument("--lg-rows", type=int, required=True)
    s.add_argument("--ob-top", type=int, default=0)
    s.add_argument("--gap", type=int, default=0)
    s.add_argument("--lg-first", action="store_true", help="frame is stacked [LG;HG] not [HG;LG]")
    s.add_argument("--black", default=None, help="per-channel black as R,Gr,Gb,B (e.g. 200,200,200,200)")
    s.add_argument("--out-prefix", default=None, help="also write <prefix>_HG.raw / _LG.raw")

    args = ap.parse_args(argv)

    if args.cmd == "guess-width":
        for w, h, score in guess_width(args.file, [int(x) for x in args.candidates.split(",")]):
            print(f"  width={w:5d}  height={h:6d}  bayer-score={score:8.2f}")
        return 0

    if args.cmd == "analyze":
        print(json.dumps(analyze(load_raw(args.file, args.width)), indent=2))
        return 0

    if args.cmd == "split":
        frame = load_raw(args.file, args.width)
        hg, lg = demux(frame, args.hg_rows, args.lg_rows, args.ob_top, args.gap,
                       hg_first=not args.lg_first)
        black = None
        if args.black:
            vals = [float(x) for x in args.black.split(",")]
            black = dict(zip(BAYER, vals))
        hs, ls = leg_stats(hg, black), leg_stats(lg, black)
        print(json.dumps({"HG_HCG": hs, "LG_LCG": ls,
                          "Rcg_HG_over_LG": rcg(hs, ls)}, indent=2))
        if not args.black:
            print("# note: Rcg on raw means; pass --black R,Gr,Gb,B (pedestal) for the true ratio")
        if args.out_prefix:
            hg.tofile(args.out_prefix + "_HG.raw")
            lg.tofile(args.out_prefix + "_LG.raw")
            print(f"wrote {args.out_prefix}_HG.raw / {args.out_prefix}_LG.raw")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
