"""Dark-frame integrity check: certify that a "lens capped" black capture is
actually light-tight before trusting it as a measured black level. Prototype
cameras rarely have a proper cap and makeshift caps leak, which would inflate a
measured pedestal (worse than a clean scalar).

Physical signatures of a TRUE dark (vs a light leak):
  - exposure-invariant mean (leaked light accumulates with exposure -> decisive)
  - mean ~ vendor optical-black scalar
  - spatially uniform (leaks are directional -> gradients/hotspots)
  - std ~ read noise (leaked photons add shot noise)
  - balanced Bayer channels (ambient leak is usually non-neutral)
"""
import numpy as np


def dark_stats(frame):
    """Per-frame dark statistics on the RAW frame (no black-level subtraction)."""
    H, W = frame.shape
    He, We = (H // 2) * 2, (W // 2) * 2
    f = frame[:He, :We].astype(np.float64)
    ch = {"R": f[0::2, 0::2], "Gr": f[0::2, 1::2], "Gb": f[1::2, 0::2], "B": f[1::2, 1::2]}
    med = float(np.median(f))
    std = float(f.std())
    # 8x8 block means -> spatial uniformity + gross gradients
    gy, gx = 8, 8
    bh, bw = He // gy, We // gx
    blocks = np.array([[f[i*bh:(i+1)*bh, j*bw:(j+1)*bw].mean() for j in range(gx)]
                       for i in range(gy)])
    return {
        "mean": float(f.mean()),
        "std": std,
        "median": med,
        "p99_9": float(np.percentile(f, 99.9)),
        "max": float(f.max()),
        "hot_frac": float((f > med + 8 * std).mean()),      # bright-tail fraction
        "block_spread": float(blocks.max() - blocks.min()),  # peak-to-peak of block means
        "grad_v": float(f[:He // 2].mean() - f[He // 2:].mean()),
        "grad_h": float(f[:, :We // 2].mean() - f[:, We // 2:].mean()),
        "channels": {k: {"mean": float(v.mean()), "std": float(v.std())} for k, v in ch.items()},
    }


def verdict(short, long_, exp_short_ms, exp_long_ms, pedestal):
    """Combine the two-exposure stats into pass/fail flags + reasons. Thresholds
    are deliberately conservative (a bring-up 'is it leaking' gate, not metrology)."""
    d_exp = max(exp_long_ms - exp_short_ms, 1e-6)
    leak_rate = (long_["mean"] - short["mean"]) / d_exp          # DN / ms
    leak_over_long = long_["mean"] - short["mean"]               # DN accumulated
    chan_spread = (max(c["mean"] for c in long_["channels"].values())
                   - min(c["mean"] for c in long_["channels"].values()))

    checks = {
        "exposure_invariant": leak_over_long < 2.0,             # <2 DN over the whole ramp
        "mean_near_pedestal": abs(short["mean"] - pedestal) < 6.0,
        "spatially_uniform": long_["block_spread"] < 8.0 and abs(long_["grad_v"]) < 4.0
                             and abs(long_["grad_h"]) < 4.0,
        "std_not_inflated": long_["std"] < short["std"] + 3.0,  # shot noise would grow std
        "channels_balanced": chan_spread < 4.0,
    }
    leaky = not (checks["exposure_invariant"] and checks["spatially_uniform"])
    reasons = []
    if not checks["exposure_invariant"]:
        reasons.append("mean rises %.1f DN from %.2f->%.0f ms (%.3f DN/ms) - light is accumulating"
                       % (leak_over_long, exp_short_ms, exp_long_ms, leak_rate))
    if not checks["spatially_uniform"]:
        reasons.append("non-uniform (block spread %.1f DN, grad v/h %.1f/%.1f) - directional leak"
                       % (long_["block_spread"], long_["grad_v"], long_["grad_h"]))
    if not checks["mean_near_pedestal"]:
        reasons.append("mean %.1f DN vs expected pedestal %.0f" % (short["mean"], pedestal))
    if not checks["std_not_inflated"]:
        reasons.append("std grows %.1f->%.1f DN with exposure - shot noise from leaked light"
                       % (short["std"], long_["std"]))
    if not checks["channels_balanced"]:
        reasons.append("Bayer channels diverge %.1f DN - coloured leak" % chan_spread)
    return {
        "leaky": leaky,
        "checks": checks,
        "leak_rate_dn_per_ms": leak_rate,
        "leak_over_ramp_dn": leak_over_long,
        "chan_spread_dn": chan_spread,
        "reasons": reasons,
        "summary": ("cap looks light-tight - measured dark is trustworthy" if not leaky
                    else "POSSIBLE LEAK - use the scalar pedestal, not this measured dark"),
    }
