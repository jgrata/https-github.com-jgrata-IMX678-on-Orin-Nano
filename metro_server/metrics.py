#!/usr/bin/env python3
"""
metrics.py — Metrics-from-RAW for cross-sensor comparison.

The "available now" quantitative column (no dimmable/color lab required): the
figures that decide a sensor's intrinsic HDR ceiling, extractable from a dark
(or min-exposure) frame stack plus optional flat/bracket captures:

  - read noise (temporal, FPN-free)      DN  (-> e- once conversion gain known)
  - DSNU / dark FPN                       DN
  - black level (pedestal)               DN
  - saturation / full-well               DN
  - DYNAMIC RANGE                         stops & dB   <- headline comparison
  - PRNU (photo-response non-uniformity)  %            (needs a clean flat)
  - conversion gain / full-well(e-)/DR(dB) via photon-transfer (needs a STABLE
    uniform source -> lab; the fit helper is here, wire it when the lab exists)

Everything is per-Bayer-phase (R/Gr/Gb/B) plus a global roll-up. Same design as
hdr.py: pure-NumPy functions on RAW + metadata (layer 2, sensor/ISP-agnostic) +
orchestrators that drive the CaptureBackend contract (layer 1). Port to a new
platform by reimplementing the backend only.

Standalone on the Jetson (main server stopped):
    python3 metrics.py --mode 0 --gain 1 --nframes 16 --out /tmp/metrics
    # For accurate read noise CAP THE LENS (dark). Uncapped uses min exposure
    # and the robust per-pixel median, which the dark scene majority dominates.
"""
import json
import numpy as np

_PHASES = {'R': (0, 0), 'Gr': (0, 1), 'Gb': (1, 0), 'B': (1, 1)}   # RGGB


# ══════════════════════════════════════════════════════════════════════════════
# Layer 2 — pure-NumPy metrics on RAW arrays (sensor/ISP-agnostic)
# ══════════════════════════════════════════════════════════════════════════════

def _phase(img, name):
    r, c = _PHASES[name]
    return img[r::2, c::2]


def _dr(sat, black, read_noise):
    wd = max(float(sat) - float(black), 0.0)
    rn = max(float(read_noise), 1e-9)
    return {'dynamic_range_stops': float(np.log2(wd / rn)),
            'dynamic_range_db': float(20.0 * np.log10(wd / rn))}


def read_noise_dsnu(stack, full_scale, sat_dn=None):
    """Read noise, DSNU, black level and dynamic range from a dark (or
    min-exposure) frame stack.

    stack : uint16/float ndarray [N, H, W] — N frames at FIXED exposure/gain,
            ideally lens-capped (dark). Temporal std across the stack is the
            temporal noise (read + any shot); FPN is constant across frames so
            it does NOT enter the temporal std. The per-pixel-median temporal
            std is the read noise, robust to a bright minority if uncapped.
    full_scale : 2**bit_depth - 1 (default saturation if sat_dn is None).

    Returns {phase: {black_dn, read_noise_dn, dsnu_dn, saturation_dn,
                     dynamic_range_stops, dynamic_range_db}, 'global': {...}}.
    """
    st = np.asarray(stack, np.float64)
    if st.ndim != 3 or st.shape[0] < 2:
        raise ValueError("stack must be [N>=2, H, W]")
    sat = full_scale if sat_dn is None else sat_dn
    mean_img = st.mean(axis=0)               # fixed pattern (pedestal + FPN)
    tstd = st.std(axis=0, ddof=1)            # temporal noise per pixel

    out = {}
    for name in _PHASES:
        m = _phase(mean_img, name)
        t = _phase(tstd, name)
        black = float(np.median(m))
        rn = float(np.median(t))             # robust read noise
        dsnu = float(np.std(m))              # spatial FPN
        d = {'black_dn': round(black, 3), 'read_noise_dn': round(rn, 4),
             'dsnu_dn': round(dsnu, 4), 'saturation_dn': float(sat)}
        d.update({k: round(v, 3) for k, v in _dr(sat, black, rn).items()})
        out[name] = d
    black = float(np.median(mean_img)); rn = float(np.median(tstd))
    g = {'black_dn': round(black, 3), 'read_noise_dn': round(rn, 4),
         'dsnu_dn': round(float(np.std(mean_img)), 4), 'saturation_dn': float(sat)}
    g.update({k: round(v, 3) for k, v in _dr(sat, black, rn).items()})
    out['global'] = g
    return out


def _boxlp(a, k):
    """Separable box low-pass (edge-normalized) via cumulative sums — O(n), no
    SciPy. Approximates the smooth (vignetting) component so PRNU can isolate
    the high-frequency response non-uniformity."""
    a = np.asarray(a, np.float64)
    k = int(k) | 1                            # odd window
    r = k // 2

    def box1d(x, axis):
        n = x.shape[axis]
        cs = np.cumsum(x, axis=axis)
        cs = np.concatenate([np.zeros_like(np.take(cs, [0], axis=axis)), cs], axis=axis)
        idx = np.arange(n)
        hi = np.minimum(idx + r + 1, n)
        lo = np.maximum(idx - r, 0)
        take = lambda src, i: np.take(src, i, axis=axis)
        s = take(cs, hi) - take(cs, lo)
        cnt = (hi - lo).astype(np.float64)
        shape = [1] * x.ndim; shape[axis] = n
        return s / cnt.reshape(shape)

    return box1d(box1d(a, 0), 1)


def prnu_map(flat, dark, full_scale, hp_box=65):
    """Photo-response non-uniformity per Bayer phase, from a flat-field frame.

    flat, dark : [H, W] (flat = uniform-source average; dark = matched dark).
    PRNU is the std of the high-frequency (vignetting-removed) normalized
    response, in %. REQUIRES a clean uniform source ~40-70% full scale;
    contamination (a target in frame, dust) inflates it.
    """
    f = np.asarray(flat, np.float64) - np.asarray(dark, np.float64)
    out = {}
    for name in _PHASES:
        sub = np.maximum(_phase(f, name), 1e-9)
        hp = sub / np.maximum(_boxlp(sub, hp_box), 1e-9)   # ~1, high-freq only
        lvl = float(np.median(sub))
        out[name] = {'prnu_pct': round(float(100.0 * np.std(hp)), 4),
                     'level_dn': round(lvl, 1),
                     'level_frac_fs': round(lvl / float(full_scale), 3)}
    return out


def ptc_fit(mean_dn, var_dn, black_dn=0.0, sat_dn=None, lo_frac=0.05, hi_frac=0.70):
    """Photon-transfer fit: variance-vs-mean over an illumination/exposure sweep
    (paired-frame temporal variance). In DN, var = signal/g + read^2, so the
    shot-noise-region slope gives conversion gain g (e-/DN). REQUIRES a stable
    uniform source (lab). Returns conversion gain, read noise (e-), full-well
    (e-), and dynamic range (dB).

    mean_dn, var_dn : arrays over sweep levels (per one Bayer phase).
    """
    m = np.asarray(mean_dn, np.float64) - float(black_dn)
    v = np.asarray(var_dn, np.float64)
    sat = (np.max(m) + black_dn) if sat_dn is None else sat_dn
    hi = hi_frac * (float(sat) - black_dn)
    lo = lo_frac * (float(sat) - black_dn)
    sel = (m > lo) & (m < hi) & np.isfinite(v)
    if sel.sum() < 2:
        return {'ok': False, 'reason': 'not enough shot-noise-region points'}
    slope, intercept = np.polyfit(m[sel], v[sel], 1)
    g = 1.0 / slope if slope > 0 else float('nan')       # e-/DN
    read_dn = np.sqrt(max(intercept, 0.0))
    read_e = read_dn * g
    fw_e = (float(sat) - black_dn) * g
    dr_db = 20.0 * np.log10(max(fw_e, 1e-9) / max(read_e, 1e-9))
    return {'ok': True, 'conversion_gain_e_per_dn': round(float(g), 5),
            'read_noise_dn': round(float(read_dn), 4),
            'read_noise_e': round(float(read_e), 3),
            'full_well_e': round(float(fw_e), 1),
            'dynamic_range_db': round(float(dr_db), 3)}


# ══════════════════════════════════════════════════════════════════════════════
# Layer 1 — orchestrators (drive the CaptureBackend contract from hdr.py)
# ══════════════════════════════════════════════════════════════════════════════

MIN_EXPOSURE_NS = 450000


def measure_intrinsics(backend, nframes=16, gain=1.0, exposure_ns=MIN_EXPOSURE_NS):
    """Capture a fixed-exposure stack and compute the "available now" intrinsic
    sensor metrics (read noise, DSNU, black, saturation, dynamic range). For
    accurate read noise CAP THE LENS; otherwise a short exposure + the robust
    median keeps the dark-scene majority dominant."""
    stack = backend.capture_repeated(nframes, exposure_ns, gain)
    full = (1 << backend.bit_depth) - 1
    res = read_noise_dsnu(stack, full)
    return {'bit_depth': backend.bit_depth, 'gain': float(gain),
            'exposure_ns': int(exposure_ns), 'nframes': int(nframes),
            'phases': res}


def measure_prnu(backend, dark, nframes=8, exposure_ns=None, gain=1.0, hp_box=65):
    """Capture a flat-field stack (point at a clean uniform source), average,
    dark-subtract, and compute PRNU per phase."""
    if exposure_ns is None:
        exposure_ns = 8000000
    stack = backend.capture_repeated(nframes, exposure_ns, gain)
    flat = stack.mean(axis=0)
    full = (1 << backend.bit_depth) - 1
    return {'bit_depth': backend.bit_depth, 'gain': float(gain),
            'exposure_ns': int(exposure_ns),
            'phases': prnu_map(flat, dark, full, hp_box=hp_box)}


def measure_ptc(backend, gain=1.0, black=None, start_ns=MIN_EXPOSURE_NS,
                step=1.5, max_levels=24):
    """Photon-transfer sweep on a UNIFORM STABLE source (lab). Sweeps exposure
    from start_ns up to saturation, 2 frames/level, per-phase mean and temporal
    variance via var(f1-f2)/2 (cancels FPN), then fits per phase (ptc_fit)."""
    full = (1 << backend.bit_depth) - 1
    if black is None:
        black = {n: 0.0 for n in _PHASES}
    elif np.isscalar(black):
        black = {n: float(black) for n in _PHASES}
    sweep = {n: {'mean': [], 'var': []} for n in _PHASES}
    e = int(start_ns)
    for _ in range(max_levels):
        pair = backend.capture_repeated(2, e, gain)
        f1 = pair[0].astype(np.float64); f2 = pair[1].astype(np.float64)
        sat_hit = False
        for n in _PHASES:
            a = _phase(f1, n); b = _phase(f2, n)
            mean = float(np.mean((a + b) / 2.0))
            var = float(np.var(a - b) / 2.0)          # temporal, FPN-cancelled
            sweep[n]['mean'].append(mean); sweep[n]['var'].append(var)
            if mean >= 0.95 * full:
                sat_hit = True
        if sat_hit:
            break
        e = int(e * step)
        if (e / 1e9) > 0.4:                            # exposure ceiling
            break
    fits = {n: ptc_fit(sweep[n]['mean'], sweep[n]['var'],
                       black_dn=black[n], sat_dn=full) for n in _PHASES}
    return {'bit_depth': backend.bit_depth, 'gain': float(gain),
            'sweep': sweep, 'fits': fits}


def sensor_report(backend, nframes=16, gain=1.0, dark_exposure_ns=MIN_EXPOSURE_NS):
    """The day-one sensor-comparison bundle: intrinsic metrics from a dark stack.
    PRNU and PTC are separate (need a clean/stable uniform source)."""
    intr = measure_intrinsics(backend, nframes, gain, dark_exposure_ns)
    g = intr['phases']['global']
    intr['headline'] = {
        'read_noise_dn': g['read_noise_dn'],
        'dynamic_range_stops': g['dynamic_range_stops'],
        'dynamic_range_db': g['dynamic_range_db'],
        'black_dn': g['black_dn'], 'saturation_dn': g['saturation_dn']}
    return intr


# ══════════════════════════════════════════════════════════════════════════════
# Standalone CLI
# ══════════════════════════════════════════════════════════════════════════════

def _cli(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Metrics-from-RAW (standalone)")
    ap.add_argument('--mode', type=int, default=0)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--gain', type=float, default=1.0)
    ap.add_argument('--nframes', type=int, default=16)
    ap.add_argument('--exposure', type=int, default=MIN_EXPOSURE_NS,
                    help='fixed exposure ns for the dark/read-noise stack')
    ap.add_argument('--out', type=str, default='/tmp/metrics')
    ap.add_argument('--no-nvargus-restart', action='store_true')
    args = ap.parse_args(argv)

    import image_server as S
    import hdr
    if not args.no_nvargus_restart:
        S.nvargus_restart()
    rcp = S.RawCaptureProcess()
    rcp.sensor_mode = args.mode; rcp.fps = args.fps
    rcp.exposure_ns = args.exposure; rcp.gain = args.gain
    print("[metrics] starting raw_capture (mode %d)..." % args.mode)
    if not rcp.start():
        print("[metrics] raw_capture failed"); return 1
    try:
        m = S.SENSOR_MODES[args.mode]
        be = hdr.JetsonArgusBackend(rcp, m['bpp'], (m['height'], m['width']))
        print("[metrics] %d-bit  %dx%d  gain=%.2f  nframes=%d  exp=%.3fms" % (
            be.bit_depth, be.shape[1], be.shape[0], args.gain, args.nframes,
            args.exposure / 1e6))
        print("[metrics] (cap the lens for accurate read noise)")
        rep = sensor_report(be, args.nframes, args.gain, args.exposure)
        h = rep['headline']
        print("\n=== SENSOR INTRINSICS (%d-bit, gain %.2f) ===" % (
            rep['bit_depth'], rep['gain']))
        print("  black level      : %.1f DN" % h['black_dn'])
        print("  saturation       : %.0f DN" % h['saturation_dn'])
        print("  read noise       : %.2f DN" % h['read_noise_dn'])
        print("  DYNAMIC RANGE    : %.1f stops (%.1f dB)" % (
            h['dynamic_range_stops'], h['dynamic_range_db']))
        print("  per-phase read noise / DR:")
        for n in ('R', 'Gr', 'Gb', 'B'):
            p = rep['phases'][n]
            print("    %-3s rn=%.2fDN dsnu=%.2fDN DR=%.1f stops" % (
                n, p['read_noise_dn'], p['dsnu_dn'], p['dynamic_range_stops']))
        with open(args.out + '_intrinsics.json', 'w') as f:
            json.dump(rep, f, indent=2)
        print("[metrics] saved -> %s_intrinsics.json" % args.out)
    finally:
        rcp.stop()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_cli())
