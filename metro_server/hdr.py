#!/usr/bin/env python3
"""
hdr.py — Onboard software-bracket HDR for the RAW DAQ.

Two layers, deliberately separated so the evaluation core ports across
platforms (Jetson -> IQ9) and sensors (IMX678 -> IMX908/828/1H1):

  1. CAPTURE-SHIM CONTRACT (`CaptureBackend`) — the ONLY platform/sensor/ISP
     coupling. Reimplement it per platform. Everything below depends solely on
     this interface, not on Argus/nvargus/V4L2.

  2. MERGE + ANALYSIS (`reconstruct_radiance`, `tonemap`, ...) — pure NumPy on
     RAW Bayer + per-frame metadata. No platform knowledge. This is the part we
     want to keep identical across sensors so comparisons are apples-to-apples.

The merge mirrors the validated MATLAB client (ECamHDRClient.reconstructRadiance):
RAW is already linear, so no camera response curve — each leg contributes a
triangle-weighted (DN - black)/(exposure*gain) estimate, saturated/near-black
pixels rejected, averaged to RELATIVE linear radiance.

Run standalone on the Jetson (needs nvargus + raw_capture, main server stopped):
    python3 -m metro_server.hdr --mode 0 --gain 1 \
        --exposures 2000000,8000000,32000000,128000000 --out /tmp/hdr_test
"""
import time
import json
import numpy as np


# ══════════════════════════════════════════════════════════════════════════════
# Layer 2 — platform-agnostic merge + analysis (pure NumPy on RAW + metadata)
# ══════════════════════════════════════════════════════════════════════════════

def reconstruct_radiance(frames, exposures_ns, gains, full_scale,
                         black_level=0.0, satfrac=0.95, flatfield=None):
    """Combine an exposure bracket into a relative linear radiance map.

    Port of ECamHDRClient.reconstructRadiance. RAW sensor data is linear, so
    no response curve: each leg's per-pixel estimate is
        (DN - black_level) / (exposure_s * gain)
    weighted by a triangle that peaks mid-range and rejects saturated and
    noise-floor pixels, then averaged.

    Args:
        frames       : uint16 ndarray [N, H, W] — native bit depth (10/12).
        exposures_ns : sequence[N] of ACTUAL exposure (ns) per leg.
        gains        : sequence[N] of ACTUAL analog gain (x) per leg.
        full_scale   : saturation DN (2**bit_depth - 1).
        black_level  : scalar DN or per-pixel float ndarray [H, W] (dark frame).
        satfrac      : saturation cutoff as a fraction of full_scale (0.95).
        flatfield    : optional per-pixel response map [H, W]; radiance /= it.

    Returns:
        (rad, wsum): rad = float32 [H, W] relative linear radiance;
                     wsum = float32 [H, W] summed weight (0 => no leg covered
                     that pixel — over/under-exposed everywhere).
    """
    frames = np.asarray(frames)
    if frames.ndim != 3:
        raise ValueError("frames must be [N, H, W]")
    N, H, W = frames.shape
    if N != len(exposures_ns) or N != len(gains):
        raise ValueError("exposures_ns/gains length must match frame count")

    sat = float(satfrac) * float(full_scale)
    bl  = np.asarray(black_level, dtype=np.float32)
    eps = np.finfo(np.float32).eps

    rad  = np.zeros((H, W), np.float32)
    wsum = np.zeros((H, W), np.float32)
    for k in range(N):
        f   = frames[k].astype(np.float32)
        eff = (float(exposures_ns[k]) / 1e9) * max(float(gains[k]), eps)
        # triangle weight: peaks mid-range, 0 at black floor and at saturation
        w = np.minimum(f - bl, sat - f)
        np.maximum(w, 0.0, out=w)
        w[f >= sat] = 0.0
        rad  += w * (np.maximum(f - bl, 0.0) / eff)
        wsum += w
    rad /= np.maximum(wsum, eps)
    if flatfield is not None:
        rad /= np.maximum(np.asarray(flatfield, np.float32), eps)
    return np.nan_to_num(rad, nan=0.0, posinf=0.0, neginf=0.0), wsum


def bayer_to_luma(mono):
    """Average each 2x2 Bayer quad -> a half-resolution luma image [H/2, W/2].
    Removes the Bayer checkerboard for a clean grayscale quicklook. Sensor-
    independent (assumes a 2x2 CFA period, true for RGGB/BGGR/etc.)."""
    H, W = mono.shape
    H2, W2 = H // 2, W // 2
    q = mono[:2 * H2, :2 * W2].reshape(H2, 2, W2, 2)
    return q.mean(axis=(1, 3))


def tonemap(rad, key=0.18, hi_pct=99.5, gamma=2.2):
    """Fixed, sensor-independent Reinhard tonemap of a linear radiance map ->
    uint8 luma [H/2, W/2] for VISUAL comparison only (not analysis).

    Deliberately parameter-fixed so every sensor is rendered the same way. Runs
    on binned luma to avoid Bayer artifacts. Not color — a downstream demosaic
    + CCM is a separate step (color needs the lab-calibrated CCM anyway)."""
    L = np.maximum(bayer_to_luma(rad), 0.0).astype(np.float32)
    eps = 1e-6
    log_avg = np.exp(np.mean(np.log(L + eps)))          # log-average luminance
    Ls = (key / (log_avg + eps)) * L
    Ld = Ls / (1.0 + Ls)                                # Reinhard global
    hi = np.percentile(Ld, hi_pct)
    hi = max(float(hi), eps)
    out = np.clip(Ld / hi, 0.0, 1.0) ** (1.0 / gamma)
    return (out * 255.0 + 0.5).astype(np.uint8)


def radiance_to_u16(rad, hi_pct=99.9):
    """Pack a linear radiance map into a 16-bit linear container (compact,
    reversible). Returns (u16 [H, W], scale) where rad ~= u16 / scale.
    The high percentile sets the scale so bright outliers don't crush the range.
    """
    finite = rad[np.isfinite(rad) & (rad > 0)]
    hi = np.percentile(finite, hi_pct) if finite.size else 1.0
    hi = max(float(hi), 1e-12)
    scale = 65535.0 / hi
    u16 = np.clip(rad * scale, 0, 65535).astype(np.uint16)
    return u16, scale


def coverage_stats(frames, full_scale, satfrac=0.95, black_level=0.0,
                   noise_floor_frac=0.02):
    """Per-leg saturation/usable coverage — a 'available now' metric (no lab).
    For each leg reports the fraction of pixels saturated, below the noise
    floor, and usable; plus the composite fraction covered by >=1 leg."""
    frames = np.asarray(frames)
    N, H, W = frames.shape
    sat = satfrac * full_scale
    floor = float(black_level if np.isscalar(black_level) else np.median(black_level)) \
        + noise_floor_frac * full_scale
    covered = np.zeros((H, W), bool)
    legs = []
    for k in range(N):
        f = frames[k].astype(np.float32)
        s = f >= sat
        lo = f <= floor
        usable = ~s & ~lo
        covered |= usable
        legs.append({'saturated_frac': float(s.mean()),
                     'below_floor_frac': float(lo.mean()),
                     'usable_frac': float(usable.mean())})
    return {'legs': legs, 'composite_covered_frac': float(covered.mean())}


# ══════════════════════════════════════════════════════════════════════════════
# Layer 1 — the capture-shim CONTRACT (reimplement per platform)
# ══════════════════════════════════════════════════════════════════════════════

class CaptureBackend:
    """Platform capture-shim contract for bracket-HDR.

    THIS is the only platform/sensor/ISP-specific surface. Port to a new SoC/ISP
    (e.g. IQ9) by reimplementing this class against that capture API; the merge
    and analysis layer above moves unchanged.

    Contract:
        request N x (exposure_ns, gain[, conv_gain])
          -> N x RAW Bayer frames (native bit depth) + per-frame ACTUAL metadata
        black level obtained separately (measure_black_level).
    """

    @property
    def bit_depth(self):
        """Native RAW bit depth (e.g. 10 or 12)."""
        raise NotImplementedError

    @property
    def shape(self):
        """(H, W) of a single RAW frame."""
        raise NotImplementedError

    def capture_bracket(self, exposures_ns, gain, conv_gain=None, settle=True):
        """Capture one RAW frame per exposure, all at the SAME (fixed) gain.

        Args:
            exposures_ns : sequence of exposure times (ns).
            gain         : analog gain (x), held fixed across the bracket.
            conv_gain    : optional conversion-gain / HCG-LCG mode selector for
                           DCG sensors (None = leave as-is). Backends that don't
                           support it must raise if a non-None value is passed.
            settle       : wait for each exposure to actually take effect (static
                           scenes). False = capture immediately (motion / speed).

        Returns:
            (frames, metas):
              frames : uint16 ndarray [N, H, W] at native bit depth.
              metas  : list[dict] with ACTUAL {'requested_ns','exposure_ns',
                       'gain'} per leg (drives the radiance weighting).
        """
        raise NotImplementedError

    def measure_black_level(self, nframes=8, gain=1.0):
        """Per-pixel dark frame (float32 [H, W]) at min exposure, given gain.
        Caller must ensure a dark scene (cap the lens). Black level is
        gain-dependent — measure at the bracket's gain."""
        raise NotImplementedError


class JetsonArgusBackend(CaptureBackend):
    """Jetson/e-CAM86 implementation of the capture-shim, over a
    RawCaptureProcess (raw_capture --server, Argus/CUDA RAW16).

    Wraps the existing rcp so it works both inside image_server (pass the live
    prefetcher to pause it during a bracket) and standalone from the CLI."""

    MIN_EXPOSURE_NS = 450000     # sensor floor (see --list-modes)

    def __init__(self, rcp, native_bpp, shape, prefetcher=None):
        self.rcp = rcp
        self._bpp = int(native_bpp)
        self._shape = (int(shape[0]), int(shape[1]))
        self.pf = prefetcher

    @property
    def bit_depth(self):
        return self._bpp

    @property
    def shape(self):
        return self._shape

    def _grab_native(self):
        """One RAW frame at the CURRENT exposure/gain, native bits (no scaling).
        Returns (frame uint16 [H, W], actual_exp_ns, actual_gain)."""
        px, w, h, bpp, nf, exp_ns, gain_x1000 = self.rcp.capture()
        return px[:w * h].reshape(h, w), int(exp_ns), gain_x1000 / 1000.0

    def _settle(self, target_ns):
        """Block until the sensor's ACTUAL exposure reaches target (within 5%),
        or a timeout (mode/range clamp). Mirrors ECamHDRClient.waitExposureSettle:
        the new exposure takes a few frames to appear and the OLD value is
        briefly stable, so flush a few frame periods first, then poll — never a
        stability shortcut. Timing scales with exposure (long exp lowers fps)."""
        target = float(target_ns)
        fp = max(0.03, target / 1e9)              # new frame period
        time.sleep(3 * fp)                        # let the change flush in
        deadline = time.monotonic() + max(2.5, 10 * fp)
        while time.monotonic() < deadline:
            _, exp, _ = self._grab_native()
            if exp > 0 and abs(exp - target) <= 0.05 * max(target, 1.0):
                return True
            time.sleep(fp)
        return False

    def capture_bracket(self, exposures_ns, gain, conv_gain=None, settle=True):
        if conv_gain is not None:
            raise NotImplementedError(
                "conversion-gain axis not supported on the e-CAM86/Argus backend")
        exps = [int(round(e)) for e in exposures_ns]
        if not exps:
            raise ValueError("need >= 1 exposure")
        H, W = self._shape
        gain = float(gain)
        frames = np.empty((len(exps), H, W), np.uint16)
        metas = []

        paused = self.pf is not None
        if paused:
            self.pf.pause()
            time.sleep(0.05)                      # let any in-flight grab finish
        orig_exp, orig_gain = self.rcp.exposure_ns, self.rcp.gain
        try:
            for k, e in enumerate(exps):
                self.rcp.set_expgain_live(e, gain)    # exposure varies, gain pinned
                if settle:
                    self._settle(e)
                self._grab_native()                   # flush one settled frame
                fr, aexp, again = self._grab_native() # keep the next
                frames[k] = fr
                metas.append({'requested_ns': e,
                              'exposure_ns': aexp if aexp > 0 else e,
                              'gain': again if again > 0 else gain})
        finally:
            self.rcp.set_expgain_live(orig_exp, orig_gain)   # restore
            if paused:
                self.pf.resume()
        return frames, metas

    def measure_black_level(self, nframes=8, gain=1.0):
        H, W = self._shape
        paused = self.pf is not None
        if paused:
            self.pf.pause(); time.sleep(0.05)
        orig_exp, orig_gain = self.rcp.exposure_ns, self.rcp.gain
        try:
            self.rcp.set_expgain_live(self.MIN_EXPOSURE_NS, float(gain))
            self._settle(self.MIN_EXPOSURE_NS)
            self._grab_native()                    # flush transitional
            acc = np.zeros((H, W), np.float64)
            for _ in range(int(nframes)):
                fr, _, _ = self._grab_native()
                acc += fr
        finally:
            self.rcp.set_expgain_live(orig_exp, orig_gain)
            if paused:
                self.pf.resume()
        return (acc / float(nframes)).astype(np.float32)


# ══════════════════════════════════════════════════════════════════════════════
# Orchestrator — capture a bracket and merge it (uses only the contract above)
# ══════════════════════════════════════════════════════════════════════════════

def capture_bracket_hdr(backend, exposures_ns, gain=1.0, satfrac=0.95,
                        black_level=0.0, flatfield=None, conv_gain=None,
                        settle=True, make_preview=True, make_coverage=True):
    """Capture an exposure bracket via `backend` and merge to linear radiance.

    Returns a dict:
        radiance   : float32 [H, W] relative linear radiance
        wsum       : float32 [H, W] summed weight (0 = uncovered)
        metas      : per-leg actual {requested_ns, exposure_ns, gain}
        bit_depth, shape, satfrac
        preview    : uint8 [H/2, W/2] tonemapped luma (if make_preview)
        coverage   : per-leg + composite coverage stats (if make_coverage)
    """
    frames, metas = backend.capture_bracket(exposures_ns, gain,
                                            conv_gain=conv_gain, settle=settle)
    full = (1 << backend.bit_depth) - 1
    exps = [m['exposure_ns'] for m in metas]
    gains = [m['gain'] for m in metas]
    rad, wsum = reconstruct_radiance(frames, exps, gains, full,
                                     black_level=black_level, satfrac=satfrac,
                                     flatfield=flatfield)
    out = {'radiance': rad, 'wsum': wsum, 'metas': metas,
           'bit_depth': backend.bit_depth, 'shape': backend.shape,
           'satfrac': satfrac}
    if make_preview:
        out['preview'] = tonemap(rad)
    if make_coverage:
        out['coverage'] = coverage_stats(frames, full, satfrac, black_level)
    return out


# ── lib-free savers (PGM: universally viewable, no PIL/imageio dependency) ─────

def _write_pgm(path, img_u8_or_u16):
    """Write a grayscale PGM (P5). 8-bit or 16-bit (16-bit is big-endian)."""
    a = np.asarray(img_u8_or_u16)
    H, W = a.shape
    maxval = 255 if a.dtype == np.uint8 else 65535
    with open(path, 'wb') as f:
        f.write(("P5\n%d %d\n%d\n" % (W, H, maxval)).encode())
        (a.astype('>u2') if maxval == 65535 else a.astype(np.uint8)).tofile(f)


def save_result(result, out_prefix, black_level=0.0):
    """Persist an HDR result for offline analysis + a viewable quicklook.
        <prefix>_radiance.npy : float32 relative linear radiance (analysis)
        <prefix>_hdr16.pgm    : 16-bit linear container (viewable, reversible)
        <prefix>_preview.pgm  : 8-bit tonemapped luma quicklook
        <prefix>_meta.json    : metas, scale, coverage, params
    """
    rad = result['radiance']
    np.save(out_prefix + '_radiance.npy', rad)
    u16, scale = radiance_to_u16(rad)
    _write_pgm(out_prefix + '_hdr16.pgm', u16)
    if 'preview' in result:
        _write_pgm(out_prefix + '_preview.pgm', result['preview'])
    meta = {'metas': result['metas'], 'bit_depth': result['bit_depth'],
            'shape': list(result['shape']), 'satfrac': result['satfrac'],
            'radiance_u16_scale': scale,
            'black_level': (black_level if np.isscalar(black_level)
                            else 'per-pixel dark frame'),
            'coverage': result.get('coverage')}
    with open(out_prefix + '_meta.json', 'w') as f:
        json.dump(meta, f, indent=2)
    return meta


# ══════════════════════════════════════════════════════════════════════════════
# Standalone CLI — validate the onboard capture->merge path (main server stopped)
# ══════════════════════════════════════════════════════════════════════════════

def _cli(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Onboard bracket-HDR (standalone)")
    ap.add_argument('--mode', type=int, default=0, help='sensor mode (0=4K12b)')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--gain', type=float, default=1.0)
    ap.add_argument('--exposures', type=str,
                    default='2000000,8000000,32000000,128000000',
                    help='comma-separated exposure ns')
    ap.add_argument('--satfrac', type=float, default=0.95)
    ap.add_argument('--dark', action='store_true',
                    help='measure a black-level frame first (CAP THE LENS)')
    ap.add_argument('--out', type=str, default='/tmp/hdr_test')
    ap.add_argument('--no-nvargus-restart', action='store_true')
    args = ap.parse_args(argv)

    # Lazy import so this module stays free of image_server at import time.
    import image_server as S

    exposures = [int(x) for x in args.exposures.split(',') if x.strip()]
    if not args.no_nvargus_restart:
        S.nvargus_restart()

    rcp = S.RawCaptureProcess()
    rcp.sensor_mode = args.mode
    rcp.fps = args.fps
    rcp.exposure_ns = exposures[0]
    rcp.gain = args.gain
    print("[hdr] starting raw_capture (mode %d)..." % args.mode)
    if not rcp.start():
        print("[hdr] raw_capture failed to start"); return 1

    try:
        m = S.SENSOR_MODES[args.mode]
        backend = JetsonArgusBackend(rcp, m['bpp'], (m['height'], m['width']))
        print("[hdr] backend: %d-bit  %dx%d" %
              (backend.bit_depth, backend.shape[1], backend.shape[0]))

        black = 0.0
        if args.dark:
            print("[hdr] measuring black level (lens should be capped)...")
            black = backend.measure_black_level(nframes=8, gain=args.gain)
            print("[hdr] black median = %.1f DN" % float(np.median(black)))

        print("[hdr] capturing bracket: %s" % exposures)
        t0 = time.monotonic()
        result = capture_bracket_hdr(backend, exposures, gain=args.gain,
                                     satfrac=args.satfrac, black_level=black)
        dt = time.monotonic() - t0
        for k, mm in enumerate(result['metas']):
            print("[hdr]  leg %d: req=%.2fms actual=%.2fms gain=%.3fx" %
                  (k, mm['requested_ns'] / 1e6, mm['exposure_ns'] / 1e6, mm['gain']))
        cov = result.get('coverage', {})
        print("[hdr] composite coverage: %.1f%%  (%.2fs)" %
              (100 * cov.get('composite_covered_frac', 0), dt))
        r = result['radiance']
        print("[hdr] radiance: min=%.3g max=%.3g median=%.3g" %
              (float(r.min()), float(r.max()), float(np.median(r))))
        save_result(result, args.out, black_level=black)
        print("[hdr] saved -> %s_{radiance.npy,hdr16.pgm,preview.pgm,meta.json}"
              % args.out)
    finally:
        rcp.stop()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_cli())
