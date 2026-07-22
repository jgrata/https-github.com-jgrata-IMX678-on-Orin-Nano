"""Faithful numpy port of jslantedge.m (slanted-edge SFR / MTF) and its jcode
dependency chain: jedgefind -> jmoments, jorthfit; japodize; internal jfft.
Pure numpy (no scipy/cv2). Indexing/rounding kept bit-faithful to MATLAB so a
regression against the MATLAB output matches to float tolerance.

    freq, mtf, esf, lsf, BW, out = jslantedge(I, osf, pixel)
"""
import numpy as np


def _mround(x):
    """MATLAB round(): half away from zero (numpy rounds half-to-even)."""
    return np.sign(x) * np.floor(np.abs(x) + 0.5)


def jmoments(x):
    """m00, m10 (column moment, 0-based), m01 (row moment, 0-based)."""
    x = np.asarray(x, float)
    rows, cols = x.shape
    m00 = x.sum()
    m10 = np.arange(cols) @ x.sum(axis=0)     # sum_j j * sum_i x[i,j]
    m01 = np.arange(rows) @ x.sum(axis=1)     # sum_i i * sum_j x[i,j]
    return np.array([m00, m10, m01])


def jorthfit(x, y):
    """Orthogonal (total-least-squares) line fit -> [1, slope, mean(x), mean(y)]."""
    x = np.asarray(x, float).ravel()
    y = np.asarray(y, float).ravel()
    hcav = x.mean()
    ycorav = y.mean()
    sxy = np.sum((y - ycorav) * (x - hcav))
    syy = np.sum((y - ycorav) ** 2)
    sxx = np.sum((x - hcav) ** 2)
    cc = np.sqrt((sxx - syy) ** 2 + 4 * sxy ** 2)
    slope = (-(sxx - syy) + cc) / (2 * sxy)
    return np.array([1.0, slope, x.mean(), y.mean()])


def jedgefind(I):
    """Locate the slant edge. Returns (slopeintercept [polyfit, unused by
    jslantedge], si [orthogonal-fit line used by jslantedge])."""
    I = np.asarray(I, float)
    esf = I.mean(axis=0)                       # MATLAB mean(I): mean over rows -> len W
    maxesf = esf.max()
    minesf = esf.min()
    maxesfpos = int(np.argmax(esf))           # 0-based; only used in < comparison
    minesfpos = int(np.argmin(esf))
    if maxesfpos < minesfpos:
        psf = esf[:-2] - esf[2:]
        xg = I[:, :-2] - I[:, 2:]
    else:
        psf = esf[2:] - esf[:-2]
        xg = I[:, 2:] - I[:, :-2]
    L = psf.shape[0]
    maxpsf = psf.max()
    maxpsfpos = int(np.argmax(psf)) + 1       # 1-based (matches MATLAB find(...,1))
    idx1 = np.arange(1, L + 1)                 # 1-based indices
    triangwindow = 1.0 / (1 + np.abs(maxpsfpos - idx1) / (0.5 * L))
    windowedpsf = psf * triangwindow
    psfpeakinds = idx1[windowedpsf > maxpsf / 2.0]      # 1-based index values
    psf_at = psf[psfpeakinds - 1]
    psfpeakcentroid = np.sum(psfpeakinds * psf_at) / np.sum(psf_at)
    peakwidth = 2 * (maxesf - minesf) / np.sum(psf_at) * len(psfpeakinds)
    edgelims = np.array([psfpeakcentroid - peakwidth, psfpeakcentroid + peakwidth])
    if edgelims[0] < 1:
        edgelims[0] = 1
    if edgelims[1] > L:
        edgelims[1] = L
    edgelims = _mround(edgelims).astype(int)
    xseg = xg[:, edgelims[0] - 1:edgelims[1]]          # 1-based inclusive -> slice
    nrows = xseg.shape[0]
    moments = np.zeros((nrows - 1, 3))
    for i in range(nrows - 1):
        moments[i, :] = jmoments(xseg[i:i + 2, :])
    momentratio = moments[:, 1] / moments[:, 0]        # m10/m00
    n = len(momentratio)
    xax = np.arange(1, n + 1)                           # 1:length(momentratio)
    p = np.polyfit(xax, momentratio, 1)
    slopeintercept = np.array([1.0 / p[0], -p[1] / p[0]])
    lin = jorthfit(xax, momentratio + edgelims[0])     # momentratio' + edgelims(1)
    si = np.array([lin[0] / lin[1], lin[2] - lin[0] * lin[3] / lin[1]])
    return slopeintercept, si


def japodize(esf, osf):
    """Center + Hamming-window the LSF. esf is [N,2] (col0 dist, col1 signal)."""
    esf = np.asarray(esf, float)
    col = esf[:, 1]
    ms = (col.max() + col.min()) / 2.0
    l = esf.shape[0]
    midind = int(np.argmin(np.abs(col - ms))) + 1      # 1-based first occurrence
    if midind - 1 > l - midind:
        esf = esf[(midind - (l - midind)) - 1:, :]
    elif midind - 1 < l - midind:
        esf = esf[:midind + (midind - 1), :]
    lsf = np.gradient(esf[:, 1])
    Lp = len(lsf)
    N = Lp - 1
    nn = np.arange(0, N + 1)
    w = 0.54 - 0.46 * np.cos(2 * np.pi * nn / N)
    return lsf * w


def jfft(x, ts):
    """Internal jfft from jslantedge.m (amplitude spectrum, *2 one-sided)."""
    x = np.asarray(x, float).ravel()
    Lx = len(x)
    fftsize = int(2 ** np.ceil(np.log2(Lx)))           # 2^nextpow2
    fftx = np.fft.fft(x, fftsize) / Lx
    p = np.real(fftx * np.conj(fftx))
    half = fftsize // 2
    f = np.arange(0, half) / fftsize / ts
    p = np.sqrt(p[:half] * 2.0)
    p1 = p[0]
    p = p / p1
    return f, p, p1


def jslantedge(I, osf, pixel):
    I = np.asarray(I, float)
    BW = I
    _, out = jedgefind(I)
    s = out[0]
    b = out[1]
    const = np.sin(np.arctan(s))
    H, W = I.shape
    yp = np.arange(1, H + 1)[:, None] * np.ones((1, W))
    xp = np.ones((H, 1)) * np.arange(1, W + 1)[None, :]
    d = ((yp - b) / s - xp) * const
    dvec = d.ravel() * osf
    Ivec = I.ravel()

    dmin = np.floor(dvec.min())
    dmax = np.floor(dvec.max())
    lims = np.arange(dmin, dmax)                        # floor(min):floor(max)-1
    Llim = len(lims)
    k = int(_mround(Llim * 0.05))
    lims = lims[k - 1:Llim - k]                         # MATLAB lims(k:end-k)

    nesf = len(lims) - 1
    esf = np.zeros((nesf, 2))
    esf[:, 0] = lims[:-1]
    lim0, lim1 = lims.min(), lims.max()
    mask = (dvec > lim0) & (dvec < lim1)
    inds = np.floor(dvec[mask] - lim0).astype(int)      # 0-based bin index
    ssum = np.bincount(inds, weights=Ivec[mask], minlength=nesf)[:nesf]
    scnt = np.bincount(inds, minlength=nesf)[:nesf].astype(float)
    with np.errstate(invalid="ignore", divide="ignore"):
        col2 = ssum / scnt
    col2[scnt == 0] = 0.0                                # NaN -> 0 (as in MATLAB)
    esf[:, 1] = col2
    esf = esf[osf - 1:nesf - osf, :]                    # esf(osf:end-osf,:)

    lsf = np.gradient(esf[:, 1])
    freq, mtf, _ = jfft(japodize(esf, osf), pixel / osf)
    return freq, mtf, esf, lsf, BW, out


def _synthetic_edge(R=96, C=110, ang_deg=5.0, sigma=1.5, dark_left=True):
    """A slanted step edge with a Gaussian LSF (MTF ~ exp(-2*(pi*sigma*f)^2))."""
    from math import erf as _erf
    yy, xx = np.mgrid[1:R + 1, 1:C + 1]
    dist = xx - (C / 2 + (yy - 1) * np.tan(np.deg2rad(ang_deg)))
    verf = np.vectorize(_erf)
    v = 0.5 * (1 + verf(dist / (np.sqrt(2) * sigma)))
    if not dark_left:
        v = 1 - v
    return 50 + 900 * v


if __name__ == "__main__":
    # Smoke test: run on a synthetic edge, report MTF@Nyquist and MTF50.
    I = _synthetic_edge()
    freq, mtf, esf, lsf, BW, out = jslantedge(I, 4, 1.0)
    nyq = 0.5
    i50 = int(np.argmin(np.abs(mtf - 0.5)))
    inyq = int(np.argmin(np.abs(freq - nyq)))
    print("jslantedge OK: edge slope=%.3f intercept=%.3f" % (out[0], out[1]))
    print("  mtf len=%d  MTF@Nyquist(0.5c/px)=%.4f  MTF50~%.4f c/px" %
          (len(mtf), mtf[inyq], freq[i50]))
