"""Regression: metro_server/mtf.py (numpy jslantedge port) vs the reference MATLAB
jslantedge output. Generate the reference first with:

    matlab -batch "cd('<this dir>'); gen_ref"     # -> jslant_ref.mat

then run:

    python test_mtf_regression.py

Passes when freq/mtf/esf/lsf/out match MATLAB within TOL (they match to ~1e-12,
i.e. floating-point identical; the port is bit-faithful to the MATLAB algorithm).
"""
import os
import sys
import numpy as np
import scipy.io as sio

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))          # import mtf from metro_server/
from mtf import jslantedge

TOL = 1e-6


def main():
    ref_path = os.path.join(HERE, "jslant_ref.mat")
    if not os.path.exists(ref_path):
        print("jslant_ref.mat missing -- run gen_ref.m in MATLAB first.")
        return 2
    d = sio.loadmat(ref_path)
    Is = d["Is"].ravel(); osfs = d["osfs"].ravel(); pixels = d["pixels"].ravel()
    refs = {n: d[n + "s"].ravel() for n in ("freq", "mtf", "esf", "lsf", "out")}

    all_ok = True
    print(f"{'case':>5} {'field':>5} {'len':>10} {'max|d|':>12} {'rms':>12}  status")
    for k in range(len(Is)):
        I = np.asarray(Is[k], float)
        osf = int(np.array(osfs[k]).ravel()[0])
        pixel = float(np.array(pixels[k]).ravel()[0])
        freq, mtf, esf, lsf, BW, out = jslantedge(I, osf, pixel)
        got = {"freq": freq.ravel(), "mtf": mtf.ravel(), "esf": esf,
               "lsf": lsf.ravel(), "out": out.ravel()}
        ref = {"freq": np.asarray(refs["freq"][k], float).ravel(),
               "mtf": np.asarray(refs["mtf"][k], float).ravel(),
               "esf": np.asarray(refs["esf"][k], float),
               "lsf": np.asarray(refs["lsf"][k], float).ravel(),
               "out": np.asarray(refs["out"][k], float).ravel()}
        for name in ("out", "esf", "lsf", "freq", "mtf"):
            r, g = ref[name], got[name]
            if r.shape != g.shape:
                print(f"{k:>5} {name:>5} {str(r.size)+'/'+str(g.size):>10} "
                      f"{'SHAPE':>12} {'MISMATCH':>12}  FAIL")
                all_ok = False
                continue
            diff = np.abs(r - g)
            mx = float(np.nanmax(diff)) if diff.size else 0.0
            rms = float(np.sqrt(np.nanmean(diff ** 2))) if diff.size else 0.0
            ok = mx < TOL
            all_ok = all_ok and ok
            print(f"{k:>5} {name:>5} {g.size:>10} {mx:>12.3e} {rms:>12.3e}  "
                  f"{'ok' if ok else 'FAIL'}")
        print("")
    print("ALL PASS" if all_ok else "*** FAILURES ***")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
