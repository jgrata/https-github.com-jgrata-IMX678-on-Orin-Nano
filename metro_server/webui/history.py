"""Session history / save for the web UI.

Each saved measurement is ONE self-contained HDF5 bundle (per the storage rec --
MAT v7.3 is HDF5, so these open in MATLAB via h5read too) plus a small JSON sidecar
for fast listing without opening the HDF5:

  {id}_{kind}.h5    attrs: kind, id, timestamp, results_json, meta_json, metric
                    datasets: overlay_jpeg (uint8), raw (uint16, gzip, optional),
                              radiance (float32 linear HxWx3, gzip, optional)
  {id}_{kind}.json  {id, kind, stem, timestamp, metric, metric_label, has_raw,
                     has_radiance, summary, meta}

raw/radiance are OPTIONAL per save (a 4K uint16 frame is ~33 MB) so routine history
stays light; include them when you want a full-fidelity archive for MATLAB re-analysis.
Deletes are SOFT (moved to .trash/) -- reversible, never a hard unlink.

kind is 'colorchecker' or 'mtf'. results is the analyze() output dict.
"""
import json
import os
import time

import numpy as np
import cv2
import h5py

SESS_DIR = os.environ.get("METRO_SESS_DIR", os.path.expanduser("~/metro_sessions"))
_TRASH = os.path.join(SESS_DIR, ".trash")


def _ensure(d=SESS_DIR):
    os.makedirs(d, exist_ok=True)


def _new_id():
    """Timestamp id, uniquified within the same second (server clock -- fine here,
    this is the FastAPI process, not a workflow script)."""
    _ensure()
    base = time.strftime("%Y%m%d_%H%M%S")
    cand, n = base, 1
    existing = os.listdir(SESS_DIR)
    while any(f.startswith(cand + "_") for f in existing):
        n += 1
        cand = "%s-%d" % (base, n)
    return cand


def _metric(kind, results):
    """The one headline number used to rank/label a bundle in the list."""
    if kind == "colorchecker":
        v = results.get("dE_xval_mean")
        return None if v is None else float(v)
    if kind == "mtf":
        vals = [e["mtf50"] for e in results.get("edges", []) if e.get("mtf50") is not None]
        return float(max(vals)) if vals else None
    return None


def _metric_label(kind):
    return {"colorchecker": "ΔE00 xval", "mtf": "MTF50 peak"}.get(kind, "")


def _summary(kind, results):
    """Compact fields shown on the history card (no big arrays)."""
    if kind == "colorchecker":
        if not results.get("detected"):
            return {"detected": False}
        return {
            "detected": True,
            "dE_xval_mean": results.get("dE_xval_mean"),
            "dE_vendor_mean": results.get("dE_vendor_mean"),
            "dE_rootpoly_xval_mean": results.get("dE_rootpoly_xval_mean"),
            "rootpoly_degree": results.get("rootpoly_degree"),
            "derived_beats_vendor": results.get("derived_beats_vendor"),
            "illuminant": results.get("illuminant"),
            "white_black_ratio": results.get("white_black_ratio"),
            "clip_frac": results.get("clip_frac"),
        }
    if kind == "mtf":
        return {
            "n_squares": results.get("n_squares"),
            "n_edges": results.get("n_edges"),
            "units": results.get("units"),
            "nyquist": results.get("nyquist"),
            "chan": results.get("chan"),
            "clip_frac": results.get("clip_frac"),
        }
    return {}


def save_bundle(kind, results, overlay_jpeg=None, raw=None, maxv=None,
                radiance=None, meta=None):
    """Write one HDF5 bundle + JSON sidecar; return the bundle id.
    overlay_jpeg: bytes. raw: uint16 HxW Bayer (optional). radiance: float32
    HxWx3 linear (optional). meta: capture metadata dict (exposure, gain, ...)."""
    _ensure()
    bid = _new_id()
    stem = "%s_%s" % (bid, kind)
    h5path = os.path.join(SESS_DIR, stem + ".h5")
    meta = dict(meta or {})
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    metric = _metric(kind, results)
    results_slim = {k: v for k, v in results.items() if k != "overlay_png"}  # overlay stored as a dataset
    try:
        with h5py.File(h5path, "w") as f:
            f.attrs["kind"] = kind
            f.attrs["id"] = bid
            f.attrs["timestamp"] = ts
            f.attrs["results_json"] = json.dumps(results_slim)
            f.attrs["meta_json"] = json.dumps(meta)
            if metric is not None:
                f.attrs["metric"] = metric
            if overlay_jpeg is not None:
                f.create_dataset("overlay_jpeg", data=np.frombuffer(overlay_jpeg, np.uint8))
            if raw is not None:
                d = f.create_dataset("raw", data=np.asarray(raw, np.uint16),
                                     compression="gzip", compression_opts=4)
                if maxv is not None:
                    d.attrs["maxv"] = float(maxv)
            if radiance is not None:
                f.create_dataset("radiance", data=np.asarray(radiance, np.float32),
                                 compression="gzip", compression_opts=4)
        side = {"id": bid, "kind": kind, "stem": stem, "timestamp": ts,
                "metric": metric, "metric_label": _metric_label(kind),
                "has_raw": raw is not None, "has_radiance": radiance is not None,
                "meta": meta, "summary": _summary(kind, results)}
        with open(os.path.join(SESS_DIR, stem + ".json"), "w") as jf:
            json.dump(side, jf)
    except Exception:
        for p in (h5path, os.path.join(SESS_DIR, stem + ".json")):   # don't leave a half-written bundle
            try:
                os.remove(p)
            except OSError:
                pass
        raise
    return bid


def list_bundles():
    """All bundles, newest first, from the JSON sidecars (cheap -- no HDF5 open)."""
    _ensure()
    out = []
    for fn in os.listdir(SESS_DIR):
        if not fn.endswith(".json"):
            continue
        try:
            with open(os.path.join(SESS_DIR, fn)) as f:
                out.append(json.load(f))
        except Exception:
            continue
    out.sort(key=lambda b: b.get("id", ""), reverse=True)
    return out


def _stem_path(stem):
    # guard against path traversal -- stem is a basename only
    stem = os.path.basename(stem)
    return os.path.join(SESS_DIR, stem + ".h5"), os.path.join(SESS_DIR, stem + ".json")


def get_bundle(stem):
    """Full detail for one bundle: results dict, meta, and the overlay re-encoded
    as a data: URI for display. Does NOT return raw/radiance (download the .h5)."""
    h5path, _ = _stem_path(stem)
    if not os.path.exists(h5path):
        return None
    import base64
    with h5py.File(h5path, "r") as f:
        res = json.loads(f.attrs.get("results_json", "{}"))
        meta = json.loads(f.attrs.get("meta_json", "{}"))
        out = {"kind": f.attrs.get("kind"), "id": f.attrs.get("id"),
               "timestamp": f.attrs.get("timestamp"), "results": res, "meta": meta,
               "has_raw": "raw" in f, "has_radiance": "radiance" in f}
        if "overlay_jpeg" in f:
            b = np.asarray(f["overlay_jpeg"]).tobytes()
            out["overlay"] = "data:image/jpeg;base64," + base64.b64encode(b).decode()
    return out


def overlay_jpeg(stem):
    """Raw overlay JPEG bytes for a thumbnail, or None."""
    h5path, _ = _stem_path(stem)
    if not os.path.exists(h5path):
        return None
    with h5py.File(h5path, "r") as f:
        if "overlay_jpeg" not in f:
            return None
        return np.asarray(f["overlay_jpeg"]).tobytes()


def download_path(stem):
    h5path, _ = _stem_path(stem)
    return h5path if os.path.exists(h5path) else None


def delete_bundle(stem):
    """Soft delete: move the .h5 + .json into .trash/ (reversible)."""
    _ensure(_TRASH)
    h5path, jpath = _stem_path(stem)
    moved = False
    for p in (h5path, jpath):
        if os.path.exists(p):
            os.replace(p, os.path.join(_TRASH, os.path.basename(p)))
            moved = True
    return moved
