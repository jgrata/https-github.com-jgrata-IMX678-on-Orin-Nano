#!/usr/bin/env bash
# =============================================================================
# install.sh -- one-shot installer for the IMX678 Jetson RAW server + web UI.
# Installs system + Python deps, the full Argus/L4T stack raw_capture links
# against, and builds the Argus raw_capture backend.
# Run from the metro_server/ directory on the target Jetson:  ./install.sh
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== IMX678 Jetson web UI installer =="
echo "   metro_server: $HERE"

# --- sanity: aarch64 Jetson ---------------------------------------------------
if [ "$(uname -m)" != "aarch64" ]; then
    echo "WARN: not aarch64 -- this package targets a Jetson (Orin Nano). Continuing anyway."
fi

MISSING=0
# ensure_pkg <probe-path-glob> <apt-package> <human description>
# Installs the apt package ONLY if the probe path is absent, so we never
# force-upgrade an already-present L4T/BSP component (which can break a
# flashed JetPack). A hard-missing dep after the install attempt is fatal.
ensure_pkg() {
    local probe="$1" pkg="$2" desc="$3"
    if compgen -G "$probe" > /dev/null 2>&1; then
        echo "  [ok]   $desc"
        return 0
    fi
    echo "  [get]  $desc  (missing $probe) -> apt install $pkg"
    sudo apt-get install -y "$pkg" || echo "  [warn] apt could not install $pkg"
    if compgen -G "$probe" > /dev/null 2>&1; then
        echo "  [ok]   $desc (installed)"
    else
        echo "  [MISS] $desc still missing ($probe)"; MISSING=1
    fi
}

# --- 1) build tools + numeric/vision stack (prebuilt aarch64) ------------------
echo "== apt: build tools + numeric/vision stack =="
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake pkg-config \
    python3 python3-pip python3-dev \
    python3-numpy python3-scipy python3-opencv python3-h5py

# --- 2) Argus / L4T multimedia stack (what raw_capture links + needs) ----------
# The CMakeLists links: nvargus_socketclient, nvbufsurface, EGL, cuda (+pthread)
# and includes: jetson_multimedia_api/argus, nvbufsurface.h, cuda.h.
# At runtime raw_capture talks to the nvargus-daemon (Argus).
echo "== Argus / L4T multimedia dependencies =="
ensure_pkg "/usr/src/jetson_multimedia_api/argus/include/Argus/Argus.h" \
           "nvidia-l4t-jetson-multimedia-api" "Jetson Multimedia API (Argus headers)"
ensure_pkg "/usr/lib/aarch64-linux-gnu/nvidia/libnvargus_socketclient.so" \
           "nvidia-l4t-camera" "nvargus socketclient lib + nvargus-daemon"
ensure_pkg "/usr/bin/nvargus-daemon" \
           "nvidia-l4t-camera" "nvargus-daemon (Argus camera service)"
ensure_pkg "/usr/lib/aarch64-linux-gnu/tegra/libnvbufsurface.so" \
           "nvidia-l4t-multimedia" "libnvbufsurface (NvBufSurface)"
ensure_pkg "/usr/lib/aarch64-linux-gnu/libEGL.so" \
           "libegl1-mesa-dev" "EGL dev lib (link target)"
# CUDA: the CMakeLists needs cuda.h + libcuda. On JetPack these come with the
# CUDA toolkit + the L4T driver; we verify rather than force a CUDA version.
if compgen -G "/usr/local/cuda*/include/cuda.h" > /dev/null 2>&1 || [ -e /usr/include/cuda.h ]; then
    echo "  [ok]   CUDA headers (cuda.h)"
else
    echo "  [MISS] cuda.h not found -- install the JetPack CUDA toolkit (e.g. 'sudo apt install cuda-toolkit')."; MISSING=1
fi
if compgen -G "/usr/lib/aarch64-linux-gnu/**/libcuda.so*" > /dev/null 2>&1 \
   || [ -e /usr/lib/aarch64-linux-gnu/nvidia/libcuda.so ] || [ -e /usr/local/cuda/lib64/libcuda.so ]; then
    echo "  [ok]   libcuda"
else
    echo "  [MISS] libcuda not found -- install the L4T CUDA driver (part of JetPack)."; MISSING=1
fi

if [ "$MISSING" -ne 0 ]; then
    echo
    echo "ERROR: one or more Argus/CUDA/EGL dependencies are still missing (see [MISS] above)."
    echo "       On a correctly-flashed JetPack these are present. If the 'nvidia-l4t-*'"
    echo "       packages came back 'not found', the L4T apt repo isn't configured (not a"
    echo "       flashed JetPack, or a bare container). Install the full SDK and re-run:"
    echo "           sudo apt install nvidia-jetpack        # or use NVIDIA SDK Manager"
    exit 1
fi

# --- 3) Python web deps not in apt --------------------------------------------
# Newer JetPack (Ubuntu 22.04, PEP 668) marks the system env 'externally managed'
# and rejects plain --user; fall back to --break-system-packages if needed.
echo "== pip: fastapi + uvicorn =="
python3 -m pip install --user -r "$HERE/requirements.txt" \
    || python3 -m pip install --user --break-system-packages -r "$HERE/requirements.txt" \
    || python3 -m pip install --break-system-packages -r "$HERE/requirements.txt"

# --- 4) build the Argus RAW capture backend -----------------------------------
echo "== building raw_capture (Argus C++) =="
cmake -S "$HERE/raw_capture" -B "$HERE/raw_capture/build"
cmake --build "$HERE/raw_capture/build" -j"$(nproc)"
# image_server.py expects the binary at raw_capture/raw_capture
cp -f "$HERE/raw_capture/build/raw_capture" "$HERE/raw_capture/raw_capture"
chmod +x "$HERE/raw_capture/raw_capture"

if [ -x "$HERE/raw_capture/raw_capture" ]; then
    echo "== OK: raw_capture built at raw_capture/raw_capture =="
else
    echo "ERROR: raw_capture did not build."; exit 1
fi

echo
echo "== install complete =="
echo "   Camera check:  ls /dev/video*   (the e-CAM86/IMX678 must be Argus-visible)"
echo "   Argus check:   systemctl status nvargus-daemon"
echo "   Launch:        ./run.sh         (starts servers + opens the UI)"
