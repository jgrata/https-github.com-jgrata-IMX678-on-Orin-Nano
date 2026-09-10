# IMX678 Jetson RAW server + web UI — deployment

Bring up the IMX678 (e-CAM86_CUONX) RAW-DAQ + image-quality **web UI** on a Jetson
(Orin Nano). Three tiers, all on the Jetson:

```
  raw_capture (Argus C++)  --:9001-->  image_server.py  --:9000-->  webui/server.py  --:8080-->  browser
   RAW off the sensor                    RAW/DAQ backend            FastAPI web UI
   (auto-spawned)                        (also serves MATLAB)
```

- **`raw_capture/`** — Argus/CUDA C++ that pulls linear RAW off the sensor. Built by `install.sh`.
- **`image_server.py`** — spawns `raw_capture --server` (localhost :9001), serves the DAQ protocol on :9000. Restarts `nvargus-daemon` on start.
- **`webui/server.py`** — FastAPI/uvicorn UI on :8080; talks to `image_server` at 127.0.0.1:9000. Live view, exposure/gain, histogram, ColorChecker, MTF, capture history.

## Prerequisites (on the Jetson)

- **JetPack / L4T** flashed, **aarch64**. The installer pulls the Argus/L4T stack `raw_capture` links against (`nvidia-l4t-jetson-multimedia-api`, `nvidia-l4t-camera` = nvargus-daemon, `nvidia-l4t-multimedia` = NvBufSurface, EGL) plus **CUDA** (toolkit `cuda.h` + L4T `libcuda`), but these are normally already present on a flashed JetPack — the installer only adds what's missing so it won't force-upgrade your BSP. The `nvidia-l4t-*` packages come from JetPack's L4T apt repo; if they report "not found" you're not on a flashed JetPack (or the repo/container lacks it) — install the full SDK with `sudo apt install nvidia-jetpack` (or NVIDIA SDK Manager). On JetPack 6 (Ubuntu 22.04) pip is "externally managed" (PEP 668); `install.sh` falls back to `--break-system-packages` automatically.
- **e-con e-CAM86_CUONX driver/overlay** installed per e-con's release for your JetPack, so the IMX678 is **Argus-visible**. Verify: `ls /dev/video*` shows the camera and `systemctl status nvargus-daemon` is active. (A quick Argus smoke test: `nvgstcapture-1.0` or the `argus_camera` sample.)
- **Passwordless `sudo` for systemctl** (or run the launch with sudo once): `image_server.py` and `run.sh` restart `nvargus-daemon`. Dev Jetsons usually allow this; otherwise add a sudoers rule for `systemctl restart nvargus-daemon`.
- Python 3 (JetPack default). `python3 -m pip` available.

## Install

```bash
git clone https://github.com/jgrata/https-github.com-jgrata-IMX678-on-Orin-Nano.git
cd https-github.com-jgrata-IMX678-on-Orin-Nano       # master branch = the Jetson build
cd metro_server
./install.sh
```

`install.sh` will:
1. `apt` the build tools + prebuilt aarch64 `numpy/scipy/opencv/h5py`.
2. Verify + install the **Argus/L4T/CUDA/EGL** dependencies (only if missing).
3. `pip install --user fastapi uvicorn`.
4. Build `raw_capture` (CMake → Argus) into `raw_capture/raw_capture`.

If any Argus/CUDA/EGL dep can't be satisfied it stops with a `[MISS]` line naming
the JetPack component to install.

## Run

```bash
./run.sh
```

Starts `image_server` (:9000, which auto-spawns `raw_capture` on :9001), then the
web UI (:8080), then opens `http://localhost:8080` in the Jetson's browser.
`Ctrl+C` stops everything.

- **Headless / over SSH:** `NO_BROWSER=1 ./run.sh` — then browse to `http://<jetson-ip>:8080` from your laptop.
- **Different ports:** `PORT=8000 IMG_PORT=9000 ./run.sh`.
- **Stop from another shell:** `./stop.sh`.
- **Logs:** `~/metro_sessions/logs/{image_server,webui}.log`.

## Ports & data

| Port | Process | Purpose |
|------|---------|---------|
| 8080 | `webui/server.py` (uvicorn) | Web UI (browser) |
| 9000 | `image_server.py` | RAW/DAQ backend (web UI **and** MATLAB client) |
| 9001 | `raw_capture` (localhost) | Argus RAW frames → image_server |

Captures/config live under **`~/metro_sessions/`** (`METRO_CONFIG_DIR` to relocate); HDF5 captures + `mtf_defaults.json` are written there.

## Autostart at boot (optional)

```bash
sudo cp metro-webui.service /etc/systemd/system/   # edit User= and paths first
sudo systemctl daemon-reload
sudo systemctl enable --now metro-webui
journalctl -u metro-webui -f
```

## Troubleshooting

- **UI loads but no image / "camera" errors** → check `~/metro_sessions/logs/image_server.log`. Usually `nvargus-daemon` isn't up or the sensor isn't Argus-visible (`ls /dev/video*`, `systemctl status nvargus-daemon`). Restart: `sudo systemctl restart nvargus-daemon`.
- **`raw_capture` build fails** → an Argus/CUDA/EGL dep is missing; re-run `./install.sh` and read the `[MISS]` lines, or install that JetPack component.
- **`Address already in use`** → a previous run is still up: `./stop.sh` then re-run.
- **First frame is slow** → Argus init takes ~7 s on first capture; the launcher waits up to 40 s for :9000.
- **Permission prompt on start** → the `nvargus-daemon` restart needs sudo (see Prerequisites).

## Static ethernet IP for the DAQ link (optional)

The host talks to the board over a direct cable or a switch. To pin the board's
wired NIC to a fixed address on that link (no gateway, won't touch your normal
internet routing):

```bash
sudo ./set-daq-ip.sh 192.168.99.3        # /24, auto-detects the wired NIC
sudo ./set-daq-ip.sh --dhcp              # revert to DHCP
```

Then browse `http://192.168.99.3:8080`. Pick an address that's free on your DAQ
subnet (avoid collisions with other boards on the same switch).
