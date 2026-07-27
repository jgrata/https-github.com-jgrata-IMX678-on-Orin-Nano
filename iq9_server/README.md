# IMX678 on IQ9

Port of the RAW-DAQ + HDR + image-quality tooling from the Jetson/Argus rig to the
**Qualcomm QCS9075 (IQ-9075 EVK)**. Companion to `../metro_server/` (the Jetson
implementation); the science/UI code is **shared**, only the capture backend and
transport are platform-specific (structure "A" — monorepo, DRY).

## Platform (probed 2026-07-24)

| | |
|---|---|
| SoC / board | Qualcomm **QCS9075**, IQ-9075 EVK, 8× aarch64 |
| OS | **Qualcomm Linux 1.7** (Yocto/QLI), kernel 6.6.116 |
| Sensor | **IMX678** via Leopard Imaging (`li-imx678` pkg); CamX sensor mode **3856×2180, 12-bit, 30 fps** |
| Camera stack | CAMSS/CamX + `cam-server` (owns V4L2); access via **GStreamer `qtiqmmfsrc`** (QMMF — the analog of Tegra `nvarguscamerasrc`) |
| HW codec | Venus `msm_vidc` (/dev/video32-33) — H.264/H.265 encode for the remote path |
| Python/libs | Python 3.12, numpy 1.26, **cv2 4.11** (newer than Jetson's 4.5.4), gi/Gst 1.22 |
| Network | `eth0` = **10.70.0.60** wired, currently **1 GbE** (2.5/10 GbE planned); `wlan0` = 10.70.1.14 |

## Capture: `qtiqmmfsrc` (see `camera_qmmf.py`)

`qtiqmmfsrc` exposes, from ONE source, up to 120 fps:
- **`video/x-bayer`** (rggb/bggr/…) — RAW Bayer, ISP-bypassed.
- **`video/x-raw`** NV12/NV16/RGB — ISP-processed.
- Multiple `video_%u` pads → **raw + processed simultaneously** (the "Option B" we
  couldn't cleanly do on Tegra is native here — no daemon handoff, no SEGV).

**Validated:** NV12 processed → BGR via `qtiqmmfsrc ! …NV12 ! videoconvert ! BGRx !
appsink` in Python (`QmmfCapture(mode="nv12")` returns a real 1080p BGR frame).

**Open (#1 risk): RAW Bayer is not accessible via `qtiqmmfsrc` on this build.**
The color/CCM/MTF science needs *linear RAW*, but every standard path failed
(investigated 2026-07-24):
- bayer on the **video pad** → links, then QMMF **`StartVideoTracks Failed`**.
- **`bpp=12`** (per the newer Qualcomm docs, doc 80-70014-50) → caps don't link;
  this build's template has no `bpp` field.
- bayer on the **image pad** (snapshot) → links alone, but `capture-image` returns
  **False / no frame** (needs a running preview; the preview+snapshot combo hits a
  negotiation quirk at full res).
- **Qualcomm's own examples** (`/usr/bin/gst-camera-*-example`,
  `gst-camera-opencv-resize.py`) use **only NV12** — none demonstrate bayer/snapshot.
- **libcamera** sees no cameras (`cam -l` empty; cam-server owns them).

**Leading option:** CAMSS **V4L2-direct** — stop `cam-server`, wire the media graph
with `media-ctl` (sensor→CSIPHY→CSID→IFE→V4L2 capture node), grab RAW over V4L2,
bypassing QMMF/CamX. Likely **mutually exclusive with `qtiqmmfsrc`** (same
raw-vs-processed exclusivity as the Jetson's Argus-vs-raw_capture). Other options:
a QMMF-SDK/CamX custom RAW stream config, or Qualcomm support. Needs platform
expertise to settle.

Canonical idiom (from the examples): `qtiqmmfsrc name=camsrc video_0::type=video !
video/x-raw,format=NV12,… ! … ! appsink`. Gotcha: `qtiqmmfsrc` can't be created
twice per process (`qmmfsrc_init` asserts) — one capture per process.

## Structure (A — shared monorepo)

```
iq9_server/
  camera_qmmf.py   # capture shim: qtiqmmfsrc appsink -> numpy (NV12 done; bayer WIP)
  server.py        # (next) FastAPI webui — reuses ../metro_server/webui portable modules
  README.md
```
Portable modules reused unchanged from `../metro_server/webui/`: `colorchecker.py`,
`mtf.py`/`mtf_analyze.py`, `imaging.py`, `darkcheck.py`, `history.py`, and the
`static/` pages; plus the MATLAB `ColorAnalysis.m`/GUI. A ΔE or MTF fix applies to
both platforms.

## Next steps

1. **Crack RAW Bayer** out of `qtiqmmfsrc` (blocks the color path). Fallback options
   to evaluate: libcamera (also present, `libcamera.so.0.4`), or a QMMF/CamX stream
   config for a RAW output.
2. Stand up `server.py` reusing the portable webui; wire `QmmfCapture` as the source
   (MTF focus + live view can run on NV12 today).
3. Zero-copy local path (GStreamer DMA buffers / appsink without copy) + Venus HW
   encode for the remote path once 2.5/10 GbE lands.
