# RAW capture on IQ9 (IMX678) — findings & enablement

**Platform:** QCS9075 IQ-9075 EVK · Qualcomm Linux 1.7 · IMX678 (LI-IMX678-FLEX-114H) on JCAM0–3 ·
camera stack = CAMSS/CamX + `cam-server` + GStreamer `qtiqmmfsrc`.

## TL;DR

The **NV12 (ISP-processed) path works** and the IQ9 web UI runs on it (live view, MTF focus, vendor-ISP
colour eval). **Linear RAW Bayer does not come out of `qtiqmmfsrc`.** The CamX **RAW usecases exist**
(`UsecasePreviewRaw`, `UsecaseRaw`, …), but the RDI/RAW output stream is **not wired to the `cmk_imx678`
sensor**, so track creation fails. Fixing this is a **CamX camera-bring-up task** (sensor RDI capability +
usecase-to-sensor selection + rebuild the CamX bins) — the layer that produced `com.qti.sensor.cmk_imx678.so`
and the module bins. The QMMF-SDK C++ route is a fallback and needs a package that is **not in the qimpsdk
source bundle** (see the QMMF ask at the end).

Once `qtiqmmfsrc` delivers bayer at **3856×2180**, wiring RAW into the web UI + the full colour-science
(derived-CCM / dark-integrity / HDR, already written and Jetson-validated) is quick on our side.

## Evidence (all reproduced on the target)

| Test | Result |
|---|---|
| `qtiqmmfsrc … video/x-bayer,format=rggb,width=3840,height=2160` | `validate_bayer_params: Invalid 3840x2160 bayer resolution!` — rejected |
| same at **3856×2180** (the sensor's RAW readout) | passes `validate_bayer_params`, then **`QMMF Recorder StartVideoTracks Failed`** (`qmmf_source_context.cc:2041`) |
| two-pad **NV12 preview + bayer** (should match `UsecasePreviewRaw`) | same `StartVideoTracks Failed` — no RAW frame |
| bayer on the **image pad** + `capture-image` | returns **False** / no frame |
| `bpp=12` in caps (per newer Qualcomm docs) | caps don't link — this build's `qtiqmmfsrc` has no `bpp` field |
| `qmmf_recorder_gtest --gtest_filter=*RawStream*` | test exists, symbols `RAW10/RAW12/RAW16`; fails in 14 ms because it wants **3 concurrent cameras** (EVK has 1) |
| `libcamera` (`cam -l`) | **no cameras** — `cam-server` owns them; not a path |
| NV12 preview/video/JPEG/H264 | **all work** (matches LI's own `li-imx678` verify steps) |

Only the sensor's exact RAW size **3856×2180, 12-bit** passes the plugin's bayer validation
(`colorFilterArrangement = BAYER_RGGB`, from `cmk_imx678_sensor.xml`).

## CamX has the RAW usecases (from CHI-CDK)

From the CHI-CDK (`…/chi-cdk/api/oem/qcom/topology/mimas/…/usecases/`):

- **`camxUsecasePreviewRaw.xml`** → `UsecasePreviewRaw`:
  - `TARGET_BUFFER_PREVIEW` → `ChiFormatYUV420NV12`
  - `TARGET_BUFFER_RAW` → **`ChiFormatRawMIPI` / `ChiFormatRawPlain16`**  (pipeline `PreviewRaw`)
- also `UsecaseRaw`, `UsecaseRaw8`, `RawSnapshot`, `QuadCFA`, and `RealtimeRdi` pipelines.

So the RAW plumbing is designed in; it just isn't reachable for this sensor.

## Root cause

`StartVideoTracks Failed` is a **QMMF-recorder → CamX** rejection, not a GStreamer-caps problem. The RAW
usecase XMLs exist but the **IMX678's RDI/RAW output stream is not enabled / not selected** for it. i.e. the
sensor→RDI capability and/or the usecase-to-sensor mapping for `cmk_imx678` is missing from the CamX config.

## Path A — enable the RDI/RAW usecase for `cmk_imx678` (recommended; this bundle supports it)

This is the same flow that produced the sensor `.so` + module bins. Concretely:

1. Ensure `cmk_imx678_sensor.xml` advertises an **RDI/RAW output stream** (stream types listed include
   `BAYER_RGGB`; confirm the RAW/RDI stream config is present and the mode 3856×2180/12-bit maps to it).
2. Confirm the **`mimas` topology** (the QCS9075 topology in CHI-CDK) is the one this target uses, and that
   `UsecasePreviewRaw` / `UsecaseRaw` are built into the shipped usecase set for it.
3. Make the **usecase selector** map a preview+raw (or raw-only) stream request to `UsecasePreviewRaw` /
   `UsecaseRaw` for this sensor.
4. **Rebuild the CamX bins** with `ParameterParser` (as for the module/sensor/socid bins) and redeploy under
   `/usr/lib/camera/`.
5. Verify: `qtiqmmfsrc … video/x-bayer,format=rggb,width=3856,height=2180,framerate=30/1 ! appsink` should
   then deliver frames (RawPlain16 → 16-bit, or RawMIPI → 10/12-bit packed).

Cross-check with the **OSS `qmmfsrc` plugin source** (CodeLinaro / `qualcomm-linux`): `qmmf_source_context.cc`
→ `validate_bayer_params` (why only 3856×2180) and `gst_qmmf_context_create_video_stream` /
`gst_qmmf_context_start_video_streams` (the exact QMMF track params it sends — shows what CamX must accept).

## Path B — direct QMMF-recorder app (fallback; needs the QMMF SDK)

Bypass `qtiqmmfsrc`: a small C++ app on the QMMF **recorder** API creating a RAW track directly (this is what
`qmmf_recorder_gtest`'s RawStream test does). **Blocker:** the QMMF SDK (recorder headers) is **not in the
qimpsdk source bundle** (its `vendor/qcom/proprietary/` has only `chi-cdk*`, `iot-core-algs-ship`,
`qualcomm-profiler`, `thermal-engine`), and no `-dev`/headers are on the target. See the ask below.

## Once RAW delivers — our side (fast)

- `iq9_server/camera_qmmf.py`: add a `mode="bayer"` path (RawPlain16 → `uint16` HxW; RawMIPI → unpack) at
  3856×2180.
- `server.py`: `/api/colorchecker` switches to `colorchecker.analyze()` (derived-CCM from linear RAW) instead
  of `analyze_processed` (vendor eval); add `/api/darkcheck`, HDR bracket capture.
- All the RAW colour-science (CIEDE2000, k-fold CCM, root-poly, dark-integrity) already exists and is
  Sharma-validated — it drops straight in via the shared modules.

---

## The specific QMMF ask (forwardable)

> **We need the Qualcomm QMMF SDK (recorder API) for QCS9100.LE.1.0 — headers + source — to build a
> RAW-capture app against the on-target QMMF runtime.**
>
> On the target the QMMF runtime libraries are present and owned by **`qcom-camera-server`**
> (`/usr/lib/libqmmf_camera_adaptor.so`, `libqmmf_camera_metadata.so`, `libqmmf_memory_interface.so`,
> `libqmmf_utils.so`), and the QMMF test binary `qmmf_recorder_gtest` is installed — **but no QMMF headers,
> no `qmmf-sdk` / `qcom-camera-server-dev` package in the feeds, and no QMMF source in the QIM Product SDK
> bundle** (`…-qimpsdk-r1.0_00114.0`, whose `vendor/qcom/proprietary/` ships only `chi-cdk*`).
>
> Specifically requesting, from the **base QCS9100.LE.1.0 platform release** (not the QIM product add-on):
> 1. The **QMMF-SDK recorder headers** — the `qmmf::recorder` API: `qmmf_recorder.h` / `recorder.h`,
>    `qmmf_recorder_params.h`, `qmmf_camera_metadata.h`, `qmmf_buffer.h` (and whatever `qcom-camera-server`
>    was built against), plus a `-dev` package so they land on the target.
> 2. The **`qmmf_recorder_gtest` source** (or an equivalent RAW-track sample) showing how a RAW/RDI video
>    track is configured — its `*RawStream*` test is the reference.
> 3. If simpler: the **CamX sensor/usecase config + build steps to enable the RDI/RAW output for a custom
>    sensor** on this platform (so `qtiqmmfsrc video/x-bayer` works without a bespoke app) — Path A above.
>
> Delivery via the Software Center (QCS9100.LE.1.0) or as a Yocto recipe (`qmmf-sdk` / a `-dev` added to the
> image, like the other `-dev` packages we already stage) both work.
