> ⚠️ **SUPERSEDED / DO NOT SEND — 2026-07-30.** This case's root cause is **wrong**. RAW (RDI)
> capture **works today** on this device with no CamX/CHI change. The request never reaches the
> CHI usecase selector: it is rejected earlier at CamX `configure_streams` → `CheckValidStreamConfig`
> because the plugin defaulted to **RAW10** (HAL fmt 37), whose advertised max is **3840×2160** —
> smaller than the sensor's 3856×2180. Requesting **RAW16** at native resolution validates and
> streams verified 12-bit RGGB bayer:
> ```
> qtiqmmfsrc ! video/x-bayer,format=rggb,bpp=(string)16,width=3856,height=2180,framerate=30/1 ! identity eos-after=N ! filesink location=...
> ```
> See `raw-enablement.md` for the corrected analysis. Kept only for history — **do not act on the request below.**

# Support request: enable RAW (RDI) capture for a custom sensor on QCS9075

**Summary.** On QCS9075 (IQ-9075 EVK) with Qualcomm Linux, a **preview (NV12) + RAW-Bayer** capture request via
`qtiqmmfsrc` fails at `StartVideoTracks`. ISP-processed (NV12) capture works. Root cause is in the CHI usecase
selector: `UsecaseSelector::GetMatchingUsecase()` has **no branch that selects a preview+RAW usecase** for a
`{NV12, RAW}` (no-JPEG) stream configuration, so the request falls through to `UsecaseId::Default`, whose
pipeline produces no RAW output. The `UsecasePreviewRaw` usecase exists in the topology but is never selected.
We request a `chicdk` change (below) to route this configuration to `UsecasePreviewRaw`, delivered as an updated
prebuilt for QCS9075 (or guidance to build CamX/CHI from source for this target).

---

## Environment

| | |
|---|---|
| SoC / board | QCS9075, IQ-9075 EVK |
| OS | Qualcomm Linux 1.7-ver.1.1, kernel 6.6.116 |
| Software | LE.QCLINUX.1.0.r1 / QIM Product SDK `qimpsdk-r1.0_00114.0` |
| Camera stack | CamX + `cam-server` + `qtiqmmfsrc` (`qcom-gstreamer1.0-plugins-oss-qmmfsrc 1.0`) |
| CamX / CHI delivery | `chicdk`, `camx`, `camxcommon`, `camxlib`, `camxapi`, `camx-autogen` shipped as **prebuilt binaries** (`qprebuilt` recipes); no source build available to us |
| Sensor | Custom Leopard Imaging LI-IMX678-FLEX-114H (Sony IMX678), driver `cmk_imx678`; one mode: **3856×2180, 12-bit, RGGB, 30 fps, NORMAL/SDR** (MIPI CSI-2 4-lane, `dt=0x2C`) |

*(Please confirm the exact SPF build ID against your records; values above are from the on-device image and the QIM PSD bundle we were given.)*

---

## Observed behavior

**NV12 (ISP-processed) capture — WORKS:**
```bash
gst-launch-1.0 -e qtiqmmfsrc camera=0 ! \
  "video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1" ! fakesink
# PREROLLED -> PLAYING, frames flow.
```

**RAW-Bayer capture — FAILS** (single RAW stream, and also preview+RAW two-stream):
```bash
gst-launch-1.0 -e qtiqmmfsrc ! \
  "video/x-bayer,format=rggb,width=3856,height=2180,framerate=30/1" ! fakesink
```
```
ERROR qtiqmmfsrc qmmf_source_context.cc:2041 gst_qmmf_context_start_video_streams:
      QMMF Recorder StartVideoTracks Failed!
ERROR qtiqmmfsrc qmmf_source.c:1101 qmmfsrc_start_stream: Stream start failed!
```
Notes:
- `qtiqmmfsrc validate_bayer_params` accepts **only 3856×2180** (the sensor's RAW readout); other sizes are
  rejected with `Invalid … bayer resolution!` — so the size above is correct and passes validation; the failure
  is at track/usecase start, not caps.
- The two-video-pad request (`video_0` NV12 + `video_1` bayer @3856×2180), which should match
  `UsecasePreviewRaw`, links and reaches PLAYING but hits the **same** `StartVideoTracks Failed`.
- The RAW+JPEG combination (which *would* select `RawJPEG`) cannot be formed through `qtiqmmfsrc` — it refuses
  to link an `image/jpeg` pad alongside a `video/x-bayer` pad.

---

## Root-cause analysis

Source examined: `chi-cdk/core/chiusecase/chxusecaseutils.cpp`, `UsecaseSelector::GetMatchingUsecase()`
(compiled into `com.qti.chiusecaseselector.so`; QMMF/`qtiqmmfsrc` drive the CHI HAL underneath, so this selector
runs for the session).

For a 2-stream configuration (`case 2:`, ≈ line 2119) the selector routes:
- `IsRawJPEGStreamConfig()` (RAW **and** JPEG present) → `UsecaseId::RawJPEG`
- else `IsPreviewZSLStreamConfig()` → `UsecaseId::PreviewZSL`
- else MFNR / GPU / `Default`

RAW is only ever routed when paired with **JPEG** (`RawJPEG`) or in **XCFA/HEIC** snapshot configs. **There is
no branch that selects a usecase for a preview(NV12) + RAW-Bayer (Raw10/Raw16), no-JPEG configuration.** Such a
request therefore resolves to `UsecaseId::Default`, whose pipeline has no `TARGET_BUFFER_RAW` output — so the
RAW track cannot be created and `StartVideoTracks` fails.

The corresponding usecase already exists in the topology but is unreachable from the selector:
`camxUsecasePreviewRaw.xml` → `UsecasePreviewRaw`:
```
TARGET_BUFFER_PREVIEW  ChiFormatYUV420NV12      (≤ 1920×1080)
TARGET_BUFFER_RAW      ChiFormatRawMIPI / ChiFormatRawPlain16   (≤ 5488×4112)
CamxInclude pipeline="PreviewRaw"
```
`GetMatchingUsecase()` never assigns `UsecaseId::PreviewRaw` (no code path to it).

---

## Requested change

Add a selector branch (mirroring the existing `RawJPEG` branch) in
`UsecaseSelector::GetMatchingUsecase()`, `chxusecaseutils.cpp`, so a preview+RAW (no-JPEG) configuration selects
`UsecasePreviewRaw`. Sketch:

```cpp
// case 2: (and case 3: with a preview) — after the IsRawJPEGStreamConfig() check:
if ( <one stream is NV12 preview> &&
     <one stream IsRawStream() (ChiStreamFormatRaw10 / Raw16)> &&
     <no JPEG/BLOB stream> )
{
    usecaseId = UsecaseId::PreviewRaw;   // maps to camxUsecasePreviewRaw.xml
    break;
}
```
Helpers already present in the file: `IsRawStream()` (≈ line 441), `IsRawJPEGStreamConfig()` (≈ line 975),
`GetSnapshotStreamConfiguration()`. Please also confirm/ensure:
1. `UsecaseId::PreviewRaw` exists and is wired to `camxUsecasePreviewRaw.xml` in the usecase factory/enum.
2. The `PreviewRaw` pipeline is built for the QCS9075 (mimas) topology.
3. The RAW target format accepted matches what `qtiqmmfsrc` requests for a `video/x-bayer,format=rggb` stream
   at 12-bit (`ChiFormatRawMIPI` / `ChiFormatRawPlain16`).

---

## What we need from Qualcomm

Because `camx`/`chicdk` are delivered to us as **prebuilt binaries** (`qprebuilt`) and we have no CamX/CHI
source-build for this target, please provide **one** of:

1. An updated **`chicdk` prebuilt** for QCS9075 (LE.QCLINUX.1.0 / matching our SPF) containing the selector
   change above — ideal, drop-in for us; **or**
2. The **supported procedure / build environment to compile CamX/CHI from source** for QCS9075, so we can apply
   and build the change ourselves; **or**
3. The **correct existing mechanism** to obtain a preview + linear-RAW (RDI) stream on this platform if we are
   requesting it incorrectly (e.g., a required stream/format/usecase combination that `qtiqmmfsrc` should use).

Use case: we need **linear RAW Bayer** for sensor characterization and color-matrix work (the ISP-processed NV12
path is already working and colour-corrected). A RAW snapshot (not necessarily full-rate streaming) is
sufficient.

---

## Reference

- Selector: `chi-cdk/core/chiusecase/chxusecaseutils.cpp` → `UsecaseSelector::GetMatchingUsecase()`
- Usecase XML: `chi-cdk/api/.../topology/mimas/.../UsecasePreviewRaw/camxUsecasePreviewRaw.xml`
- Plugin error origin: `qmmf_source_context.cc:2041` (`gst_qmmf_context_start_video_streams`)
- Sensor: `cmk_imx678` (Leopard Imaging LI-IMX678-FLEX-114H), 3856×2180 / 12-bit / RGGB / 30 fps
