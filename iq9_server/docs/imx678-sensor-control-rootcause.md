# IMX678 sensor control on the IQ9 (QCS9075) — root-cause map

**Date:** 2026-08-04 · **Platform:** IQ-9075 EVK, QCS9075, Qualcomm Linux 1.7-ver.1.1,
kernel 6.6.116-qli-1.7 · **Sensor:** Leopard Imaging IMX678 (STARVIS 2), cmk_imx678,
one mode 3856×2180 12-bit RGGB @30 fps.

## TL;DR

RAW/RDI capture works and is stable (snapshot), but **manual exposure and gain do not
affect the RAW output**, and **HCG (high conversion gain) is not exposed as a runtime
control**. This is *not* a bug in our qmmfsrc usage — it is where the QCS9075 CamX/IFE
stack is still immature relative to the Jetson:

- The RAW/RDI path is a passive tap of the sensor running at its **compiled static init
  register values**. CamX does **not** drive per-frame sensor 3A (exposure/gain) on the
  RDI-only usecase, so RAW is frozen at the init exposure/gain.
- The gst plugin (`qtiqmmfsrc`, open source) sends the correct manual-exposure metadata;
  the closed **CamX** binary simply doesn't forward it into the sensor node for RDI.
- The vendor "experience" demo app on the IQ9 desktop **also** exposes no sensor-level
  exposure/gain — confirming the gap is in the platform camera stack, not our path.

The only lever we own is the **static sensor init config**, which requires recompiling the
sensormodule `.bin` from XML with the QTI ParameterParser. That makes **HCG achievable as
a static build**, but not per-capture exposure/gain sweeps.

## The registers (all present in our sensor XML init sequence)

`meta-metro-mcs/recipes-multimedia/li-imx678/files/cmk_imx678_sensor.xml`, `<streamInfo>`
regSettings. Standard IMX678 addresses, currently pinned to fixed values:

| Field | Register | Init value | Meaning / effect |
|---|---|---|---|
| **SHR0** (shutter) | 0x3050–0x3052 | `0x000384` = 900 | integration = VMAX−SHR0 = 2250−900 = **1350 lines**, fixed |
| **GAIN** (analog) | 0x3070–0x3072 | `0x14` = 20 | analog gain, fixed |
| **FDG_SEL0** (conv. gain) | 0x3030 [1:0] | `0x00` = **LCG** | `0x01` → **HCG** (High Conversion Gain) |
| VMAX (frame length) | 0x3028–0x302A | `0x0008CA` = 2250 | matches `<frameLengthLines>2250` |
| HMAX (line length) | 0x302C–0x302D | `0x044C` = 1100 | matches `<lineLengthPixelClock>1100` |

Note: the `<regAddrInfo>` block (xOutput, coarseIntgTimeAddr, globalGainAddr, …) is an
all-`0x332` **stub** and is vestigial — the real programming is the regSettings above plus
the sensor customlib `.so`.

## The control path, end to end

1. **`qtiqmmfsrc`** (OPEN SOURCE — CodeLinaro `gst-plugins-qti-oss`, branch
   `imsdk.lnx.2.0.0.r2-rel`, SRCREV `dcb4b82…`). Verified in source: on `StartVideoTracks`
   it calls `initialize_camera_param()` → `recorder->SetCameraParam()` with
   `ANDROID_CONTROL_MODE=OFF`, `ANDROID_CONTROL_AE_MODE=OFF`, `ANDROID_SENSOR_EXPOSURE_TIME=exptime`
   for **any** track type incl. bayer/RDI. So the manual-exposure request *is* emitted
   correctly. (Caveats: it always also sends the QTI `iso_exp_priority` vendor tag with
   `select_priority=ISO`, and never sends `ANDROID_SENSOR_SENSITIVITY` — but AE_MODE=OFF
   should bypass AEC anyway.)
2. **QMMF recorder** (OPEN SOURCE — `le-services.git`) — passes params to CamX.
3. **CamX + chicdk** (PREBUILT, Qualcomm proprietary, `qprebuilt` from artifactory) —
   **this is where manual exposure is dropped for RDI.** The RDI-only usecase does not run
   the per-frame sensor-control/AEC node, so `SENSOR_EXPOSURE_TIME` never reaches the sensor.
   Not editable.
4. **Sensor customlib** `com.qti.sensor.cmk_imx678.so` (source is a Leopard/Qualcomm
   prebuilt in `li-imx678`) — exports `FillExposureSettings` / `CalculateExposure`. These
   *do* program SHR0/gain per frame **when CamX calls them** (i.e. the processed/AUTO path).
5. **Compiled sensor module** `com.qti.sensormodule.cmk_imx678_cam0.bin` — this is what
   CamX loads at runtime (the `/usr/share/li-imx678/*.xml` files are source-for-regen only,
   NOT read live). Per the recipe it is "compiled from LI sensor + module XMLs with the
   **QLI 1.7 ParameterParser (V5.5.1, 2411131018)**". Register changes require regenerating
   this `.bin`.

### Empirical proof exposure is not applied to RDI
2 ms vs 64 ms (32×) RAW16 snapshots are statistically identical — mean 424.6 vs 426.7,
std 360 vs 363, identical p1/50/90/99/99.9 percentiles, identical spatial quadrants — over
a real structured scene (std≈360, highlights to 4095). 20-frame bursts are flat, so it is
not a settle/frame-skip effect. Turning DMX lights to max raised the mean only ~48 counts
and did not restore any exposure response → the sensor exposure is simply not changing.

## Two distinct IFE SMMU faults (watchdog → full SoC reset)

Both are in-kernel `camera_qcs9100` IFE SMMU page faults that `qcom_wdt` converts to a hard
reset — **uncatchable in userspace** (see resilience layer below). Trace saved:
[`nv12_smmu_crash_trace.txt`](nv12_smmu_crash_trace.txt).

| Fault | Signature | Trigger | Status |
|---|---|---|---|
| Release use-after-unmap | "Cannot find vaddr in SMMU ife" in `cam_ife_mgr_release_hw` | RDI release churn | **FIXED** by camera-kernel `f7b70309` (in loaded module md5 `f08df7145219`) |
| Out-of-bounds DMA | `PF Type: faulting addr out of bounds`, `Faulted ctx NOT found`, `Context bank dump for ife`, `synchronous external abort` | **NV12 (processed IFE→IPE) snapshot with `control-mode=off` + manual-exposure** | **OPEN** — distinct code path, in prebuilt CamX/IFE |

Also unstable / not root-caused: **continuous RAW/RDI streaming** reboots (snapshot RAW is
the only stable RAW path). RAW **snapshot** with manual-exposure/iso (iso-mode=manual) is
stable — it just has no effect.

## What is achievable, and how

| Goal | Path | Effort |
|---|---|---|
| **HCG characterization** | Rebuild sensormodule `.bin` with `0x3030 = 0x01`; a second LCG build for comparison. Static per session (reboot/reload between LCG↔HCG). | Needs QTI ParameterParser V5.5.1 (build host / hvo), like the cameradlkm rebuild. |
| **Different fixed exposure/gain** | Same `.bin` rebuild with new SHR0 / 0x3070. Static only — not a sweep. | Same. |
| **Per-capture exposure/gain sweep on RDI** | Blocked in prebuilt CamX. No source lever. | Escalate to Qualcomm/hvo, or wait for a CamX fix. |
| **Characterization *now*** | **DMX light sweep** at the (fixed) init exposure; read back the actual exposure/gain from result metadata. PTC (variance-vs-mean → e⁻/DN gain, read noise), OETF/linearity all come from varying *signal*, which light provides. | Available today — `iq9_client.py lightsweep`. |

## Resilience layer (built 2026-08-04)

Because the reboots are kernel watchdog resets, resilience is PC-side detect-and-recover:
- **Board:** `iq9web.service` (systemd, `Restart=always`, enabled at boot) auto-restores the
  DAQ server after any reset. RAW gated off by default.
- **PC:** `iq9_client.py` — `resilient()` wraps each call with vanish/reboot detection,
  wait-for-recovery, soft-fail cleanup+retry; `sweep()` is checkpointed (a reboot costs one
  point, not the run); `KNOWN_UNSTABLE` allowlist refuses reboot-triggering ops unless
  `allow_unstable=True`; events logged to `iq9_events.jsonl` (the growing failure-mode map).

## Corroboration
The vendor camera "experience" app on the IQ9 Wayland desktop exposes **no** sensor-level
exposure/gain settings either — independent confirmation that manual sensor control is not
wired through the platform camera stack on QLI 1.7, not a limitation of our integration.
