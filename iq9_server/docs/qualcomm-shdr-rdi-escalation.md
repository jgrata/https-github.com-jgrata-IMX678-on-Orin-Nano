# IMX678 Clear HDR (SHDR) raw RDI capture — sensor is SHDR(2EXP) but IFE acquires single-RDI (is_shdr=0)

**For:** hvo (QLI 1.7) · kkufalk (QLI 2.0) · Qualcomm camera
**Date:** 2026-08-07 · **Reporter:** Metropolis (Jeremy Grata)
**Separate from** the RDI-release SMMU panic (`qualcomm-rdi-smmu-escalation.md`) — different failure, no panic here.

## TL;DR / asks
We want to RDI-capture the IMX678's **Clear HDR** (dual-conversion-gain, 2-exposure) **raw legs** for sensor
characterization (per-leg PTC/OETF/SNR, conversion-gain ratio). We have the sensor in **SHDR(2EXP)** mode
(confirmed by CamX) and set `qtiqmmfsrc vhdr=shdr-raw`, but the IFE still acquires a **single RDI port with
`is_shdr 0`**, so no buffers flow. Asks:
1. **[go/no-go for our DIY]** Will the *core* CamX (`camera.qcom.qcs9100.so`) honor an **OEM CHI usecase**
   that requests **SHDR-without-SFE** (2 RDI ports + `CAM_IFE_CTX_SHDR_EN`) on QCS9075 (which has **no SFE**)?
   Or is SHDR-RDI wired only through Qualcomm's own built-in usecases?
2. **[preferred path]** Is there a supported way on **QLI 1.7** to RDI-capture a 2-exposure SHDR sensor's
   raw legs (a CHI usecase / property / config we've missed)?
3. **[kkufalk / QLI 2.0]** We are migrating to **2.0 soon**. Does 2.0's camera stack change any of this —
   native SHDR-RDI raw support, SFE presence on QCS9075-class parts, a different CHI/usecase model, or
   different qtiqmmfsrc HDR handling? **We do not want to invest in a 1.7 CHI-override DIY if 2.0 supports
   it natively (or changes the mechanism).** If 2.0 differs, what's the recommended path there?
4. **[alternative]** Is `vhdr=shdr-yuv` (the VC-mode, ISP-processed path) expected to expose the raw legs,
   or only a merged/processed output? We need the *raw* per-leg data, not a merged frame.

## Environment
| | |
|---|---|
| SoC / board | QCS9075 / IQ-9075 EVK (**no SFE**) |
| SW | QLI 1.7 **r1.0_00114.0**, kernel `6.6.116-qli-1.7-...-gde229c16e2aa` |
| Sensor | Leopard IMX678 (`cmk_imx678`), 4-lane, Clear HDR = dual-VC (HG/LG), RAW10 |
| Capture | `qtiqmmfsrc` RAW16 bayer (RDI) + `vhdr=shdr-raw` |

## What works / what doesn't
- **Sensor SHDR mode — WORKS.** We rebuilt the sensormodule `.bin` with the Clear HDR dual-VC descriptor
  (one `<streamConfiguration>` with **two `<vc>`** 0+1, `dt=0x2B` RAW10, per-leg height 2180,
  `HDRExposureType=TWOEXPOSURE`, `capability=SHDR`; reverse-engineered from `camxsensordriver.xsd` +
  the imx766 DOL reference). CamX reads it: `capability: SHDR(2EXP), width 3856, height 2180, fps 15`.
  Deploys clean, **no reboot**.
- **qtiqmmfsrc — property applied.** `vhdr=shdr-raw` sets `QMMF_VIDEO_HDR_MODE=kSHDRRaw` (traced in
  `qmmf_source_context.cc`; the shipping plugin exposes `vhdr` enum: off / shdr-raw / shdr-yuv).
- **IFE acquire — FAILS to enter SHDR.** dmesg:
  ```
  cam_ife_hw_mgr_print_acquire_info: Acquired Single IFE[1] with [9 pix] [0 pd] [1 rdi] ports ...
  __cam_isp_ctx_acquire_hw_v2: Acquire success ... is_shdr 0 is_shdr_master 0
  __cam_isp_ctx_process_evt: Get unexpect evt:2 in acquired state
  ```
  Single RDI port, `is_shdr 0` → no buffers → no frames (tried capture height 2180 and 4360). **No SMMU
  panic / reboot** (distinct from the RDI-release issue).

## Root cause (source-grounded: open camera-kernel + chi-cdk)
- `is_shdr=1` requires the IFE-acquire `op_flags` to set **`CAM_IFE_CTX_SHDR_EN` (BIT7)** —
  `cam_isp/isp_hw_mgr/include/cam_isp_hw_mgr_intf.h:52`; consumed by `cam_isp_context.c` /
  `cam_ife_hw_mgr.c`. No SFE → the **"auto SHDR without SFE"** path (`cam_req_mgr_interface.h`).
- **The kernel only consumes that flag; it is decided by the CHI usecase resolved for the stream config.**
  For a `kBayerRDI16BIT` (raw) video track the mimas CHI selector
  (`oem/qcom/chiusecase/mimas/chxusecaseselector.cpp: DefaultMatchingUsecaseSelection`) resolves a
  single-RDI raw usecase (`UsecasePreviewRaw` → one `TFEOutputPortRDI0`) and does **not** set the SHDR
  flag — it even *strips* the `StreamConfigModeVideoHdr` operation_mode bit before matching. So the SHDR
  camera-mode never becomes an SHDR IFE acquire on the raw path.
- The camera usecases are **compiled into the prebuilt libs** (`camera.qcom.qcs9100.so`,
  `/usr/lib/hw/com.qti.chi.override.so`) — no runtime usecase XML on the board (`usecaseKvManager.xml`
  is the *audio* PAL manager, not camera). So there is no config knob to flip.

## Our DIY option (scoped) — and the question it hinges on
The CHI override + usecase topology are open in the chi-cdk (same SPF toolchain we use to build the sensor
`.bin`), so we could author a **custom SHDR-RAW usecase** (topology with **2 RDI ports** for the two VCs)
+ a selector branch, and rebuild `com.qti.chi.override.so`. Full plan: **`dcg-shdr-chi-override-scope.md`**.
This is viable **only if ask #1 is yes** — that the closed core CamX honors an OEM SHDR-without-SFE 2-RDI
usecase. If the core wires SHDR only through its own usecases, the DIY can't set the flag and this is a
Qualcomm core change. **Please answer #1 before we invest in the build.**

## Reproduction
1. Build/deploy the dual-VC sensormodule `.bin` (our `build_clearhdr.py --exp-gain 0` → `deploy.sh
   cmk_imx678_cam0_chdr_dcgcal`); CamX logs `capability: SHDR(2EXP)`.
2. Capture RAW16 with `qtiqmmfsrc ... vhdr=shdr-raw` at 3856×2180 (or 4360).
3. dmesg shows the single-RDI `is_shdr 0` acquire above; no frames delivered, no panic.

## Notes
- `shdr-raw` is documented as *line-interleaved, 2-frame* (one stream) — so if is_shdr flips, expect the
  two legs interleaved by line (we have a de-interleave demux ready).
- Serial console `ttyMSM0 @ 115200`; direct-link static IP for reboot-proof access + a checkpointed harness.
