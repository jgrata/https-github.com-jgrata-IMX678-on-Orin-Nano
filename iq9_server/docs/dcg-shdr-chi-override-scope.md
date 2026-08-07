# DIY scope: force SHDR (is_shdr=1) raw RDI capture via a custom CHI override

**Goal:** make CamX acquire the IFE in SHDR mode (`is_shdr=1`, dual-RDI) for a raw Bayer capture of
the IMX678 Clear-HDR mode, so the two exposure/gain legs actually stream over RDI — by authoring a
custom SHDR-RAW usecase + rebuilding `com.qti.chi.override.so`. This is the "own the CHI usecase
layer" path, parallel to how we already own the sensor `.bin`.

Context: sensor `.bin` ✓ (CamX reads `SHDR(2EXP)`), `qtiqmmfsrc vhdr=shdr-raw` ✓ (sets
`QMMF_VIDEO_HDR_MODE=kSHDRRaw`), but the IFE still acquires **single-RDI `is_shdr 0`** and no frames
flow. Root cause (source-traced): `is_shdr=1` needs the IFE acquire `op_flags` to set
`CAM_IFE_CTX_SHDR_EN` (BIT7, `cam_isp/isp_hw_mgr/include/cam_isp_hw_mgr_intf.h:52`), which is decided
by the CHI usecase resolver — not the open kernel or qtiqmmfsrc. See `imx678-iq9-dcg-pipeline` memory.

## The selection chain (where each decision lives)
```
qtiqmmfsrc vhdr=shdr-raw
  -> QMMF recorder  QMMF_VIDEO_HDR_MODE = kSHDRRaw          (camera-level xtraparam; OPEN)
  -> CamX HAL stream config  (operation_mode, num_streams)  (recorder->HAL; partly closed)
  -> CHI usecase SELECTOR    DefaultMatchingUsecaseSelection (OPEN, buildable)
        oem/qcom/chiusecase/mimas/chxusecaseselector.cpp
        - picks a usecase from the compiled map by operation_mode/num_streams
        - line ~324: STRIPS the StreamConfigModeVideoHdr bit before matching (for EIS)
        - raw path selects `UsecaseRawId`
  -> usecase TOPOLOGY pipeline (OPEN, buildable, XML)
        api/oem/qcom/topology/mimas/usecase-components/usecases/UsecasePreviewRaw/
          pipelines/camxPreviewRaw.xml  -> ONE port: TFEOutputPortRDI0 -> SinkBuffer RAW
  -> IFE/CSID acquire  op_flags (CAM_IFE_CTX_SHDR_EN?)       (CLOSED core camera.qcom.qcs9100.so)
  -> kernel  is_shdr  (cam_ife_hw_mgr / cam_isp_context)     (OPEN, consumes op_flags only)
```
QCS9075 has **no SFE**, so the relevant path is "auto SHDR without SFE" (`cam_req_mgr_interface.h`).

## The three gaps to close
1. **Signal:** the SHDR intent must reach the selector. `vhdr=shdr-raw` sets a camera xtraparam, not
   necessarily `operation_mode StreamConfigModeVideoHdr` on the raw stream — and the selector even
   strips that bit. Cleanest fix: have the selector query the **sensor's advertised SHDR(2EXP)
   capability** (it already reads sensor caps) rather than rely on operation_mode.
2. **Usecase + topology:** there is no SHDR-RAW usecase. Author one:
   - `UsecasePreviewRawSHDR/pipelines/camxPreviewRawSHDR.xml` with **two RDI output ports**
     (RDI0=VC0/HG, RDI1=VC1/LG) instead of the single `TFEOutputPortRDI0`, plus the node/port
     properties that make the acquire request SHDR (2-exposure). Model on the DOL/SHDR references
     (imx766) + the existing dual-RDI usecases (VR/multi-cam) already in `topology/mimas`.
   - `UsecasePreviewRawSHDR/camxUsecasePreviewRawSHDR.xml` usecase wrapper.
3. **Selector wiring:** in `mimas/chxusecaseselector.cpp` `DefaultMatchingUsecaseSelection`, when the
   sensor is SHDR(2EXP) and the stream is a single RAW stream, select the new SHDR-RAW usecase.

## Build + deploy
- QLI (linuxembedded) build is CMake, per chi-cdk component:
  `core/chiframework|chiusecase|chiutils|lib/common/build/linuxembedded/CMakeLists.txt` +
  `oem/qcom/chiusecase/build/linuxembedded/CMakeLists.txt` -> **`com.qti.chi.override.so`**.
- Toolchain: the same SPF SDK (`qualcomm-toolchain-mm/.../LE.QCLINUX.1.0.r1`) we use for the sensor
  `.bin` (ParameterParser) and that already recompiled the sensor customlib `.so`. So the build is in
  reach, but standing up the chi-cdk override build (deps, sysroot) is the biggest single unknown.
- Deploy: replace `/usr/lib/hw/com.qti.chi.override.so` on the board (back up first), restart
  cam-server + iq9web. Recoverable by restoring the shipped `.so`.

## Prerequisites / uncertainties (confirm BEFORE investing in the full build)
- **Does the CLOSED core CamX (`camera.qcom.qcs9100.so`) honor an OEM usecase requesting
  SHDR-without-SFE (2 RDI + `CAM_IFE_CTX_SHDR_EN`)?** The kernel supports the flag, but the core
  translates the topology into the acquire. If the core only wires SHDR through its own known
  usecases, an OEM SHDR-RAW usecase may not set the flag. **This is the one question to put to HVO
  first** — it decides whether the DIY is viable at all.
- Can the IFE/CSID deliver 2 RDI for the sensor's two VCs on this SoC (HW port availability)?
  Likely yes (IFE has multiple RDI paths) but confirm.
- Whether the QMMF recorder passes a raw-SHDR hint into the HAL stream config, or the selector must
  infer SHDR from sensor caps (option (1) above).

## Recommended prove-out order (cheapest risk first)
1. **HVO gate:** confirm the core CamX will honor an OEM SHDR-RAW (2-RDI) usecase on a no-SFE SoC.
   If no -> stop; it's an HVO/Qualcomm core change, not DIY.
2. **Stand up the chi-cdk override build** unchanged; reproduce the shipped `com.qti.chi.override.so`
   byte-for-byte-ish (the same proof we did for the sensor `.bin`). Gate: if we can't build the
   stock override, DIY is blocked on tooling.
3. **Author the SHDR-RAW topology + usecase + selector edit**; rebuild; deploy; `dcg-sweep --shdr`
   and check dmesg for `is_shdr 1` + frames -> `dcg_demux analyze --interleaved`.

## Effort / risk
- Topology + selector edits: **moderate** (2 XMLs + a selector branch, patterned on existing dual-RDI
  usecases).
- chi-cdk override build bring-up: **the main effort/unknown** (deps, sysroot, integrating into the
  SPF build).
- Overall: **medium-large**, and gated by the HVO feasibility answer in step 1. If the core honors it,
  this is a real self-serve path; if not, it's a Qualcomm core change.

## Key paths
- Selector: `chi-cdk/oem/qcom/chiusecase/mimas/chxusecaseselector.cpp`
- RAW usecase/topology: `chi-cdk/api/oem/qcom/topology/mimas/usecase-components/usecases/UsecasePreviewRaw/`
- Override build: `chi-cdk/oem/qcom/chiusecase/build/linuxembedded/CMakeLists.txt`
- On board: `/usr/lib/hw/com.qti.chi.override.so`
- Kernel flag: `CAM_IFE_CTX_SHDR_EN` (`camera-kernel .../cam_isp_hw_mgr_intf.h:52`)
