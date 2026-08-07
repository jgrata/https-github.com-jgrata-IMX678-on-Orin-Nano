# Slack post — SHDR/Clear-HDR raw RDI capture (#camera-hw-eng, 2026-08-07)

New post (not a reply). Tag hvo (QLI 1.7) + kkufalk (QLI 2.0). Attach the two files below.

Attach:
- `qualcomm-shdr-rdi-escalation.md` — the escalation writeup (finding, source-grounded root cause, 4 asks)
- `dcg-shdr-chi-override-scope.md` — the DIY CHI-override scope (referenced by ask #1)

---

## Slack message (copy from here)

@hvo @kkufalk — separate camera issue from the RDI-release SMMU panic (no panic in this one). This is about RDI-capturing the IMX678's Clear HDR (dual-conversion-gain, 2-exposure) *raw legs* for sensor characterization.

Where we are:
• *Sensor SHDR mode works* — we rebuilt the sensormodule .bin with the Clear HDR dual-VC descriptor (one streamConfiguration with two `<vc>`, dt 0x2B RAW10, per-leg 2180, HDRExposureType=TWOEXPOSURE, capability=SHDR; reverse-engineered from camxsensordriver.xsd + the imx766 DOL reference). CamX reads it: `capability: SHDR(2EXP), 3856x2180, 15fps`. Deploys clean, no reboot.
• *qtiqmmfsrc property applied* — `vhdr=shdr-raw` sets `QMMF_VIDEO_HDR_MODE=kSHDRRaw` (traced in qmmf_source_context.cc).
• *IFE acquire won't enter SHDR* — dmesg: `Acquired Single IFE[1] with [1 rdi] ports ... is_shdr 0 is_shdr_master 0` + `Get unexpect evt:2 in acquired state`. Single RDI, is_shdr 0 → no buffers → no frames (tried 2180 and 4360). No SMMU panic.

Source-grounded root cause (open kernel + chi-cdk): `is_shdr=1` needs the IFE-acquire op_flags to set `CAM_IFE_CTX_SHDR_EN` (BIT7, cam_isp_hw_mgr_intf.h:52). The kernel only *consumes* it; it's set by the CHI usecase resolved for the stream config. For a kBayerRDI16BIT raw track the mimas selector (chxusecaseselector.cpp) resolves a single-RDI raw usecase (UsecasePreviewRaw = one TFEOutputPortRDI0) and never sets the flag — it even strips the VideoHDR operation_mode bit. QCS9075 has no SFE, so this is the "auto SHDR without SFE" path. Camera usecases are compiled into the prebuilt libs (no runtime usecase XML on the board; usecaseKvManager.xml is audio-only), so there's no config knob.

Asks:
1. *(DIY go/no-go)* Will the *core* CamX (camera.qcom.qcs9100.so) honor an *OEM* CHI usecase requesting SHDR-without-SFE (2 RDI ports + CAM_IFE_CTX_SHDR_EN) on the no-SFE QCS9075 — or is SHDR-RDI wired only through Qualcomm's own built-in usecases? We can author a custom SHDR-RAW usecase + rebuild com.qti.chi.override.so (scope attached), but only if the core will honor it.
2. *(supported 1.7 path)* Any supported way on 1.7 to RDI-capture a 2-exposure SHDR sensor's raw legs that we've missed (usecase/property/config)?
3. *(QLI 2.0 — kkufalk)* We're migrating to 2.0 soon. Does 2.0's camera stack change any of this — native SHDR-RDI raw support, an SFE on this SoC class, a different CHI/usecase model, or different qtiqmmfsrc HDR handling? We don't want to invest in a 1.7 CHI-override DIY if 2.0 supports it natively or changes the mechanism. If 2.0 differs, what's the recommended path?
4. *(alternative)* Is `vhdr=shdr-yuv` (the VC/processed-YUV path) expected to expose the raw legs, or only a merged output? We need the raw per-leg data.

Full writeup + the DIY scope are attached. Happy to hop on a call if easier.
