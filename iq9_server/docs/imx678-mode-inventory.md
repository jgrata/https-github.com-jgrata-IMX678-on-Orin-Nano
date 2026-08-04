# IMX678 sensor mode inventory for IQ9 DAQ characterization

**Date:** 2026-08-04 · **Target:** IQ-9075 EVK (QCS9075, QLI 1.7), Leopard IMX678 (cmk_imx678).
**Purpose:** define the sensor modes/configs to compile into the sensormodule `.bin` so the
IQ9 DAQ can measure **OETF, PTC, SNR1s, and WB/CCM across linear, DOL, and DCG HDR** — matching
the Jetson characterization suite.

> Read alongside [`imx678-sensor-control-rootcause.md`](imx678-sensor-control-rootcause.md).
> Per-frame exposure/gain are **not** available on the RDI path (closed CamX). That shapes the
> whole plan below: signal is swept with **light (DMX)**, and the axes CamX can't sweep per
> frame (analog gain, conversion gain, exposure) are covered with **build-swapped `.bin` configs**.

---

## 0. Two hard constraints that shape the inventory

1. **CamX picks the sensor mode by the requested caps** — width×height, data-type/bit-depth,
   framerate, HDR type. Modes that differ *only* in gain / conversion-gain / exposure at the
   **same geometry are not caps-selectable** → they must be **build-swapped** (swap the `.bin`,
   restart cam-server). So the inventory has two layers:
   - **Layer A — caps-selectable modes**: differ in res / bit-depth / fps / HDR. One `.bin`
     can hold all of them; the DAQ selects via GStreamer caps.
   - **Layer B — build-swap variant ladders**: same geometry, differ in analog gain /
     conversion gain / exposure. Delivered as a small set of `.bin` files, swapped between runs.
2. **Per-frame sensor control is off on RDI** → PTC/SNR gain sweeps and exposure sweeps can't
   be done frame-to-frame; they're done by light-sweep (signal) + build-swap (gain/CG/exposure).

## 1. Register levers & timing (verified from the compiled mode)

| Lever | Register | Law / value | Notes |
|---|---|---|---|
| Conversion gain | **FDG_SEL0** 0x3030[1:0] | 0=LCG, 1=HCG | HCG analog-gain floor ~0x22 (34) per Sony SRM |
| Analog gain | **GAIN** 0x3070–72 | **0.3 dB/step**, 0–72 dB (reg 0–0xF0) | 6dB=0x14 12dB=0x28 24dB=0x50 36dB=0x78 48dB=0xA0 72dB=0xF0; current 0x14=6 dB |
| Digital gain | 0x3071? | ≤2× (maxDigitalGain=2) | avoid for DAQ (keep 1×) |
| Exposure (long) | **SHR0** 0x3050–52 | integ_lines = VMAX − SHR0 | current 900 → 1350 lines → **20.0 ms** |
| Exposure (short, DOL) | **SHR1** 0x3058–5A | short-leg shutter for DOL | present in init (394); used only in DOL |
| Frame length | **VMAX** 0x3028–2A | fps = 1/(VMAX·t_line) | current 2250 |
| Line length | **HMAX** 0x302C–2D | sets t_line | current 1100 |
| Bit depth (sensor DT) | mode `<dt>` | RAW8=0x2A, RAW10=0x2B, RAW12=0x2C(44) | current 44 (RAW12) |

**Verified timing (current mode):** t_line = **14.815 µs** (4-lane, 900 MHz out-clk); exposure
range at 30 fps = 1 line…33 ms; current fixed exposure **20 ms**. Exposure for a target t:
`SHR0 = VMAX − round(t / 14.815 µs)` (at this VMAX).

`realToRegGain`/`regToRealGain` XML fields are empty — the gain law (0.3 dB/step) is applied by
the customlib `.so`; use it for planning.

---

## 2. Layer A — caps-selectable sensor modes (compile into the `.bin`)

| ID | Geometry | Bit | Target fps | HDR | dt | Serves | Status of register table |
|----|----------|-----|-----------|-----|----|--------|--------------------------|
| **A0** | 3856×2180 | 12 | 30 | linear | 0x2C | OETF/PTC/SNR/CCM baseline; dark noise | **HAVE** (current compiled mode) |
| **A1** | 3856×2180 | 10 | ~60* | linear | 0x2B | 10-bit OETF/PTC; fps headroom; verify 10-bit RDI | need Sony 10-bit all-pixel table |
| **A2** | 3856×2180 | 12 | ~15 | linear | 0x2C | long-integration / low-noise floor; dark-current baseline | derive from A0 (VMAX↑) |
| **A3** | 3856×**4450** | 10 | 30 | **DOL** 2-exp | 0x2B | DOL OETF/PTC/SNR/CCM (long+short legs + stitch) | **register table from Sony SRM** (Jetson e-con is MCU-abstracted — not portable; see §5) |
| **A4** | 3856×2180 | 12 | 30 | **DCG** (Clear HDR) | 0x2C | DCG HDR: HCG+LCG combine; conversion-gain ratio | need Sony DCG/Clear-HDR table |
| **A5**† | 1928×1090 | 12 | ~60* | linear (2×2 bin) | 0x2C | binned SNR/sensitivity; fps | need Sony binning table |
| **A6**† | 1920×1080 | 12 | ~90* | linear (center ROI) | 0x2C | ROI/windowed DAQ; fps | derive (crop window) |

\* framerate targets — **confirm against the LI/Sony mode tables**; the ceiling is set by the
sensor readout PLL (INCK/SYS), not just the 4-lane MIPI budget (which has headroom).
† A5/A6 optional (framerate/ROI coverage); primary science is A0–A4.

## 3. Layer B — build-swap variant ladders (same geometry, per-`.bin`)

These cover the axes CamX can't sweep per frame. Each is a `.bin` built from A0 (or A3/A4) with
one field changed; swap + `systemctl restart cam-server` between runs (the resilience supervisor
handles the restart+recovery). Keep the set small and purposeful:

| Ladder | Variants | Field | Why |
|---|---|---|---|
| **Conversion gain** | LCG (0x3030=0), **HCG (0x3030=1)** | FDG_SEL0 | HCG vs LCG read-noise & full-well → DCG ratio; low-light noise floor |
| **Analog gain** (PTC/SNR-vs-gain) | 0, 6, 12, 24, 36 dB (reg 0x00/14/28/50/78) | GAIN 0x3070 | anchor K(gain) & read-noise(gain); ≥3 points to fit the gain law, extrapolate the rest |
| **Exposure** (dark-current & linearity-vs-t) | e.g. 1, 5, 20, 33 ms (SHR0) | SHR0 | separate dark current (∝ t) from read noise; verify integration linearity |

> If per-frame exposure/gain ever unlocks (CamX escalation), Layer B collapses to runtime
> parameters and no build-swap is needed. Until then this is the interim mechanism — flag it to hvo.

---

## 4. Characterization coverage matrix

| Measurement | Independent variable | Mode(s) | Method | Blocker-free? |
|---|---|---|---|---|
| **OETF / linearity** | light (DMX 0→255) | A0/A1 (linear), A3/A4 (HDR piecewise) | light-sweep @ fixed exp; fit DN vs relative luminance | ✅ light-sweep |
| **PTC** (K e⁻/DN, read noise, full-well, PRNU) | light | A0 + Layer-B gain/CG ladders | variance-vs-mean over flat-field light-sweep; per gain & per CG | ✅ signal; ⚠ gain/CG axis = build-swap |
| **SNR1s** (EMVA1288) | light + gain | A0 + Layer-B gain ladder | SNR vs signal, normalized by known exp/gain | ⚠ gain axis = build-swap |
| **Dark read noise / DSNU / dark current** | exposure | A0 + Layer-B exposure ladder | lens-capped dark frames vs t; temporal + spatial σ | ⚠ exposure axis = build-swap |
| **WB gains** | illuminant (D65/tungsten via DMX; TL84 ext.) | A0 (+ per CG/HDR to verify invariance) | ColorChecker; R/G/B channel ratios on linearized raw | ✅ |
| **CCM** | illuminant | A0, A3 (DOL), A4 (DCG) | ColorChecker on OETF-linearized, black-subtracted raw; solve 3×3 (+ per-mode to catch stitch effects) | ✅ |

**All measurements consume RDI RAW (pre-ISP)** — exactly what EMVA1288-style work needs (no
tonemap/denoise contamination). The existing Jetson lab science (`lab/snr1s_vs_gain.py`, CCM
solver, OETF/PTC) ports directly; the DAQ just feeds it RDI `.npy` frames via `iq9_client.py`.

## 5. DOL & DCG specifics

- **DOL (A3):** long (SHR0) + short (SHR1) exposures, ratio configurable (start **16:1** → +24 dB
  DR). Characterize **each leg as a linear sensor** (own OETF/PTC/K/read-noise), then the stitch
  knee & blending.
  - **What the Jetson (e-con e-CAM86) taught us — geometry & exposure model (portable), registers (NOT):**
    the e-con module is **MCU-mediated** (`e-con_cam` @0x42) with high-level controls
    (`sensor_mode=3`, `hdr_enable=1`, `exposure` long 450–400001 µs, `exposure_short` 28–25000 µs,
    ratio ≤16×, mode-3 max 30 fps). So there is **no raw IMX678 register table to lift** — the A3
    register/mode sequence must come from the **Sony IMX678 SRM DOL/Clear-HDR table** (the IQ9
    Leopard path is register-level; `SHR1` 0x3058–5A already present in init = DOL-capable).
  - **DOL frame geometry (from Jetson `IDolWdrSensorMode`/DT):** physical **3856×4450**, 2 exposures
    stacked, VBP 65, line-info-marker width 16, ~16.6 MB/frame @30 fps. So A3's `<frameDimension>`
    height must be the **full 4450**, not 2180.
  - **CRITICAL pitfall (the exact Jetson failure to avoid):** the Jetson VI **clamped the DOL
    readout to 3840×2160**, so it captured a 2160-row slice of the 4450-row frame → every buffer
    `V4L2_BUF_FLAG_ERROR` → zero usable RAW. **On the IQ9, the RDI/IFE stream config for A3 must
    accept the full 4450 height** (validate `CheckValidStreamConfig` doesn't clamp it — analogous
    to the RAW10 3840-max rejection we already hit). This is the #1 risk for A3.
  - Reconstruction (long/short → linear radiance) reuses the Jetson `reconstructRadiance` path.
- **DCG (A4):** per-pixel HCG+LCG. The key parameter is the **conversion-gain ratio K_LCG/K_HCG**,
  obtained directly from the two Layer-B PTCs (LCG vs HCG). HCG leg → shadow read-noise; LCG leg →
  highlight full-well. If the sensor outputs a companded "Clear HDR" single stream, capture the
  companding curve (its OETF) and invert before PTC.

## 6. Gaps / risks / to-source

1. **Register tables to source** for A1/A3/A4/A5 (and HCG/gain/exposure deltas): **Sony *IMX678
   Software Reference Manual* mode tables** are the authoritative source (register-level, matches
   the Leopard/IQ9 path). **NOTE: the Jetson is NOT a source for the register tables** — its e-con
   e-CAM86 module is MCU-abstracted (no register tables) and raw DOL never captured there; it
   contributes only the DOL geometry/exposure model + the geometry-clamp pitfall (§5). Cross-check
   Leopard's own driver if they publish a DOL/DCG mode. Leads: Sony SRM (user has it), the e-con
   `IMX678Standard_vs_HDR` docs (Data/imx678/econ) for the HDR behaviour, and the `ATG-IMX678
   Source Code Overview` PDF in the repo root.
2. **ParameterParser** (QLI 1.7 V5.5.1) to compile XML→`.bin` — **already in-house** (Metropolis
   has CreatePoint access and has compiled the current `.bin` with it). SRM to be sourced from
   **Leopard directly** (reputable) rather than third-party scans.
3. **RDI over HDR modes unverified** — validate A3/A4 actually stream over the RDI/bayer path on
   this stack before committing the full DAQ matrix.
4. **Gain/CG/exposure axes are build-swaps** until per-frame control is unlocked (CamX escalation).
   Keep Layer-B ladders minimal (≥3 gain points to fit, not the full 0–72 dB range).

## 6b. BSP ownership reality (from the LI↔Metropolis BSP thread, May–Jun 2026)

- **The rebuild path is already in-house, not hypothetical.** Metropolis has Qualcomm CreatePoint
  access + the **QLI 1.7 ParameterParser (V5.5.1)**, has LI's XML **source**
  (`cmk_imx678_sensor.xml`, `cmk_imx678_module_cam0.xml`, `cmk_imx678_sensor.cpp`), and has
  **already compiled `com.qti.sensormodule.cmk_imx678_cam0.bin` and recompiled
  `com.qti.sensor.cmk_imx678.so`** against QLI 1.7. ⇒ adding modes = authoring XML resolutionData
  + re-running the parser we already have (the customlib `.cpp` is also recompilable if
  `FillExposureSettings`/`CalculateExposure` ever need changing).
- **LI's config is minimal by design** — "only basic image tuning… 1 or 2 control variables," GA1.4
  base, one linear mode, tuning shared with IMX676. Full DOL/DCG/multi-mode support from LI is
  **NRE or DIY**. ⇒ the mode set in §2–§3 is ours to author (Sony SRM register tables), which is
  exactly "writing our own sensor driver."
- **Color is a hybrid** — LI's 9 CCMs spliced into a `lemans_imx577` skeleton, `cc13_ipe_v2.xml`
  excluded → ISP colors approximate. ⇒ **derive WB/CCM from RAW ourselves** (already the DAQ plan);
  do not depend on the ISP-tuned color.
- **Scope boundary (important):** owning the sensor driver unlocks all of §2 (modes) and §3
  (static per-build gain/CG/exposure). It does **NOT** fix per-frame exposure/gain on RDI — that
  handoff is in prebuilt **CamX**, upstream of the sensor driver. Per-frame 3A on RDI stays a
  Qualcomm/CamX escalation regardless of how much of the sensor driver we own.

## 7. Rebuild / deploy steps (in-house — toolchain already available)

1. Edit `meta-metro-mcs/.../li-imx678/files/cmk_imx678_sensor.xml`: add A1–A4(+A5/A6) as new
   `<resolutionData>` blocks (each with its resSettings mode table), and produce the Layer-B
   variant XMLs (A0 with FDG_SEL0/GAIN/SHR0 deltas).
2. Run the **QLI 1.7 ParameterParser (V5.5.1)** to regenerate
   `com.qti.sensormodule.cmk_imx678_cam0.bin` (and cam1–3) per config.
3. Install to `/usr/lib/camera/` (rm-then-cp; `/usr` is ostree — `mount -o remount,rw /usr`),
   `systemctl restart cam-server`, then `systemctl start iq9web`.
4. Validate each caps-mode: request its caps via `iq9_client.py`, confirm the RDI frame geometry
   /bit-depth, run the coverage matrix. Build-swaps: swap `.bin` + restart cam-server (supervisor
   recovers automatically if a mode trips the watchdog).
