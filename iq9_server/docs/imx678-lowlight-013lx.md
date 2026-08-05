# IMX678 low-light / 0.13 lx first-order analysis (IQ9)

**Date:** 2026-08-05 · Sensor: IMX678-AAQR1 (color), lens EFL=4 mm **f/2**, our mode 3856×2180 12-bit.

## Datasheet specs (IMX678-AAQR1-C, p23; 12-bit, Gain 0 dB, F5.6, Tj 60 °C)
| Item | Value |
|---|---|
| G sensitivity, HCG | typ **16309** Digit/lx/s (3637 mV/lx/s) |
| G sensitivity, LCG | typ **6117** Digit/lx/s (1364 mV/lx/s) |
| Saturation Vsat (LCG) | **3895** Digit (868 mV) |
| Dark signal (1/30 s) | ≤ **0.67** Digit (negligible) |
| Conversion-eff ratio Rcg (HCG/LCG) | **2.4 / 2.7 / 2.9** (min/typ/max) |
| Pixel | 2.0 µm, Type 1/1.8 |
| 1 Digit | 0.2230 mV (12-bit) |

## Measured on our IQ9 unit (register-level control, RDI RAW)
- **Conversion-gain ratio K_LCG/K_HCG = 2.4×** (signal 367.7 vs 154.0 at fixed light) → **matches datasheet Rcg (2.4–2.9)** ✓
- **Read noise (LCG) = 1.82 DN** (clean capped dark, 7 good frames; DSNU 0.8 DN; uniform across R/Gr/Gb/B)
- Dark pedestal ≈ 200 DN

## 0.13 lx estimate (datasheet sensitivity × F5.6→f/2 scaling [7.84×] + measured read noise)
Sensitivity at f/2: HCG **127 863** Digit/lx/s, LCG **47 957** Digit/lx/s.

| Exposure | signal @0.13 lx (HCG) | SNR_read @0.13 lx | min illum SNR=1 (HCG) | min illum SNR=10 |
|---|---|---|---|---|
| 1/30 s | **554 DN** | 304 | **0.43 mlx** | 4.3 mlx |
| 20 ms (our mode) | 332 DN | 183 | 0.71 mlx | 7.1 mlx |

**Conclusion:** with the f/2 lens the sensor sits **~300× below** the 0.13 lx target (read-noise-limited floor ≈ 0.4 mlx); 0.13 lx is a strong, well-exposed signal, not a threshold. Low-light floor is read-noise-dominated (dark current negligible), consistent with the measured 1.82 DN.

## Caveats / to firm up
1. Uses datasheet F5.6 sensitivity scaled to f/2, green channel, typ values. Real SNR at 0.13 lx is **shot-limited** → exact SNR-in-electrons needs **K (e⁻/DN)** from a PTC (deferred: RDI capture reboots intermittently — see `qualcomm-rdi-smmu-escalation.md`).
2. Precise pass/fail vs "Sony 0.13 lx" needs that spec's own **f/#, exposure, and SNR criterion** (datasheet min-illum here is not literally 0.13 lx; that figure's conditions TBD).
3. HCG read noise (DN) approximated by the LCG value; at 554 DN it's shot-dominated so this is second-order.

## Sources
`C:\Users\JGrata\mms\LI-IMX678\IMX678\` — datasheet, `IMX678_Standard_Register_Setting_Ver3.0.xlsx`, ClearHDR/DOL/DualGain app notes, `IMX678-AAQR1_SpectralSensitivity` (relative QE). SRM = `IMX678_SoftwareReferenceManual_E_Rev4.0.pdf`.

## Illuminant caveat (2850K datasheet vs D65 measurement) — added 2026-08-05
Datasheet **Sensitivity** is measured under **Standard imaging condition I = 2850 K source + IR-cut CM700** (3200K is only used for saturation/condition II). Our DMX captures were **D65 (~6500 K)**. Implications:
- **Conversion-gain ratio (2.4x) and read noise (1.82 DN) are unaffected** — a same-light ratio (illuminant cancels) and a dark measurement.
- **Absolute sensitivity / SNR1s** need a **D65->2850K green-channel spectral correction** (computable from IMX678-AAQR1 relative spectral response x the two illuminant spectra; magnitude is modest, order +-20%). It does NOT change the first-order conclusion (we sit ~300x below 0.13 lx). Fold the exact factor in when computing a precise SNR1s / sensitivity number.
