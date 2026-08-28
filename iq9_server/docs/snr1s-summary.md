# IMX678 SNR1s — consolidated notes, findings, to-dos & deficiencies

*Rollup of all SNR1s (low-light figure-of-merit) work across the eCAM/Jetson and IQ9 phases.
Measured results are point-in-time; tooling state verified against the repo. Last consolidated 2026-08-28.*

---

## 1. What SNR1s is, and the Sony spec

- **SNR1s** = the scene illuminance (lux) at which **SNR = 1**. At SNR=1 the signal is **shot-noise-limited (~1 e⁻)**, so it's a low-light floor metric. **Lower = better.**
- **Sony call-out conditions:** **100 lux at the surface of an 18% gray target, F1.4, 1/60 s, 1 m**, source **3200 K** (datasheet *sensitivity* is spec'd at **2850 K** + IR-cut CM700; the SNR1s marketing figure's exact conditions are less clearly documented — see deficiencies).
- **Sensor-referred vs system:** our measurement is a **system** number (sensor **+ lens + IR-cut**); Sony's datasheet figure is a **sensor reference**. They are not directly comparable without optics de-embedding.

### ⚠️ Vendor figure is ambiguous in our own notes
| Source | Sony color-IMX678 SNR1s |
|---|---|
| `dmx-illuminant-control` memory | **0.13 lx** |
| `imx678-lowlight-013lx.md` (retraction note) | **~0.31 lx** |
| User (verbal, this program) | "~half of our 0.26" → **~0.13 lx** |

**Pin this before any pass/fail claim.** The doc filename ("013lx") predates the 0.31 note in the same file.

---

## 2. Methodology (hard-won — follow these)

1. **DARK read noise + light responsivity/K (hybrid). NEVER the light-PTC intercept.** LED/switching-PSU/DMX flicker inflates the PTC read-noise intercept (once gave ~10 e⁻ vs true ~2.6 e⁻). True dark = **cap the lens**; "DMX=0" is *not* dark (residual ambient shot noise).
2. **Responsivity (e⁻/lux) is conversion-gain-INDEPENDENT** (HCG changes only e⁻→DN, not photons→e⁻). ⇒ **HCG SNR1s = SNR1_e(HCG capped-dark read) ÷ responsivity(measured on the stable LCG bin)** — no HCG sweep needed (and HCG sweeps crash the IQ9 sensor — see §5).
3. **PTC from CONSECUTIVE frames of a persistent stream** (2-frame diff = true temporal noise). The `grab_raw16` open/close path corrupts variance/K (each grab lands pre-3A-steady-state → false 6–7% "flicker").
4. **Formulas** (`raw_ptc.py`):
   - sensor-referred `SNR1_e = (1 + √(1 + 4·read²)) / 2`
   - lux-referred `SNR1s = SNR1_e / responsivity(e⁻/lux)`
   - spec-normalize `SNR1s_spec = SNR1s_meas · (t_meas/t_spec) · (N_spec/N_meas)² · (ρ_meas/ρ_spec)` (exposure, aperture, reflectance). **Spectrum is NOT correctable this way** — match 3200 K with a tungsten source.
5. **Lux calibration:** read a photometer at the target plane per light level. Lux calibrates the *light axis*; it does **not** convert e⁻→lux (that ratio depends on exposure/aperture/QE).

---

## 3. Findings — measured results

### 3a. eCAM / Jetson (Orin, LCG-only, analog-gain sweep, D65+tungsten)
| Metric | Value |
|---|---|
| Green SNR1s (light-PTC, flicker-contaminated) | 3.75 lux min @ gain 4× |
| **Green SNR1s (dark-read-noise corrected, Sony F1.4/18%)** | **0.86 lux min @ gain 16×** |
| Dark read noise | 4.1 e⁻ @1× → **1.70 e⁻ floor @16×** |
| Conversion gain | LCG only (libargus/e-CAM86 expose no HCG; DOL mode = dual-*exposure*, not dual-CG) |
| Raw green SNR1s @DMX48 tungsten | 1.37 lux → 5.33 lux normalized |

### 3b. IQ9 (QLI 1.7, register-level HCG, tungsten, 2026-08-24) — best data to date
| Metric | LCG | HCG |
|---|---|---|
| **SNR1s green (lux @ SNR=1)** | 0.48 | **0.26** |
| SNR1s R / B (lux) | 0.73 / 1.23 | 0.39 / 0.66 |
| K (e⁻/DN) | 1.37 | 0.53 (ratio ~2.6×) |
| Read noise, capped dark (e⁻) | 2.57 | **1.07** (~2.4× lower) |
| Responsivity tungsten R/G/B (e⁻/lux) | 4.27 / 6.56 / 2.54 | (CG-independent) |
| **HCG vs LCG** | — | **1.85× better** |

- **D65 earlier run:** responsivity R/G/B 2.25 / 5.25 / 3.12 e⁻/lux; SNR1s 1.37 / 0.60 / 1.00 lux (green ~0.60).
- **OETF linear** vs calibrated lux, R² 0.996–0.99995 (residual = canopy daylight offset; slope/responsivity offset-invariant).
- **Flicker negligible:** LED PSU + DMX frame-mean spread **0.001–0.016%**.
- Full-well / DR from the LCG sweep are **underestimates** (vignette clips center first; no capped LCG dark that session) — trust prior DR ~63 dB(G); redo with flat field.

### 3c. Datasheet (IMX678-AAQR1-C, 12-bit, 0 dB, F5.6, Tj 60 °C)
| Item | Value |
|---|---|
| G sensitivity HCG / LCG | 16309 / 6117 Digit/lx/s |
| Conversion-eff ratio Rcg (HCG/LCG) | 2.4 / 2.7 / 2.9 (min/typ/max) — **our measured 2.4–2.6× matches** ✓ |
| Vsat (LCG) | 3895 Digit |
| Dark signal (1/30 s) | ≤ 0.67 Digit |
| Measured read noise (LCG) | 1.82 DN (clean capped dark); pedestal ≈ 200 DN |

### 3d. The "~2× worse than Sony" gap (measured 0.26 lx green HCG vs Sony ~0.13)
SNR1s ∝ read_e / (e⁻ per lux). With HCG already applied, the residual gap is **signal-collection efficiency**, i.e. a **system vs sensor-reference** difference:
- **IR-cut filter under IR-rich tungsten (~2–3×)** — dominant. Lux counts visible only; a bare/reference sensor's QE collects the NIR tail our IR-cut blocks.
- **Lens true T-stop / transmission** and **QE / coverglass (~1.3–1.7×)**.
- Plus not-yet-de-embedded differences: illuminant (D65/our-tungsten vs 2850–3200 K), target (white vs 18% gray), distance (≠1 m) — most of these the `spec_snr1s` normalizer already handles *except* spectrum/IR-cut.

**Verdict on "is a ~2× responsivity difference plausible?": Yes.** A ~2× system penalty from IR-cut + lens-T + QE under IR-rich tungsten is expected and does **not** indicate a measurement error. To *match* a datasheet number requires de-embedding the optics.

---

## 4. Tooling state (what exists, where)
- **IQ9:** `iq9_server/raw_ptc.py` — PTC/OETF/SNR analyzer, `lux_metrics()` (responsivity + lux-SNR1s), `spec_snr1s()` (f/#/exposure/reflectance normalization); self-test PASS. `iq9_server/ptc_capture.py` — stream-based sweep (`--capture-dark`, `--lux`). Webui: `/api/ptc/dark`, `/api/ptc/sweep`, `static/ptc.html` (per-level lux + flicker readout), `/fieldmap` (uniformity).
- **Jetson:** `lab/oetf_ptc.py` (`snr1s_lux`, run_sweep/compute_metrics/snr1s_normalize), `lab/snr1s_vs_gain.py` (gain sweep + dark-rn fold-in), `lab/readnoise_dark.py`.
- **Data:** `lab/snr1s_vs_gain_tungsten.*`, `lab/readnoise_dark.json`, `lab/ptc_tungsten_lvl48*.csv`, `lab/ptc_d65_lvl48_baseline.csv`.

---

## 5. Deficiencies / open risks
- ⚠️ **Sony spec figure not pinned** (0.13 vs 0.31 lx) and its exact conditions (f/#, exposure, SNR criterion, source) undocumented.
- **System→sensor de-embedding missing:** need measured **lens T-stop/transmission**, **IR-cut transmission @3200 K**, and **sensor QE** to make our number comparable to the datasheet.
- **Illuminant/spectral correction** (D65→2850 K green, ~±20%) not folded in; run under matched **3200 K tungsten**.
- **Ambient contamination:** canopy not yet light-tight (skylight leak) — recent runs mixed-illuminant. *(Canopy rebuild in progress.)*
- **HCG sweep instability (1.7):** rapid open/apply on the HCG `.bin` wedges CCI/I2C → cam-server dies (`i2c poll -22`). Worked around (LCG sweep + HCG-dark projection). Stability on **2.0 unknown**.
- **Full-well / DR underestimated** (vignette + no capped LCG dark that session).
- **HCG `.bin` not applied on 2.0** (board on shipping LCG bin) — needed for the best SNR1s.

---

## 6. What changed on QLI 2.0 (unblocks a clean re-baseline)
- ✅ **RAW decode now works on 2.0** (linear RGGB, stride 3856 / active H 2176) — the PTC/SNR1s pipeline was blocked on 2.0 by the decode; now unblocked.
- ✅ **`manual-exposure-time` (ns) + `manual-iso-value` are settable on 2.0** — enables true exposure-controlled PTC/SNR1s (1.7 exposure was static in the sensor `.bin`, a major constraint). ⚠️ **Verify these actually drive the RAW/RDI capture** (on 1.7 CamX skipped 3A on RDI).
- ✅ **No RAW-induced reboot on 2.0** — the 1.7 SMMU/RDI instability is moot; HCG-sweep stability still TBD.
- ✅ `spec_snr1s` normalizer ready.

---

## 7. Next steps — rigorous 2.0 SNR1s baseline
1. **Ambient-tight canopy**, pure **tungsten ~3200 K**; meter lux at the 18% gray (or white + reflectance-normalize).
2. **Capped-lens dark** read noise, LCG **and** HCG, on 2.0.
3. **LCG lux-sweep PTC** (consecutive stream frames) → K + responsivity (e⁻/lux).
4. **Project HCG SNR1s** = SNR1_e(HCG dark) / responsivity(LCG).
5. Set **manual exposure = 1/60 s** (confirm it reaches RAW) for a direct number, or normalize via `spec_snr1s`.
6. **Pin the Sony figure** + de-embed optics (lens T, IR-cut, QE) for a sensor-reference comparison.
7. **Apply/rebuild the HCG `.bin`** on 2.0 if needed; re-check sweep stability.

---

## 8. Headline numbers to remember
- **Best measured: ~0.26 lx green (HCG, tungsten, IQ9 1.7).**
- **Sony (unconfirmed): ~0.13 lx** → we are ~2× worse, explained by system-vs-sensor optics (IR-cut/lens-T/QE), not a measurement fault.
- **HCG buys 1.85×** over LCG; conversion-gain ratio 2.4–2.6× (matches datasheet).
