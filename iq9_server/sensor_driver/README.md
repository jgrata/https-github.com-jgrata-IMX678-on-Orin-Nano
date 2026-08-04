# IMX678 sensor-config kit — own the driver ourselves

Generate, compile, and deploy IMX678 sensor-mode variants for the IQ9 **without hvo / Leopard /
a build host**. Everything runs on this PC (the QLI 1.7 ParameterParser is in the toolchain we
pulled). This is the mechanism behind [`../docs/imx678-mode-inventory.md`](../docs/imx678-mode-inventory.md).

## Status: PROVEN end-to-end (2026-08-04)
Recompiling the baseline reproduces the **shipping `.bin` byte-for-byte** (md5 `eb236184e9df`,
0 byte diffs). The HCG variant differs only in the conversion-gain register data + name tag.

## The loop
```bash
python build_variants.py     # baseline + register overrides -> generated/*_sensor.xml (+ yaml)
./compile.sh                 # -> bins/com.qti.sensormodule.cmk_imx678_cam0*.bin
./deploy.sh cmk_imx678_cam0_hcg   # compile to cam0 identity, install, restart cam-server, validate
#   ...characterize with the DAQ (iq9_client.py)...
./deploy.sh restore          # back to the shipping sensor
```

## What's here
- **`baseline/`** — LI's `cmk_imx678_sensor.xml` (the one linear mode) + `cmk_imx678_module_cam0.xml`.
  These are the *source*; `baseline/cmk_imx678_sensor.xml` == the shipping LCG config.
- **`build_variants.py`** — stamps out variants by named-register override (no hand-editing 900-line
  XML). Verifies each override landed. Edit the `VARIANTS` list to add configs.
- **`compile.sh`** — runs the local ParameterParser (`b` command) directly on each variant.
- **`deploy.sh`** — activate a variant on the IQ9 (or `restore` / `status`).
- `generated/`, `bins/`, `.deploy_stage/` — build outputs (git-ignored; regenerate any time).

## Variants generated now (all no-SRM, baseline-derived)
| Variant | Register | Purpose |
|---|---|---|
| `cmk_imx678_cam0` | — (identity) | LCG baseline / reference (reproduces shipping bin) |
| `cmk_imx678_cam0_hcg` | `0x3030=0x01` | **HCG** — with LCG gives the DCG/Clear-HDR ratio K_LCG/K_HCG |
| `cmk_imx678_cam0_g0/g12/g24` | `0x3070=0x00/28/50` | analog-gain ladder (0/12/24 dB) for PTC / SNR-vs-gain |

Register facts: `FDG_SEL0` 0x3030 (0=LCG,1=HCG); `GAIN` 0x3070 (0.3 dB/step); `SHR0` 0x3050–52
(integ = VMAX−SHR0; t_line 14.815 µs → baseline 20 ms); `VMAX` 0x3028–2A (framerate). Layer-A
modes (10-bit, DOL, **DCG**, ROI/binned) need Sony-SRM register tables — add them as new
`<resolutionData>` blocks, then the same compile/deploy loop applies.

## Toolchain / environment notes (the gotchas we hit)
- **ParameterParser** ships at `…/chi-cdk/api/tools/buildbins/{linux64,linuxarm,win32}/`. We use
  `linux64`.
- **Windows MAX_PATH**: the toolchain extracts to a >260-char path, so launching the native
  `.exe` from Git Bash fails ("working directory too long"). Fix used: run the parser through
  **WSL** (Linux has no MAX_PATH). `compile.sh`/`deploy.sh` auto-detect Windows and route through
  `wsl.exe` (with `MSYS2_ARG_CONV_EXCL='*'` so `/mnt/c` args aren't rewritten). On a Linux build
  host they run natively — no WSL.
- **Staging location**: XMLs must sit at top-level `…/chi-cdk/oem/qcom/{sensor/cmk_imx678,module}`
  where each XML's `../../../../api/sensor` schema path resolves. (We call ParameterParser directly
  rather than `buildbins.py`, whose ChiCdkRoot=`api` makes it look in `api/oem/qcom` instead.)
- **cam0 identity on deploy**: `deploy.sh` compiles the chosen variant to output name
  `com.qti.sensormodule.cmk_imx678_cam0.bin` so the internal tag matches the socid_map entry —
  installing a differently-tagged bin risks the cam-server SIGSEGV seen in LI bring-up.
- **Deploy resilience**: activating a variant restarts `cam-server`; the PC supervisor
  (`../iq9_client.py`) confirms the board recovered and can stream. If a variant ever trips the
  watchdog, the board auto-restores (iq9web.service) and `deploy.sh restore` reverts the sensor.

## Override the toolchain path
`CHICDK=/path/to/chi-cdk ./compile.sh` (defaults to the local `iq9075-ref` extraction).
