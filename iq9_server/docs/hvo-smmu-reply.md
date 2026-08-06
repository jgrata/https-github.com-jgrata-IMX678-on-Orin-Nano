# Reply to hvo — #camera-hw-eng Slack thread (2026-08-06)

Reply to hvo's question ("did you walk the actual source, or is this from the panic
trace + symbols alone?"). Post in-thread and attach the three files below.

Attach:
- `qualcomm-rdi-smmu-escalation.md` — full writeup (source walk, r11 dead-end + DT proof, two-level fix, the five asks)
- `0001-iommu-arm-smmu-qcom-skip-fault-path-ATOS-no-tbu-power.patch` — proposed unblocker (git am-ready)
- `rdi_smmu_panic_serial.txt` — full serial panic trace (register dump + module list)

---

## Slack message (copy from here)

Following up on your question — yes, this is now from the actual source, not trace+symbols.

I pulled the CLO kernel at the exact SRCREV our image builds from (git.codelinaro.org/clo/la/kernel/qcom.git, branch `kernel.qclinux.1.0.r1-rel`, SRCREV `de229c16e2aa` — matches the running `6.6.116-...-gde229c16e2aa`) and walked the path end to end:

• `arm-smmu.c`: `arm_smmu_context_fault()` takes `arm_smmu_rpm_get(smmu)` at entry, then calls `impl->context_fault = qcom_smmu_context_fault()` with that ref held — i.e. the "Invoke pm runtime for context fault" change is already in our tree. So the apps_smmu itself is powered during the fault; the abort is *not* on the main SMMU register space. Matches the panic trace exactly (`arm_smmu_context_fault -> qcom_smmu_context_fault -> qsmmuv500_iova_to_phys`).
• `arm-smmu-qcom-tbu.c`: `qsmmuv500_iova_to_phys()` resolves the faulting address with a hardware ATOS that pokes a per-TBU DEBUG block (`tbu->base`). It's fail-soft and brackets the ATOS with `icc_set_bw(tbu->path)` + `clk_prepare_enable(tbu->clk)` + `qsmmuv500_tbu_halt()`.
• BUT `sa8775p.dtsi` gives all 18 `qcom,qsmmuv500-tbu` nodes — including `cam_tbu@0x151f1000` — no clocks/power-domains/interconnects. So `tbu->clk` and `tbu->path` are NULL, those guards are no-ops, and `qsmmuv500_tbu_halt()` pokes `cam_tbu` regardless.

*Root cause:* `cam_tbu`'s DEBUG register block is powered by the camera subsystem (CAMNOC/camcc), not by anything the SMMU driver can enable. During IFE/RDI teardown that subsystem powers down; the threaded context-fault IRQ races the teardown and the ATOS register access takes a synchronous external abort (ESR 0x96000010, DFSC 0x10) -> qcom_wdt reset. `arm_smmu_rpm_get()` covers the apps_smmu, not `cam_tbu`, so the pm-runtime change doesn't help. (The 0x1568-byte `qsmmuv500_iova_to_phys` in the trace is just inlined find_tbu/halt/trigger_atos, not an older version.)

I also checked the obvious lead and it's a dead end: diffing `arm-smmu-qcom-tbu.c` between our r1-rel and the newer r11-rel, `qsmmuv500_iova_to_phys()` is byte-identical (only delta is `fault_dev` vs `NULL` to `report_iommu_fault`). So a version bump / backport does not fix this.

*Proposed unblocker* (patch attached, git am-ready, builds on r1): the ATOS result is used only for a diagnostic `dev_err` in `qcom_smmu_context_fault()` — the fault report and FSR clear/RESUME don't depend on it. So when the TBU has no power handle (`!tbu->clk && !tbu->path`), skip the hardware ATOS. SoC-agnostic; TBUs that do expose a clock/interconnect are unaffected. Turns the reboot into a recoverable, reported fault. We'll build + soak this on our IQ-9075.

*What we can't resolve on our end and need from you:*
1. Is `!tbu->clk && !tbu->path` the right "no driver-controllable TBU power" test for SA8775P, and is any such TBU actually always-on (where skipping the ATOS would drop a needed diagnostic)? We can't see the TBU power topology.
2. Or should `cam_tbu` instead be *given* a controllable clock/GDSC in DT + a pm_runtime/clk get in the ATOS path — making it safe rather than skipped? Only your HW/power docs answer this.
3. Can this (or your preferred variant) land as a CR in QLI 1.7 (hvo) and 2.0 (kkufalk)?
4. The true root fix is in camera-kernel — the IFE RDI-release ordering (a buffer accessed after unmap) that raises the fault in the first place. Is there a CR for that in r2 / 2.0? We can build camera-kernel but can't author this without the IFE internals.
5. Does any debug/crash-dump tooling consume the fault-path `phys_atos`, i.e. would skipping it lose a diagnostic you rely on?

On your third question (instrumenting the release/unmap + TBU clock state): the DT + source already pin the mechanism, but if you'd like empirical confirmation before accepting the guard, we can add a debug print of `tbu->clk` and the faulting address in the fault path and capture it over serial — just say the word.

Full writeup and the serial trace are attached.
