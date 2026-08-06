# IMX678 RDI capture → kernel panic in the Qualcomm SMMU driver (`qsmmuv500_iova_to_phys`)

**For:** hvo (QLI 1.7) · kkufalk (QLI 2.0) · Qualcomm camera/IOMMU
**Date:** 2026-08-05 · **Reporter:** Metropolis (Jeremy Grata)
**Full serial trace attached:** `rdi_smmu_panic_serial.txt`

## TL;DR / asks
Capturing native RAW (RDI/bayer) from the IMX678 on QCS9075 reliably-intermittently **panics the kernel inside the ARM-SMMU driver** — the SMMU **fault handler itself** takes a synchronous external abort in `qsmmuv500_iova_to_phys`, and the `qcom_wdt` watchdog resets the SoC. It is **not** the camera DLKM crashing, and it is **not** fixed by the r1-rel camera-kernel (which we run, incl. `f7b70309`). We need:
1. **Known fix?** The obvious lead is a **dead end**: we diffed `arm-smmu-qcom-tbu.c` between our `r1-rel` and the newer `r11-rel` and `qsmmuv500_iova_to_phys()` is **byte-identical** (only delta: `qcom_smmu_context_fault` passes `fault_dev` vs `NULL` to `report_iommu_fault`) — so moving to / backporting **r11 does not fix this**. Is there any CR that makes the fault-path ATOS safe on a **camera TBU during teardown**? Root cause (below): the ATOS pokes `cam_tbu@0x151f1000`, which has **no DT clock/power-domain**, so the in-tree `clk_prepare_enable(tbu->clk)` guard is a no-op on SA8775P; when the camera subsystem powers down at IFE release the ATOS register access external-aborts and the SMMU driver has no handle to re-power it. (The SMMU-device PM-runtime fix *"iommu/arm-smmu: Invoke pm runtime for context fault"*, quicinc Apr 2025, is present but powers only the apps_smmu, not cam_tbu.)
2. **Handler hardening (high value):** can the context-fault handler be made **non-fatal** (report `FSR`/`FSYNR`/`CBFRSYNRA`/`FAR` **without** the HW ATOS translation, or skip ATOS when the TBU/CAMNOC power domain is down)? That alone would turn the IFE fault into a recoverable dropped frame instead of a reboot — **making RDI capture usable now**. GPL source, we can build it.
3. **kkufalk / QLI 2.0:** is this SMMU panic (and/or the IFE RDI-release fault) already resolved in 2.0's kernel + camera-kernel? If so, which commits — for backport to 1.7 or to inform the 2.0 move.

## Environment
| | |
|---|---|
| SoC / board | QCS9075 / IQ-9075 EVK |
| SW | QLI 1.7 **r1.0_00114.0**, kernel `6.6.116-qli-1.7-ver.1.1-05801-gde229c16e2aa-dirty` |
| Sensor | Leopard Imaging IMX678 (`cmk_imx678`), CCI0/JCAM0, 4-lane, RAW12, native 3856×2180 |
| Capture | RDI/bayer **RAW16** via `qtiqmmfsrc` (`video/x-bayer,format=rggb,bpp=(string)16,3856x2180`) |
| camera-kernel | built at `camera-kernel.qclinux.1.0.r1-rel` **HEAD f4491100** (includes `f7b70309` "msm: camera: isp: Fix KMD buffer handle in IFE prepare" + all r1-rel UAF/lifecycle fixes). Loaded module md5 `f08df714…` |

## Reproduction
Cold RDI snapshot (NV12 released, camera quiesced, cam-server fresh):
```
gst-launch-1.0 -e qtiqmmfsrc ! "video/x-bayer,format=rggb,bpp=(string)16,width=3856,height=2180,framerate=30/1" \
   ! identity eos-after=2 ! multifilesink location=/var/iq9raw/f_%03d.bin
```
The valid RAW frame **is** produced; the panic fires on the pipeline **release/flush**, intermittently (~50% at present; historically ~1/49 at 3 s quiesce).

## Root cause — serial console capture (`ttyMSM0 @ 115200`)
```
CAM_INFO: CAM-CRM: cam_req_mgr_process_flush_req: 3209: Last request id to flush is 8 on link 0x170304
CAM_ERR:  CAM-SMMU: cam_smmu_dump_cb_info: 747: Usage: shared_usage=0 io_usage=3674112 ...
Internal error: synchronous external abort: 0000000096000010 [#1] PREEMPT SMP
CPU: 0 PID: 617 Comm: irq/44-arm-smmu  Tainted: G   M   W  O   6.6.116-qli-1.7-...-dirty
pc : qsmmuv500_iova_to_phys+0x950/0x1568
lr : qsmmuv500_iova_to_phys+0x13c/0x1568
Call trace:
 qsmmuv500_iova_to_phys+0x950/0x1568
 qcom_smmu_context_fault+0x44c/0x890
 arm_smmu_context_fault+0x70/0x548
Kernel panic - not syncing: synchronous external abort: Fatal exception
```
**Interpretation:** on RDI release the IFE issues a DMA to an IOVA that faults → `arm_smmu_context_fault` → `qcom_smmu_context_fault` calls `qsmmuv500_iova_to_phys` (hardware ATOS) to resolve the faulting address for the report → that **register access external-aborts (ESR 0x96000010 = sync external abort)** → panic. This is consistent with the **IFE TBU being unclocked / powered-down during teardown** when the fault handler runs the ATOS. The underlying *trigger* is an IFE buffer accessed after unmap on RDI release; the *fatal* event is the SMMU driver dying while handling it.

## Source walk (CLO r1-rel `de229c16e2aa` = our running kernel) — not trace/symbols alone
**Source location:** `git://git.codelinaro.org/clo/la/kernel/qcom.git`, branch `kernel.qclinux.1.0.r1-rel`, `SRCREV = de229c16e2aad78e054a222957219e8fda5bb335` — the short hash matches the running `6.6.116-qli-1.7-ver.1.1-05801-gde229c16e2aa-dirty`. (Yocto recipe `meta-qcom-hwe/.../linux-qcom-custom_6.6.bb`; this is the image that ships, **not** the mainline-stable `linux-qcom-base_6.6.bb`.)

Read directly from that tree:
1. **`arm_smmu_context_fault()` (`drivers/iommu/arm/arm-smmu/arm-smmu.c`) already holds a PM-runtime ref.** It calls `arm_smmu_rpm_get(smmu)` at the very top (before the `ARM_SMMU_CB_FSR` read) and dispatches to `smmu->impl->context_fault` (= `qcom_smmu_context_fault`) with that ref held (`goto out_power_off` → `arm_smmu_rpm_put`). This is the QCLINUX fix **"iommu/arm-smmu: Invoke pm runtime for context fault"** (Prakash Gupta / Pratyush Brahma, quicinc; Change-Id `I560923fd…`; 2 Apr 2025; Upstream-Status: Pending — also carried in the base recipe as `common/0001-QCLINUX-iommu-arm-smmu-Invoke-pm-runtime-for-context.patch`). **So that patch is already present in our kernel.**
2. **Therefore the abort is NOT on the main-SMMU register space** (FSR/FSYNR/CBFRSYNRA are powered by the rpm_get). It is one level deeper.
3. **The abort is the hardware ATOS on a camera TBU.** `qsmmuv500_iova_to_phys` (in `arm-smmu-qcom-tbu.c`) resolves the faulting address by poking a per-TBU DEBUG register block (`tbu->base`; for the IFE stream this is `cam_tbu@0x151f1000`). That block is powered/clocked by the **camera subsystem (CAMNOC/camcc)**, independent of the apps_smmu's own clocks. During IFE RDI release the camera powers down; the threaded `irq/44-arm-smmu` context-fault races the teardown and the ATOS register access **synchronous-external-aborts** (ESR `0x96000010`, DFSC `0x10`). `arm_smmu_rpm_get(smmu)` powers the apps_smmu but **not** cam_tbu's block. (The `+0x1568` panic-function size is just the compiler inlining `find_tbu`/`halt`/`trigger_atos` into `qsmmuv500_iova_to_phys` — **not** an older/monolithic version.)
4. **Our code already has the fail-soft TBU guard — but it is a NO-OP on this SoC.** `qsmmuv500_iova_to_phys` *does* bracket the ATOS with `icc_set_bw(tbu->path,…)` + `clk_prepare_enable(tbu->clk)` + `qsmmuv500_tbu_halt` (each with an error-return path). **But in the SA8775P DT all 18 `qcom,qsmmuv500-tbu` nodes — including `cam_tbu` — declare no `clocks`, `power-domains`, or `interconnects`** (`tbu->clk`/`tbu->path` come back NULL via `devm_clk_get_optional`/`devm_of_icc_get`), so those calls do nothing and the driver has **no handle to re-power cam_tbu** before the ATOS. The guard that would save another SoC is inert here.

## Moving to `r11-rel` does NOT fix it (source diff + DT, 2026-08-06)
We diffed `drivers/iommu/arm/arm-smmu/arm-smmu-qcom-tbu.c` between our `r1-rel` and the newer `r11-rel`: **`qsmmuv500_iova_to_phys()` is byte-identical.** The only functional delta in the whole file is `qcom_smmu_context_fault()` passing `fault_dev` (r1) vs `NULL` (r11) to `report_iommu_fault` — irrelevant to the abort. Both releases already carry the fail-soft ATOS wrapper:
```c
tbu = qsmmuv500_find_tbu(qsmmu, sid);
if (!tbu) return 0;
ret = icc_set_bw(tbu->path, 0, UINT_MAX); if (ret) return ret;      // tbu->path = NULL here -> no-op
ret = clk_prepare_enable(tbu->clk);       if (ret) goto disable_icc; // tbu->clk  = NULL here -> no-op
ret = qsmmuv500_tbu_halt(tbu, smmu_domain); ...                       // <-- writes tbu->base (cam_tbu) -> ABORTS
    ... qsmmuv500_tbu_trigger_atos(...)                               //     more tbu->base writes
```
…but as noted above, on SA8775P `tbu->clk`/`tbu->path` are NULL, so the guard is inert and `qsmmuv500_tbu_halt` pokes `cam_tbu` regardless. **So backporting / moving to r11 buys nothing** — it would abort in the same place. *(An earlier draft of this doc wrongly proposed the r11 backport; retracted after diffing the actual source and the DT.)*

**Dispatch confirmed in our kernel** (`arm-smmu.c`): `arm_smmu_context_fault` does `arm_smmu_rpm_get` (line 421) then calls `impl->context_fault` (426) = `qcom_smmu_context_fault` → `qsmmuv500_iova_to_phys` — exactly the panic trace. (r11 instead registers `qcom_smmu_context_fault` as a **direct threaded IRQ** at `arm-smmu.c:794-804` and drops the surrounding `rpm_get` — arguably *worse* for SMMU power, and it does nothing for the cam_tbu problem.)

**The fix is not a version bump — it is one of:**
1. **Skip the diagnostic ATOS — the unblocker (small local patch, GPL source we build).** In `qcom_smmu_context_fault`, the ATOS result (`phys_atos`, from `qcom_smmu_verify_fault` → `qsmmuv500_iova_to_phys`, called **twice**, before/after a TLBIALL) feeds **only a `dev_err` diagnostic** — the fault handling (clear `FSR`, `RESUME`) never uses it. Guard/drop that call and report `FSR`/`FSYNR`/`CBFRSYNRA`/`FAR` + the soft `ops->iova_to_phys()` instead. The context fault becomes **non-fatal → RDI capture usable** despite the IFE bug.
2. **Give `cam_tbu` a controllable clock/power-domain in DT + `pm_runtime`** so the ATOS path can actually re-power it — only viable if such a handle exists (the camera TBU may be implicitly gated by CAMNOC with no independent clock to expose).
3. **Root fix (camera-kernel): IFE RDI-release ordering** so the context fault does not fire after the camera subsystem has powered down.

## Ruled out
- **r1-rel camera-kernel does NOT fix it.** `f7b70309` fixed a *different* variant ("Cannot find vaddr in SMMU ife" in `cam_ife_mgr_release_hw`, verified by a 100/100 soak). This `qsmmuv500_iova_to_phys` external-abort is a distinct fault and survives the full r1-rel branch.
- **Not userspace.** `qtiqmmfsrc`/QMMF merely open→grab→release an RDI stream; the crash is in-kernel.

## Two-level fix (our read, refined by the source walk)
1. **SMMU fault handler (`arm-smmu-qcom-tbu.c`) — the unblocker, GPL source we build.** The TBU clock/halt guard is already in-tree but **inert on SA8775P** (cam_tbu has no DT clock — see above), so the practical fix is to **skip the hardware ATOS** in `qcom_smmu_context_fault` (it is diagnostic-only — `phys_atos` feeds only a `dev_err`) and report `FSR`/`FSYNR`/`CBFRSYNRA`/`FAR` + the soft `ops->iova_to_phys()`. The context fault becomes **non-fatal** → RDI capture usable despite the IFE bug. *This is the unblocker.*
2. **IFE release-path (camera-kernel) — the proper root fix.** Eliminate the buffer-accessed-after-unmap on RDI-only context release. Likely needs a fix beyond r1-rel (r2 / a specific CR).

## Proposed unblocker patch — and what only Qualcomm can resolve
We drafted the skip-ATOS unblocker as a `git am`-ready patch (validated to apply on our r1 source): **`0001-iommu-arm-smmu-qcom-skip-fault-path-ATOS-no-tbu-power.patch`** (attached). One guard in `qsmmuv500_iova_to_phys()`: if the TBU exposes no clock/interconnect the driver can enable (`!tbu->clk && !tbu->path`), skip the hardware ATOS. SoC-agnostic — TBUs with a power handle keep the ATOS unchanged. The caller already handles a `phys == 0` return (reports the fault from the software page-table walk).

**We will do this ourselves (no action needed):** build it into the QLI 1.7 kernel via the `linux-qcom-custom` recipe, flash our IQ-9075, and soak-test RDI capture.

**We cannot resolve these on our end — need Qualcomm (hvo · kkufalk · camera+IOMMU):**
1. **Is the predicate right?** Confirm `!tbu->clk && !tbu->path` correctly identifies "no driver-controllable TBU power" on SA8775P/QCS9075, and that no such TBU is actually always-on where skipping the ATOS would drop a needed diagnostic. (We can't see the TBU power topology.)
2. **Or should `cam_tbu` be *given* a power handle?** Does `cam_tbu@0x151f1000` have a controllable clock/GDSC that should be wired into the DT + a `pm_runtime`/clk get in the ATOS path — making the ATOS *safe* rather than skipped? This is the "proper" alternative; only your HW/power docs answer it.
3. **Land it as a CR** in QLI 1.7 (hvo) and 2.0 (kkufalk) so it's a supported fix, not a private fork we maintain.
4. **The true root fix (camera-kernel):** provide/point to the CR that fixes the IFE RDI-release ordering (the buffer-accessed-after-unmap that raises the context fault). Is it in r2 / 2.0? We can build camera-kernel but can't author this without the IFE internals.
5. **Any hidden dependency on the hard ATOS?** Confirm nothing in your debug/crash-dump tooling consumes the fault-path `phys_atos`, so skipping it loses no required diagnostic.

## Notes for reproduction on your side
- Serial console is `ttyMSM0 @ 115200 8N1` (netconsole is not built: `CONFIG_NETCONSOLE` unset; the watchdog eats the on-disk log, so serial is required to see it).
- We have a reboot-resilient harness (direct-link static IP + auto-restart + checkpointed retry) so we can keep characterizing through the resets — but each RDI capture risks a reset until this is fixed.

**Attachments:** `rdi_smmu_panic_serial.txt` (full trace incl. register dump + module list) · `0001-iommu-arm-smmu-qcom-skip-fault-path-ATOS-no-tbu-power.patch` (proposed unblocker).
