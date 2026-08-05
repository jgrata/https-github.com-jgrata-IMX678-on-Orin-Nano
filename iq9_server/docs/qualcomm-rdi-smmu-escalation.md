# IMX678 RDI capture → kernel panic in the Qualcomm SMMU driver (`qsmmuv500_iova_to_phys`)

**For:** hvo (QLI 1.7) · kkufalk (QLI 2.0) · Qualcomm camera/IOMMU
**Date:** 2026-08-05 · **Reporter:** Metropolis (Jeremy Grata)
**Full serial trace attached:** `rdi_smmu_panic_serial.txt`

## TL;DR / asks
Capturing native RAW (RDI/bayer) from the IMX678 on QCS9075 reliably-intermittently **panics the kernel inside the ARM-SMMU driver** — the SMMU **fault handler itself** takes a synchronous external abort in `qsmmuv500_iova_to_phys`, and the `qcom_wdt` watchdog resets the SoC. It is **not** the camera DLKM crashing, and it is **not** fixed by the r1-rel camera-kernel (which we run, incl. `f7b70309`). We need:
1. **Known fix?** Is there a Qualcomm CR / patch for `qsmmuv500_iova_to_phys` taking a **synchronous external abort while servicing a context fault** (classic ATOS-on-an-unclocked-TBU)?
2. **Handler hardening (high value):** can the context-fault handler be made **non-fatal** (report `FSR`/`FAR` without the HW ATOS translation)? That alone would turn the IFE fault into a recoverable dropped frame instead of a reboot — **making RDI capture usable now**.
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

## Ruled out
- **r1-rel camera-kernel does NOT fix it.** `f7b70309` fixed a *different* variant ("Cannot find vaddr in SMMU ife" in `cam_ife_mgr_release_hw`, verified by a 100/100 soak). This `qsmmuv500_iova_to_phys` external-abort is a distinct fault and survives the full r1-rel branch.
- **Not userspace.** `qtiqmmfsrc`/QMMF merely open→grab→release an RDI stream; the crash is in-kernel.

## Two-level fix (our read)
1. **SMMU fault handler (`arm-smmu-qcom` / qcom SMMU impl) — highest value, and it's GPL source we can build.** Guard the fault-path `iova_to_phys`: skip the HW ATOS (or ensure the TBU clock/power is on first) and report `FSR`/`CBFRSYNRA`/`FAR` instead. Result: the context fault is **reported, non-fatal** → RDI capture usable despite the IFE bug. *This is the unblocker.*
2. **IFE release-path (camera-kernel) — the proper root fix.** Eliminate the buffer-accessed-after-unmap on RDI-only context release. Likely needs a fix beyond r1-rel (r2 / a specific CR).

## Notes for reproduction on your side
- Serial console is `ttyMSM0 @ 115200 8N1` (netconsole is not built: `CONFIG_NETCONSOLE` unset; the watchdog eats the on-disk log, so serial is required to see it).
- We have a reboot-resilient harness (direct-link static IP + auto-restart + checkpointed retry) so we can keep characterizing through the resets — but each RDI capture risks a reset until this is fixed.

**Attachment:** `rdi_smmu_panic_serial.txt` (full trace incl. register dump + module list).
