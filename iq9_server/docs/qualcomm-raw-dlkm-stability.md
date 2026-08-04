# Support request: RAW/RDI capture intermittently hangs the CAMSS driver → watchdog reboot (QCS9075)

> ## ✅ RESOLVED 2026-08-04 — confirmed fixed by camera-kernel `f7b70309`
> Rebuilt `cameradlkm` at `camera-kernel.qclinux.1.0.r1-rel` HEAD (`f4491100`, which includes
> `f7b70309` "isp: Fix KMD buffer handle in IFE prepare"), installed on the QLI-1.7 device (vermagic
> matches `6.6.116-qli-1.7-ver.1.1`). A **100/100 live-mode NV12↔RDI handoff soak (3 s quiesce) ran
> clean: 0 reboots, 0 SMMU faults, 1 benign scheduling-delay warning.** The unpatched module (`69cdd66`)
> faulted at ~49 with the `cam_check_iommu_faults` IFE-SMMU trace and 18–31 congestion lines. Root cause
> and fix are as analyzed below: the uninitialized `hw_update_data.kmd_cmd_buff_info` handle drove
> `cam_mem_put_kref` onto the wrong buffer during release → premature IFE-SMMU unmap → page fault;
> `f7b70309` populates the persistent handle correctly. **RAW DAQ is unblocked on QLI 1.7.** (Recommend
> landing the SRCREV bump in the layer + carrying it into QLI 2.0 so it's not a manual module overlay.)


> ## UPDATE 2026-07-31 (b) — `cameradlkm` is OPEN SOURCE; a likely fix is already upstream (correction)
> Earlier text calling the camera kernel driver "prebuilt / not buildable by us" is **WRONG** (thanks hvo).
> `cameradlkm_1.0.qcom.bb` = `inherit module`, GPL-2.0, built from **CodeLinaro `camera-kernel.git`**, branch
> `camera-kernel.qclinux.1.0.r1-rel`, pinned `SRCREV 69cdd66`. Only `camx` + `chicdk` are `qprebuilt`; the
> GStreamer plugin, the QMMF recorder, **and this camera kernel driver are all source we can build.** So the
> IFE-SMMU-on-release fault is **ours (with hvo's kernel build) to patch — not blocked on a Qualcomm binary.**
>
> Better: our pinned rev is **71 commits behind the branch HEAD (`f4491100`)**, and several of those fixes
> target our exact fault class — the `cam_smmu` driver itself is unchanged, so the fault is a buffer-lifecycle
> bug in the layers above it, which these fix:
> - **`f7b70309` msm: camera: isp: Fix KMD buffer handle in IFE prepare** (CRs 4399397/4424334) — KMD buffer
>   handle kept in a stack var, never stored in the persistent `prepare_hw_data`, so `cam_mem_put_kref` runs
>   with an **invalid handle during request cleanup** → ref leaks → **"camera stream freeze after several
>   iterations."** This maps directly onto our "faulting buffer not found on `cam_ife_mgr_release_hw` after
>   ~49 iterations."
> - `f5ba6da2` (isp: RDI-only ctx on non-EPOCH targets), `cf4196d5` (sync: UAF), `56a8c196` (reqmgr: UAF),
>   `0b4e1bf8`/`4dd96c13` (UAF), `8134e5f4` (buffer validation).
>
> **Action (on current QLI 1.7, no need to wait for QLI 2.0):** bump `cameradlkm` `SRCREV` `69cdd66 → f4491100`
> (or cherry-pick `f7b70309` + the UAF fixes), rebuild the DLKM, and re-run the 100-capture soak.
> **Reframed ask to Qualcomm/hvo:** confirm `f7b70309` et al. address this IFE-SMMU-on-release fault, and
> whether moving to `camera-kernel.qclinux.1.0.r1-rel` HEAD is the recommended fix.

> ## UPDATE 2026-07-31 — ROOT CAUSE FOUND: IFE SMMU/IOMMU page fault (use-after-unmap) on RDI release
> A 100-capture soak (live-mode NV12↔RDI handoff, ~3 s release quiesce) ran **49 clean captures, then a
> hard reboot from a kernel ARM-SMMU (IOMMU) context fault in the IFE.** Each capture acquires+releases an
> IFE HW context; at ~capture 49 (IFE ctx ~13) a release **raced an in-flight IFE access to an already
> unmapped buffer** → stage-1 SMMU page fault → watchdog reset. **Captured kernel call trace
> (`camera_qcs9100` DLKM):**
> ```
> irq/40-arm-smmu  pc: cam_check_iommu_faults [camera_qcs9100]
>   cam_smmu_page_fault_work → cam_smmu_iommu_fault_handler → report_iommu_fault
>     → qcom_smmu_context_fault → arm_smmu_context_fault → irq_thread
> CAM-SMMU: smmu fault ids bid:0 pid:0 mid:0 ; Cannot find vaddr:f6e80e80 in SMMU ife virt address
> CAM-CTXT: PF Evt: faulting buffer not found; PF Type: faulting addr out of bounds; PF Stage: stage 1 (non-secure)
> ```
> …logged **concurrently with** `cam_ife_mgr_release_hw: Release HW success ctx 13` and
> `CAM_RELEASE_DEV Success for cmk_imx678`, i.e. during teardown between captures.
>
> **This corrects the earlier framing.** The `cam_ife_csid_ver2_*_bottom_half` "delay in schedule" /
> "WQ congestion" warnings are a *separate, recoverable* symptom (they also fire during plain NV12 preview
> and recover). **The actual reboot is the IFE SMMU page fault on release.** A longer release quiesce only
> *reduces* the odds (it shrinks the race window) — it cannot eliminate it, because this is a
> **use-after-unmap race in the RDI release path**, not a fixed-timing issue. Observed rate ≈ 1 hang per
> ~49 RDI acquire/release cycles at 3 s quiesce (was far more frequent with short quiesce / incomplete
> release). NV12-only (no repeated RDI acquire/release) is unaffected.
>
> **Ask for Qualcomm/hvo (specific DLKM bug):** the IFE SMMU/IOMMU context fault above during repeated RDI
> HW-context acquire/release — is it known on QCS9075 `cameradlkm` (`camera_qcs9100`), and is there a
> fixed DLKM/SPF? The fault (`cam_check_iommu_faults` / `cam_smmu_iommu_fault_handler`, "faulting buffer
> not found / addr out of bounds" on the **ife** context bank, during `cam_ife_mgr_release_hw`) points to a
> buffer being unmapped from the IFE SMMU while an IFE access is still in flight on RDI teardown.
> The original blocker framing below is superseded; the config/repro detail remains accurate.


**Summary.** On QCS9075 (IQ-9075 EVK, Qualcomm Linux 1.7) we can capture native RAW Bayer from the
custom IMX678 sensor via `qtiqmmfsrc` (`video/x-bayer, RAW16, 3856×2180`) — it produces **valid 12-bit
RGGB** data. However, RAW/RDI capture is **not reliable**: the same request intermittently yields (a) a
valid frame, (b) no frame, or (c) a **hard hang of the camera subsystem that trips the 30 s `qcom_wdt`
watchdog and reboots the board**. The kernel log at the point of failure shows the CAMSS/CSID ISP path
going into scheduling congestion and SOF-recovery "bubble" churn. Because the hang ends in a hard
watchdog reset, nothing is flushed to disk (no panic, no oops). We have localized the failure to the
**kernel camera driver** (shipped to us as the prebuilt `cameradlkm` DLKM), not the userspace CamX/CHI
or the open-source `qtiqmmfsrc`/QMMF recorder. **We request confirmation of whether this is a known
QCS9075 CAMSS/RDI issue and an updated camera DLKM / SPF or the correct RDI usage/constraints.**

---

## Environment

| | |
|---|---|
| SoC / board | QCS9075, IQ-9075 EVK |
| OS | Qualcomm Linux 1.7-ver.1.1 |
| Kernel | `6.6.116-qli-1.7-ver.1.1-05801-gde229c16e2aa-dirty` |
| SPF / SDK | LE.QCLINUX.1.0.r1 / QIM Product SDK `qimpsdk-r1.0_00114.0` (prebuilt build id `r1.0_00114.0`) |
| Camera userspace | CamX + `cam-server` + `qtiqmmfsrc` (GStreamer) — QMMF recorder built from `le-services` |
| Camera kernel driver | `cameradlkm` (prebuilt DLKM, `cameradlkm_1.0.qcom.bb`), module tag `camera_qcs9100` |
| Sensor | LI-IMX678-FLEX-114H (Sony IMX678), driver `cmk_imx678`; one mode: **3856×2180, 12-bit, RGGB, 30 fps**, `dt=0x2C` (RAW12), MIPI CSI-2 |
| Watchdog | `qcom_wdt`, hardware timeout 30 s, systemd-managed |

---

## What works vs. what fails

**NV12 (ISP-processed) preview — stable.** Continuous live view runs indefinitely via the CHI
`RealTimeFeatureZSLPreviewRawYUV` usecase. No stability issue.

**RAW16 Bayer — validated but unstable.** The correct request is RAW16 at native resolution:
```bash
gst-launch-1.0 -e qtiqmmfsrc ! \
  "video/x-bayer,format=rggb,bpp=(string)16,width=3856,height=2180,framerate=30/1" ! \
  identity eos-after=2 ! filesink location=/tmp/r16.bin
```
(RAW10 is rejected earlier by `CheckValidStreamConfig` — advertised max 3840×2160 < sensor 3856×2180 —
and RAW12 destabilizes `cam-server`; RAW16 is the working format and yields verified 12-bit RGGB:
`max=4095`, `G1≈G2`, inter-row correlation 0.988.)

The instability appears when the RDI stream actually **delivers a frame**. Two outcomes from the
identical request, repeated: a valid RAW frame, or a **hard hang → `qcom_wdt` reboot** during/just after
the capture. Crucially, the hang occurs even from a **COLD / idle camera** (a fresh `qtiqmmfsrc` RDI
pipeline with no concurrent preview) — not only across a preview→RDI transition, though the transition
makes it worse. So this is not merely a reconfigure-ordering issue; steady RDI frame delivery itself can
hang the CSID.

(An earlier "no frame" outcome was a **client-side bug on our side**, not a platform symptom:
`identity eos-after=1` fires EOS as the first buffer passes and tears the sink down before it is written
— `eos-after=1` yields 0 bytes, `eos-after=2` yields a full frame. Fixed with `eos-after=n+1`. Mentioned
only so it is not confused with the hang.)

The primary evidence below is not the reboot tally (some resets on this bring-up unit had unrelated
causes — e.g. cable reroutes) but the **kernel CAMSS signature captured in the ring buffer at the moment
of the RAW capture**, via an on-device `/dev/kmsg` trigger marker placed immediately before the capture.

Operational note: after such a watchdog reboot, `cam-server` sometimes comes back **wedged** — any process
that opens the camera then blocks — until `systemctl restart cam-server` is run. Recovering the rig after
a RAW hang therefore needs a cam-server restart, not just a webui restart.

---

## Evidence (kernel ring buffer, captured on-device via `dmesg -wT` with a `/dev/kmsg` trigger marker)

Immediately after the RAW capture is triggered, the CAMSS driver `camera_qcs9100` reports growing ISP
tasklet scheduling delays, workqueue congestion with dropped frames, and SOF-recovery "bubble" churn:

```
CAM_WARN CAM-UTIL cam_common_util_thread_switch_delay_detect:184
    ISP Tasklet cb: cam_ife_csid_ver2_ipp_bottom_half [camera_qcs9100]
    delay in schedule detected ... diff 7 : threshold 5     (then diff 14, then diff 27)
CAM_INFO CAM-CRM  __cam_req_mgr_find_dev_name:319
    WQ congestion, Skip Frame: req ... not ready on link ... dev: cam-sensor open_req count: 3
CAM_ERR  CAM-ISP  __cam_isp_ctx_recover_sof_timestamp:1502  erroneous call to SOF recovery ...
CAM_WARN CAM-ISP  __cam_isp_ctx_send_sof_timestamp:1637      Missed SOF Recovery for invalid req ...
CAM_WARN CAM-ISP  __cam_isp_ctx_notify_error_util:918        Notify CRM about bubble req ...
CAM_WARN CAM-ISP  __cam_isp_ctx_reg_upd_in_epoch_bubble_state:3177  Unexpected regupdate in Substate[BUBBLE]
```

The `cam_ife_csid_ver2_ipp_bottom_half` scheduling delay grows monotonically (7 → 14 → 27, threshold 5),
i.e. the CSID interrupt bottom-half is being progressively starved. When it recovers we get "no frame";
when it does not, the CSID/ISP path stalls and the 30 s `qcom_wdt` fires. The reset is a **hard watchdog
reset**: no kernel panic/oops/Call-trace is logged, `/var/log/messages` shows normal camera activity and
then the boot banner, and there is no `pstore`/`ramoops` region configured to retain a last-gasp dmesg.

Userspace context at failure (from `cam-server`): the RAW request passes `configure_streams`
(`HAL_PIXEL_FORMAT_RAW16`, 3856×2180) — this is **not** a stream-config rejection; the failure is in the
subsequent RDI streaming/reconfigure in the kernel driver.

---

## Analysis

The failure is in the **kernel CAMSS/CSID driver** (`camera_qcs9100`, delivered as the prebuilt
`cameradlkm` DLKM), in the IFE-CSID RDI (RAW-dump) path:
- the CSID bottom-half tasklet is being starved under RDI streaming (growing schedule delay),
- the request manager workqueue congests and drops SOFs,
- the ISP enters a bubble/recovery loop it does not always exit, and a full stall trips the watchdog.

This is consistent with an RDI reconfigure / ISP-bandwidth / clock-voting or tasklet-serialization
robustness issue for full-resolution 12-bit RDI on this SoC/DLKM. Everything above the kernel driver is
either open-source and correct (QMMF recorder / `qtiqmmfsrc`) or a passing `configure_streams`, so we do
not believe this is a userspace or usecase-selection defect.

---

## What we are asking Qualcomm

1. Is this a **known CAMSS/CSID RDI issue** on QCS9075 / QLI-1.7 / SPF `r1.0_00114.0` (or the
   `cameradlkm` for `camera_qcs9100`)? If so, is there a **fixed camera DLKM / SPF** we can take?
2. Are there **required constraints / configuration for full-res 12-bit RDI** we may be violating —
   e.g. mandatory quiesce/settle between tearing down a preview (ZSL) session and starting an RDI
   stream, ISP clock/bandwidth (bus) voting for RDI, a CSID/IFE tasklet or WQ setting, or a limit on
   RDI framerate/resolution?
3. Is RDI/RAW **validated on this platform+sensor bring-up** at all, or is it expected to require
   additional camera-driver enablement for `cmk_imx678`?
4. What **console trace** would you want to root-cause the hang? The console is `ttyMSM0 @ 115200 8N1`;
   we can attach a serial console and provide the full trace through the watchdog reset if useful.

## Reproduction for Qualcomm

1. Boot the IQ-9075 EVK with the `cmk_imx678` sensor (3856×2180 / 12-bit / RGGB / 30 fps).
2. (Optional, worsens it) start an NV12 preview session, let it run a few seconds, stop it.
3. Run the RAW16 `gst-launch` line above.
4. Observe: intermittent valid-frame / no-frame / **`qcom_wdt` reboot**, with the CAMSS log signature
   above in the kernel ring buffer.

## References

- Working RAW recipe + corrected stream-config analysis: `raw-enablement.md` (same repo).
- Retracted earlier case (usecase selector — wrong root cause): `qualcomm-raw-support-case.md`.
- Kernel signature source: `cam_ife_csid_ver2_ipp_bottom_half`, `__cam_req_mgr_find_dev_name`,
  `__cam_isp_ctx_recover_sof_timestamp` (CAMSS driver `camera_qcs9100`).
