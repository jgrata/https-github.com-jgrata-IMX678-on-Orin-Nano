# IQ9 camera DAQ — streaming + provenance metadata (design)

Locked 2026-09-01. Extends the dual-pad daemon ([[imx678-iq9-2.0-camera-wedge]]). The 2.0 camera is
single-client (one `qtiqmmfsrc` session), so EVERYTHING — RAW DAQ, NV12 live, RTSP, metadata — must
come from that one session. A separate pipeline = the open/close CCI wedge. The daemon is the hub.

## Architecture
`iq9cam` = **standalone systemd service** (own the camera from boot; decouples webui deploys from
the camera so webui restarts stop needing a reboot). One `qtiqmmfsrc`, pads:
- `video_0` → NV12 → `/dev/shm/iq9_nv12`  (live view / field-map)
- `video_1` → RAW16 → `/dev/shm/iq9_raw`  (characterization; published only while `iq9_raw.on`)
- `video_2/3/4` → HW encode (`qtismartvencbin`) → `qtirtspbin` ×3:
  `rtsp://IQ9:8554/h264-1080`, `:8555/h265-1080`, `:8556/h264-4k`  (verify Venus holds all + RAW @30fps)
- `attach-cam-meta=true` → per-frame CamX result metadata → the metadata record below

The webui (iq9web) becomes a pure shm consumer + control-file writer (raw-flag, exposure-comp); it
never opens/kills the camera. `GstRtspServer` python lib is absent — use native `qtirtspbin`.
Confirmed: 30 fps on NV12 + RAW simultaneously; `attach-cam-meta` exposes actual sensor settings.

## Metadata record (per frame/capture)
`requested` + `actual` wherever they can differ (e.g. gain req 1.0006 vs act 1.000599; focus cmd vs achieved).
```
timing:      frame_seq, sensor_ts_ns(SOF/PTS), host_recv_ns
sensor:      bit_depth, exposure_ns{req,act}, gain_iso{req,act}, conv_gain(HCG|LCG),
             roi{req,act}, hdr_mode{req,act: linear|DCG|DOL|ClearHDR}, fps{req,act},
             black_level, sensor_temp_c
lens_focus:  type(fixed|liquid|EM), focus_pos{req,act}, focus_distance_m, lens_temp_c,
             calib_id, intrinsics{fx,fy,cx,cy}, distortion{radial[k1,k2,k3], tangential[p1,p2]}
product:     type(RAW16 | NV12-ISP | H264-RTSP | H265-RTSP…), resolution, pixfmt
isp[]:       each task = {applied, source: off|vendor-default|user-custom, id/version, params}
             DPC, BLC, LSC, CAC, LDC(uses lens_focus.distortion), demosaic,
             AWB{gains,CCT}, CCM{illuminant/CCT-bin, matrix}, WDR/DRC, HDR_fusion(DCG/DOL),
             GTM{curve}, LTM{params}, gamma/OETF{curve}, sharpen{kernel/strength},
             NR(2D/3D/MFNR){strength}, chroma/color-enhance, binning/scaling, EIS
provenance:  tuning_scenario(Chromatix), pipeline_version
```
- RAW products: every `isp[]` task `off` (raw sensor data). Each stage flips to
  `vendor-default`/`user-custom` with its config as ISP tasks come online; the exact CCM /
  tone-curve / LUT used is always recoverable.
- **Optics geometry is keyed to focus.** A liquid lens (driver on the IQ9) has per-focus-position
  intrinsics + radial/tangential distortion → a per-focus **calibration table** on the IQ9;
  `lens_focus` records the active `focus_pos → calib_id` and LDC/CAC consume those coefficients so
  post-processing undistorts for the exact focus each frame was shot at. Fixed lens = one calib entry.
- EIS, if ever active, is flagged since it perturbs effective intrinsics.

## Build phases (each = one reboot + validation; daemon restart re-wedges, so daemon iters need a boot)
1. **Standalone `iq9cam` daemon** — move the dual-pad daemon to its own service; webui → pure shm reader.
2. **Metadata** — `attach-cam-meta=true`; probe the exact CamX field names; parse actual settings +
   the ISP/CCM provenance; publish the record (shm sidecar + `/api/frame_meta`).
3. **RTSP** — 3 encode→`qtirtspbin` branches; verify Venus capacity.

Schema is intentionally open — add/drop fields as the ISP pipeline + optics evolve.
