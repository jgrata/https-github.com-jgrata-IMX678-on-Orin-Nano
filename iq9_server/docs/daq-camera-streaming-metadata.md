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
1. **Standalone `iq9cam` daemon** — ✅ DONE (commit de38751). Dual-pad daemon runs as its own
   service (owns the camera from boot); webui is a pure shm consumer. `systemctl restart iq9web` no
   longer wedges the camera → webui deploys need no reboot.
2. **Metadata** — ✅ DONE (commit c84246b). Per-frame CamX result metadata + sensor timestamp
   published; `/api/frame_meta` composes the full requested-vs-actual provenance record. See
   **How the metadata is actually extracted** below.
3. **RTSP** — ✅ DONE (commits aae983e, 12eacca). 3 HW-encoded streams TEED off the SAME camera
   session (single-client): the two 1080p tee off the NV12 pad (no extra camera stream), the 4k uses
   its own pad (video_2). Each = leaky queue → `v4l2{h264,h265}enc` (dmabuf-import) →
   `{h264,h265}parse config-interval=1` → `qtirtspbin` (own port + mount):
   `rtsp://<board>:8554/h264-1080`, `:8555/h265-1080`, `:8556/h264-4k` (bound 0.0.0.0). Validated:
   all three deliver frames while NV12 + RAW + metadata stay live; CamX/Venus hold 3 streams + 3
   encodes. **Two gotchas:** (a) `qtirtspbin`'s RTSP server services clients from GLib main-loop
   callbacks — the daemon's blocking appsink-pull loop has none, so run a `GLib.MainLoop` in a daemon
   thread or the server accepts TCP but never answers OPTIONS (client timeout). (b) `v4l2*enc` runs
   CONTINUOUSLY (extra SoC heat) — set `IQ9_RTSP=0` + reboot for temperature-sensitive characterization.
   `/api/info` surfaces the stream URLs. Gated by `IQ9_RTSP` / `IQ9_RTSP_4K` in iq9cam.service.

## How the metadata is actually extracted (the non-obvious part)
The buffer meta (`attach-cam-meta` / `GstCameraMeta` in `libgstqticamerabase`) is a proprietary C
struct with **no header/typelib**, so it is NOT readable from Python. Instead we read the
**`result-metadata` element signal** (emitted per frame with a `G_TYPE_POINTER` to a
`qmmf::CameraMetadata`):
- PyGObject marshals the pointer as a **`GPointer` wrapper** whose `int()` raises. The wrapped C
  pointer is at **offset 16** of the CPython object (offset 24 holds the gtype `0x44` = G_TYPE_POINTER,
  which confirms 16). Read it with `ctypes.c_void_p.from_address(id(ptr)+16)`.
- ctypes-call `qmmf::CameraMetadata::getbuffer()` (mangled `_ZN4qmmf14CameraMetadata9getbufferEv` in
  `libqmmf_camera_metadata.so.1`) → the serialized **Android `camera_metadata_t`** (its first u32 =
  total size; copy that many bytes).
- `cam_meta.py` parses it: **all-uint32 header, 48 bytes** (size, version, flags, entry_count,
  entry_capacity, entries_start=48, data_count, data_capacity, data_start, padding, then u64
  vendor_id), 16-byte entries (tag u32, count u32, data u32/inline, type u8). Tag NAMES resolve
  authoritatively from `libcamera_metadata_lemans.so.0` (QCS9075 = "lemans") via
  `get_camera_metadata_tag_name`. **Pure Python → the parser iterates with no daemon reboot.**
- **CRASH SAFETY:** a wrong `this` fed to the C++ `getbuffer` segfaults the whole process
  (uncatchable in Python), so the extractor uses ONLY the confirmed offset 16, guarded to look like a
  userspace VA.

Key tags (confirmed on this build): `0x00000001` transform=CCM (rat[9]), `0x00000002` gains=AWB
(f32[4]), `0x000e0000` exposureTime (i64 ns), `0x000e0001` frameDuration, `0x000e0002` sensitivity
(ISO), `0x000e0010` sensor timestamp, `0x000e001c` dynamicBlackLevel (f32[4]), `0x000e001d`
dynamicWhiteLevel, `0x000d0000` cropRegion, `0x000c0000` frameCount.

Schema is intentionally open — add/drop fields as the ISP pipeline + optics evolve.
