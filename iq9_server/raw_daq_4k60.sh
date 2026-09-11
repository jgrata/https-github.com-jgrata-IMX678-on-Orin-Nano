#!/bin/bash
# IQ9 4K@60 12-bit RAW DAQ / GMSL link-stress capture.
#
# Confirmed on QCS9075 + IMX678: 3856x2180 12-bit RAW @ 60fps, 0 drops (~6.0 Gbps),
# matching the Jetson e-CAM86 GMSL test case. The dual-pad webui (NV12 live + RAW) caps
# at 30fps (NV12/IPE path); the RAW-ONLY pipeline (RDI-direct / UsecaseRaw, which follows
# the sensor mode) is the 60fps route -- so this stops the iq9cam daemon for exclusive
# camera access, runs RAW@60, then restores the daemon.
#
# Prereqs (already deployed): the multimode sensor bin (idx0=4K12@30, idx1=4K12@60,
# idx2=4K10@30, idx3=4K10@60; md5 2991e3a9e036) at
# /usr/lib/camx/lemans/camera/com.qti.sensormodule.cmk_imx678_cam0.bin  (stock backup at
# /var/li_backup/*.orig -> restore with iq9_server/sensor_driver/deploy.sh restore).
#
#   ./raw_daq_4k60.sh [seconds]     measure/stress (default 15s, to fakesink)
#   ./raw_daq_4k60.sh [seconds] /path/prefix   also WRITE frames (multifilesink f_%05d.bin)
#
# Notes: 4K@72 is NOT reachable (IFE throughput ceiling, key-10014 SEGV); 10-bit 4K RAW
# is NOT reachable (RDI wired 12-bit only). 60fps/12-bit is the IQ9 ceiling = the GMSL case.
set -uo pipefail
SECS="${1:-15}"; OUTPREFIX="${2:-}"
export XDG_RUNTIME_DIR=/run/user/0
CAPS='video/x-bayer,format=rggb,bpp=(string)16,width=3856,height=2180,framerate=60/1'

echo "== force sensor mode idx1 (4K 12-bit 60fps) =="
echo overrideForceSensorMode=1 > /var/cache/camera/camxoverridesettings.txt
echo "== stop iq9cam (exclusive camera for RAW-only) =="
systemctl stop iq9cam; sleep 3

if [ -n "$OUTPREFIX" ]; then
  echo "== RAW 4K@60 -> ${OUTPREFIX}f_%05d.bin for ${SECS}s =="
  timeout "$SECS" gst-launch-1.0 -e qtiqmmfsrc ! "$CAPS" \
    ! multifilesink location="${OUTPREFIX}f_%05d.bin" 2>&1 | tail -3
  echo "wrote: $(ls -1 ${OUTPREFIX}f_*.bin 2>/dev/null | wc -l) frames"
else
  echo "== RAW 4K@60 measure (fakesink) for ${SECS}s =="
  timeout "$SECS" gst-launch-1.0 -e -v qtiqmmfsrc ! "$CAPS" \
    ! fpsdisplaysink text-overlay=false video-sink=fakesink sync=false 2>&1 \
    | grep -oE "current: [0-9.]+, average: [0-9.]+|dropped: [0-9]+" | tail -6
fi

echo "== restore iq9cam daemon =="
systemctl start iq9cam
echo "done. (sensor stays forced to idx1; reset with: iq9_server/set_sensor_mode.sh auto)"
