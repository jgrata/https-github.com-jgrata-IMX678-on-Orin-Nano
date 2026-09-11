#!/usr/bin/env bash
# Reboot-free IMX678 sensor-mode (resolutionData index) switch on the IQ9.
#
# The multi-mode sensor bin (sensor_driver/build_modes.py) carries these resolutionData indices:
#   0 = 4K 12-bit 30fps    1 = 4K 12-bit 60fps    2 = 4K 10-bit 30fps    3 = 4K 10-bit 60fps
#
# CamX auto-selects the sensor mode from the requested CAPS = resolution + framerate ONLY (bit-depth
# is invisible: the bayer pad packs 10/12-bit into bpp=16). To pin a SPECIFIC index we set CamX's
# `overrideForceSensorMode` in the CamX override-settings file and restart cam-server. This is the
# CHI ExtensionModule OverrideForceSensorMode -> DesiredSensorMode.forceMode -> FindBestSensorMode
# force path (traced in QLI chi-cdk-kt). Reboot-free (~15s); only a .bin SWAP needs a reboot.
#
#   overrideForceSensorMode is read ONLY at cam-server start -> we restart cam-server (+ iq9cam).
#   File path found by stracing cam-server: /var/cache/camera/camxoverridesettings.txt (also ./).
#
# CAVEAT (10-bit): a forced 10-bit index (2/3) streams via the NV12/ISP path but the bayer RAW pad
# FAILS to open for every bpp (10/12/16) in this build -- the RDI/IFE RAW path is wired for the
# 12-bit RAW16 container. So 10-bit is usable via NV12 today; 10-bit *RAW* capture needs the RDI
# 10-bit path sorted first. 12-bit indices (0/1) give both RAW16 and NV12.
#
#   usage:  set_sensor_mode.sh <0|1|2|3|auto>
set -u
OVR=/var/cache/camera/camxoverridesettings.txt
IDX="${1:-}"
declare -A NAME=([0]=4K12b30 [1]=4K12b60 [2]=4K10b30 [3]=4K10b60)
case "$IDX" in
  ''|-h|--help) echo "usage: $0 <0|1|2|3|auto>   (0=4K12b30 1=4K12b60 2=4K10b30 3=4K10b60)"; exit 2;;
  auto|-1)  rm -f "$OVR"; echo "cleared forced mode -> AUTO (caps-selected, default 12-bit)";;
  0|1|2|3)  mkdir -p "$(dirname "$OVR")"; printf 'overrideForceSensorMode=%s\n' "$IDX" > "$OVR"
            echo "forced sensor mode $IDX (${NAME[$IDX]}) via $OVR";;
  *) echo "invalid mode: $IDX (want 0|1|2|3|auto)"; exit 2;;
esac
# cam-server reads the override at start; iq9cam re-acquires the (single dual-pad) camera after.
systemctl restart cam-server 2>/dev/null; sleep 4
systemctl restart iq9cam 2>/dev/null;   sleep 8
sel=$(journalctl -u cam-server -b 0 --no-pager --since '25 sec ago' 2>/dev/null \
        | grep -oE 'SensorMode:[0-9]+ \(WxH:[0-9x]+, FPS:[0-9.]+' | tail -1)
fd=$(grep -oE '"frame_duration_ns": [0-9]+' /dev/shm/iq9_nv12.meta 2>/dev/null | grep -oE '[0-9]+')
echo "active: ${sel:-<none logged>}  frame_dur=${fd:-?}ns  iq9cam=$(systemctl is-active iq9cam) cam-server=$(pgrep -x cam-server || echo NONE)"
