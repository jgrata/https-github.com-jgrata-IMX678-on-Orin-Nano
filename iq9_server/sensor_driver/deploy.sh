#!/bin/bash
# Activate an IMX678 sensor variant on the IQ9, or restore the shipping sensor.
#
#   ./deploy.sh <variant>     e.g. cmk_imx678_cam0_hcg   (compile -> install -> validate)
#   ./deploy.sh restore                                   (revert to the shipping .bin)
#   ./deploy.sh status                                    (show what's installed)
#
# The variant's sensor XML is compiled to the *cam0 identity* (output name
# com.qti.sensormodule.cmk_imx678_cam0.bin) so the on-device bin's internal tag matches
# what CamX/socid_map expect — installing a differently-tagged bin risks the SIGSEGV seen
# in the LI bring-up thread. The shipping bin is backed up once; `restore` puts it back.
#
# Deploying a new sensor config restarts cam-server. The PC resilience supervisor
# (iq9_client.py) is used to confirm the board came back and can stream.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CMD="${1:-status}"

# QLI 2.0 (lemans): root is passwordless and the sensormodule .bin lives under camx/lemans/camera.
# (1.7 was metro@ + /usr/lib/camera.) A .bin swap needs a reboot for a clean bring-up in the new
# mode on 2.0 -- IQ9_DEPLOY_POST=restart to try a cam-server restart instead (reconfigure test).
HOST="${IQ9_HOST:-192.168.99.2}"; SSHH="root@$HOST"
DEVDIR="${IQ9_DEVDIR:-/usr/lib/camx/lemans/camera}"
DEVBIN="$DEVDIR/com.qti.sensormodule.cmk_imx678_cam0.bin"
POST="${IQ9_DEPLOY_POST:-reboot}"
BACKUP="/var/li_backup/com.qti.sensormodule.cmk_imx678_cam0.bin.orig"
CHICDK="${CHICDK:-/c/Users/JGrata/iq9075-ref/qualcomm-toolchain-mm/qualcomm-linux-spf-1-0_ap_standard_oem_nm-qimpsdk-r1.0_00114.0/qualcomm-linux-spf-1-0_ap_standard_oem_nm-qimpsdk-r1.0_00114.0-cc0652ada55b237510884c34fc4dd2f3f8a2201d/LE.QCLINUX.1.0.r1/apps_proc/sources/vendor/qcom/proprietary/chi-cdk}"
MODULE="$HERE/baseline/cmk_imx678_module_cam0.xml"
PY="${PY:-python}"

ssh_do() { ssh -o ConnectTimeout=15 "$SSHH" "$@"; }
towsl() { case "$1" in /mnt/*) echo "$1";; /[a-zA-Z]/*) echo "/mnt/${1:1}";; *) echo "$1";; esac; }

if [ "$CMD" = "status" ]; then
  ssh_do 'echo "installed cam0 md5: $(md5sum '"$DEVBIN"' 2>/dev/null | cut -c1-12)";
          echo "backup present:    $(test -f '"$BACKUP"' && md5sum '"$BACKUP"' | cut -c1-12 || echo none)";
          echo "cam-server:        $(pgrep -x cam-server >/dev/null && echo running || echo stopped)"'
  exit 0
fi

if [ "$CMD" = "restore" ]; then
  echo "== restoring shipping sensor bin =="
  ssh_do 'set -e; test -f '"$BACKUP"' || { echo "no backup found"; exit 1; }
          mount -o remount,rw /usr 2>/dev/null || true
          rm -f '"$DEVBIN"'; cp '"$BACKUP"' '"$DEVBIN"'; sync
          echo "restored: $(md5sum '"$DEVBIN"' | cut -c1-12); rebooting"
          ( sleep 2; systemctl reboot ) >/dev/null 2>&1 &'
  exit 0
fi

VARIANT="$CMD"
SRCXML="$HERE/generated/${VARIANT}_sensor.xml"
[ -f "$SRCXML" ] || { echo "ERROR: no generated XML for '$VARIANT' ($SRCXML). Run build_variants.py."; exit 1; }

# 1) compile this variant to the cam0 identity (internal tag = cam0)
STAGE="$HERE/.deploy_stage"; mkdir -p "$STAGE"
cp -f "$SRCXML" "$STAGE/cam0_deploy_sensor.xml"
OUTBIN="$STAGE/com.qti.sensormodule.cmk_imx678_cam0.bin"
rm -f "$OUTBIN"
read -r -d '' INNER <<'SH' || true
set -e
CHICDK="$1"; XML="$2"; MODULE="$3"; OUT="$4"
PP="$CHICDK/api/tools/buildbins/linux64/ParameterParser"
SDIR="$CHICDK/oem/qcom/sensor/cmk_imx678"; MDIR="$CHICDK/oem/qcom/module"
[ -x "$PP" ] || chmod +x "$PP" 2>/dev/null || true
mkdir -p "$SDIR" "$MDIR"
cp -f "$XML" "$SDIR/cam0_deploy_sensor.xml"; cp -f "$MODULE" "$MDIR/"
"$PP" "$OUT" b "$SDIR/cam0_deploy_sensor.xml" "$MDIR/$(basename "$MODULE")" -q 2>&1 | tail -2
SH
echo "== compiling $VARIANT -> cam0 identity =="
if uname -s | grep -qiE 'mingw|msys|cygwin'; then
  MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc "$INNER" _ \
    "$(towsl "$CHICDK")" "$(towsl "$STAGE/cam0_deploy_sensor.xml")" "$(towsl "$MODULE")" "$(towsl "$OUTBIN")"
else
  bash -c "$INNER" _ "$CHICDK" "$STAGE/cam0_deploy_sensor.xml" "$MODULE" "$OUTBIN"
fi
[ -s "$OUTBIN" ] || { echo "ERROR: compile produced no bin"; exit 1; }
echo "   built $(stat -c%s "$OUTBIN") bytes"

# 2) push + install (back up the shipping bin once), restart cam-server
echo "== installing on $HOST =="
scp -o ConnectTimeout=15 "$OUTBIN" "$SSHH:/tmp/cam0_variant.bin" >/dev/null
ssh_do 'set -e
  mkdir -p /var/li_backup
  test -f '"$BACKUP"' || cp '"$DEVBIN"' '"$BACKUP"'      # one-time backup of the shipping bin
  mount -o remount,rw /usr 2>/dev/null || true
  rm -f '"$DEVBIN"'; cp /tmp/cam0_variant.bin '"$DEVBIN"'; sync
  echo "installed: $(md5sum '"$DEVBIN"' | cut -c1-12)  (backup: $(md5sum '"$BACKUP"' | cut -c1-12))"
  if [ "'"$POST"'" = restart ]; then systemctl restart cam-server; sleep 4;
  else echo rebooting; ( sleep 2; systemctl reboot ) >/dev/null 2>&1 & fi'

echo "deployed variant: $VARIANT (post=$POST); board coming back — validate manually via /api/frame_meta"
echo "  restore with './deploy.sh restore'."
