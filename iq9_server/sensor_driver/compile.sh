#!/bin/bash
# Compile the generated IMX678 sensor variants -> com.qti.sensormodule.*.bin using the
# LOCAL QLI 1.7 ParameterParser. No hvo / build-host / CreatePoint round-trip needed.
#
# PROVEN: recompiling the baseline reproduces the shipping .bin byte-for-byte
# (md5 eb236184e9df); the HCG variant differs only in the conversion-gain register + name tag.
#
# We call ParameterParser DIRECTLY (not buildbins.py) with the XMLs staged in the top-level
#   <chicdk>/oem/qcom/{sensor/cmk_imx678,module}
# where each XML's `../../../../api/sensor` schema path resolves. On Windows the parser is run
# through WSL (Linux has no MAX_PATH limit; the toolchain lives at a >260-char path). On Linux
# it runs natively.
#
#   python build_variants.py     # -> generated/*_sensor.xml
#   ./compile.sh                 # -> bins/com.qti.sensormodule.cmk_imx678_cam0*.bin
#   ./deploy.sh <variant>        # activate one on the IQ9
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

# chi-cdk root, in this shell's path form. Override CHICDK to relocate the toolchain.
# MUST be the 2.0 chi-cdk PP (the 1.7 PP makes bins 2.0 CamX SIGSEGVs on -- bad symbol table).
CHICDK="${CHICDK:-/c/Users/JGrata/iq9075-2.0/built-chi-cdk/chi-cdk}"
GEN="$HERE/generated"
MODULE="$HERE/baseline/cmk_imx678_module_cam0.xml"
OUT="${OUT:-$HERE/bins}"

# /c/x -> /mnt/c/x for WSL; leave already-/mnt paths and Linux paths unchanged.
towsl() { case "$1" in /mnt/*) echo "$1";; /[a-zA-Z]/*) echo "/mnt/${1:1}";; *) echo "$1";; esac; }

read -r -d '' INNER <<'SH' || true
set -e
CHICDK="$1"; GEN="$2"; MODULE="$3"; OUT="$4"
PP="$CHICDK/api/tools/buildbins/linux64/ParameterParser"
SDIR="$CHICDK/oem/qcom/sensor/cmk_imx678"; MDIR="$CHICDK/oem/qcom/module"
[ -x "$PP" ] || chmod +x "$PP" 2>/dev/null || true
[ -f "$PP" ] || { echo "ERROR: ParameterParser not found at $PP"; exit 1; }
mkdir -p "$SDIR" "$MDIR" "$OUT"
cp -f "$GEN"/*_sensor.xml "$SDIR"/
cp -f "$MODULE" "$MDIR"/
mbase="$(basename "$MODULE")"
rc=0
for x in "$SDIR"/*_sensor.xml; do
  v="$(basename "$x" _sensor.xml)"
  out="$OUT/com.qti.sensormodule.$v.bin"
  if "$PP" "$out" b "$x" "$MDIR/$mbase" -q >/tmp/pp.log 2>&1; then
    echo "[$v] OK  $(stat -c%s "$out") bytes"
  else
    echo "[$v] FAILED:"; sed 's/^/    /' /tmp/pp.log | head -4; rc=1
  fi
done
exit $rc
SH

echo "== compiling variants (ParameterParser, direct) =="
if uname -s | grep -qiE 'mingw|msys|cygwin'; then
  command -v wsl.exe >/dev/null || { echo "ERROR: on Windows but WSL not available (needed to beat MAX_PATH). Run this on Linux, or 'wsl --install'."; exit 1; }
  # MSYS2_ARG_CONV_EXCL stops Git Bash from rewriting the /mnt/c args into Windows paths.
  MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1 \
    wsl.exe -e bash -lc "$INNER" _ "$(towsl "$CHICDK")" "$(towsl "$GEN")" "$(towsl "$MODULE")" "$(towsl "$OUT")"
else
  bash -c "$INNER" _ "$CHICDK" "$GEN" "$MODULE" "$OUT"
fi

echo "== built bins =="
ls -la "$OUT"/com.qti.sensormodule.cmk_imx678_cam0*.bin 2>/dev/null || true
echo "verify baseline reproduces the shipping .bin:"
echo "  cmp <shipping cam0.bin> $OUT/com.qti.sensormodule.cmk_imx678_cam0.bin   # expect identical"
echo "next: ./deploy.sh cmk_imx678_cam0_hcg"
