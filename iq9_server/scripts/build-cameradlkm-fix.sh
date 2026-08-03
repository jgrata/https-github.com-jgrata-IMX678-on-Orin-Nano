#!/bin/bash
# build-cameradlkm-fix.sh
#
# Build the QCS9075 camera kernel driver (cameradlkm -> camera_qcs9100.ko) for QLI 1.7
# (kernel 6.6.116) with the camera-kernel r1-rel fixes that resolve the RDI-release
# IFE-SMMU page fault -> qcom_wdt watchdog reboot during RAW capture. The key fix is
# f7b70309 "msm: camera: isp: Fix KMD buffer handle in IFE prepare" (invalid KMD buffer
# handle in cleanup -> stream freeze after several iterations); we pin r1-rel HEAD which
# also carries the sync/reqmgr UAF fixes.
#
# FIRE-AND-FORGET for hvo: set the two vars below, then run and walk away:
#     nohup ./build-cameradlkm-fix.sh >build-dlkm.out 2>&1 &
# When it finishes, the module is at $OUT_KO. Hand THAT file to the Metropolis team
# (e.g.  scp camera_qcs9100.ko metro@10.70.0.60:/tmp/ ). We handle deploy + test.
#
# NOTE: build in the SAME 1.7 tree that produced the device image, so the module's
# vermagic matches the running kernel (6.6.116-qli-1.7-ver.1.1-...-dirty). The script
# prints the vermagic at the end — it MUST match the device or the module won't load.

# ---- set these for YOUR 1.7 tree / build env -------------------------------------
YOCTO_TREE="${YOCTO_TREE:-/home/metropolis-yocto/workspace/iq9075-evk-yocto}"
# The command that builds ONE recipe in your env. `-k` (keep-going) so any unrelated
# host do_package hiccup doesn't stop the module (we only need the .ko from do_compile).
#   oe/repo:  source setup-environment <builddir> && bitbake -k cameradlkm
#   kas:      kas shell <kas.yml> -c "bitbake -k cameradlkm"
BUILD_CMD="${BUILD_CMD:-source setup-environment build && bitbake -k cameradlkm}"
# ----------------------------------------------------------------------------------

SRCREV_FIX="f4491100af82926747ec7cef502e26fd95ce46ce"   # r1-rel HEAD (has f7b70309); min w/ fix: 705682c5e52fce0c8cb61d82e336af1725eab462
OUT_KO="$YOCTO_TREE/camera_qcs9100.ko"

cd "$YOCTO_TREE" || { echo "ERROR: tree not found: $YOCTO_TREE"; exit 1; }
RECIPE=$(ls layers/*/recipes-multimedia/cameradlkm/cameradlkm_1.0.qcom.bb 2>/dev/null | head -1)
[ -n "$RECIPE" ] || { echo "ERROR: cameradlkm recipe not found under layers/*/recipes-multimedia/cameradlkm/"; exit 1; }

echo "== $(date) : cameradlkm build, SRCREV -> $SRCREV_FIX =="
cp -n "$RECIPE" "$RECIPE.orig"                                   # keep original to restore
sed -i -E "s|^([[:space:]]*SRCREV[[:space:]]*=).*|\1 \"$SRCREV_FIX\"|" "$RECIPE"
echo "recipe SRCREV now: $(grep -E '^[[:space:]]*SRCREV' "$RECIPE")"

eval "$BUILD_CMD"
echo "== bitbake exit: $? =="

KO=$(find tmp*/work -path "*cameradlkm*" -name "camera_qcs9100.ko" 2>/dev/null | head -1)
if [ -n "$KO" ]; then
    cp "$KO" "$OUT_KO"
    echo "SUCCESS: module at  $OUT_KO"
    modinfo "$OUT_KO" 2>/dev/null | grep -E "vermagic|srcversion|^name"
    echo ">>> vermagic MUST be 6.6.116-qli-1.7-ver.1.1-... to load on the device <<<"
    echo ">>> hand off:  scp \"$OUT_KO\" metro@10.70.0.60:/tmp/   (or wherever we can fetch it)"
else
    echo "FAIL: no camera_qcs9100.ko produced. Check the build log; the .ko is created in"
    echo "      cameradlkm do_compile (before do_package), so 'bitbake -k' should yield it."
fi

mv -f "$RECIPE.orig" "$RECIPE" 2>/dev/null   # restore recipe (leave hvo's tree unmodified)
echo "== done $(date) =="
