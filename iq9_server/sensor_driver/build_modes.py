#!/usr/bin/env python3
"""Build a SINGLE multi-resolutionData IMX678 sensor XML for reboot-free mode switching.

The IMX678 li-baseline ships ONE <resolutionData> (Full 4K 12-bit 30fps). CamX/qtiqmmfsrc
select a sensor mode by INDEX via the `sensor-mode` property (0-15, -1=auto) OR by matching
the requested caps (width/height/framerate) -- bit-depth is NOT caps-distinguishable (the bayer
pad packs 10/12-bit into bpp=16), so the `sensor-mode` index pin is the robust selector.

This clones the baseline resolutionData once per mode below, applies each mode's verified Sony
AllPixel register delta + geometry, and concatenates them inside <resolutionInfo> so ONE bin
carries every linear mode. Deploy once (reboot); thereafter switch modes at runtime with
`qtiqmmfsrc sensor-mode=<index>` -- NO reboot.

  index 0: 4K 12-bit 30fps  (STOCK baseline, Sony AllPixel col16: 1188 Mbps, HMAX 1100)
  index 1: 4K 12-bit 60fps  (Sony col20: 1782 Mbps, HMAX 550)
  index 2: 4K 10-bit 30fps  (Sony col14:  891 Mbps, HMAX 1100)
  index 3: 4K 10-bit 60fps  (Sony col18: 1440 Mbps, HMAX 550)

4K@72 (Sony col21) is DELIBERATELY excluded: it trips CamX StaticMetadata key 10014 (SEGV) --
the QCS9075 IFE/CSID single-stream ceiling is ~4K@60 (eCON caps 4K@60 too). 72fps is an ROI
feature (reduced height), added later via input-roi-info, not a full-FOV mode.

VMAX (frameLengthLines) stays 2250 for all; only HMAX + datarate + AD/output-bit change.
Datarate reg 0x3015: 05=891, 04=1188, 03=1440, 02=1782, 01=2079 Mbps. HMAX = 0x302D:0x302C.

  python build_modes.py   # -> generated/cmk_imx678_cam0_multimode_sensor.xml
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "baseline", "cmk_imx678_sensor.xml")
OUTDIR = os.path.join(HERE, "generated")
OUT = os.path.join(OUTDIR, "cmk_imx678_cam0_multimode_sensor.xml")

# Baseline resolutionData values (the FROM side of every geom edit).
B = {"dt": "44", "bitWidth": "12", "lineLengthPixelClock": "1100", "frameRate": "30"}

# Each mode = register overrides (addr->hex) + geometry overrides (tag->new). Only the deltas
# vs the baseline block are listed; unlisted fields inherit the baseline (stock) value.
MODES = [
    ("4k12b30", "4K 12-bit 30fps (stock, col16 1188Mbps HMAX1100)",
        {}, {}),
    ("4k12b60", "4K 12-bit 60fps (col20 1782Mbps HMAX550)",
        {"0x3015": "0x02", "0x302C": "0x26", "0x302D": "0x02"},
        {"frameRate": "60", "lineLengthPixelClock": "550"}),
    ("4k10b30", "4K 10-bit 30fps (col14 891Mbps HMAX1100)",
        {"0x3015": "0x05", "0x3022": "0x00", "0x3023": "0x00"},
        {"dt": "43", "bitWidth": "10"}),
    ("4k10b60", "4K 10-bit 60fps (col18 1440Mbps HMAX550)",
        {"0x3015": "0x03", "0x3022": "0x00", "0x3023": "0x00", "0x302C": "0x26", "0x302D": "0x02"},
        {"dt": "43", "bitWidth": "10", "frameRate": "60", "lineLengthPixelClock": "550"}),
]


def apply_reg(block, addr, data):
    pat = re.compile(r'(<registerAddr>' + re.escape(addr) +
                     r'</registerAddr><registerData>)0x[0-9A-Fa-f]+(</registerData>)')
    block, n = pat.subn(r'\g<1>' + data + r'\g<2>', block)
    return block, n


def apply_geom(block, tag, new):
    old = B[tag]
    needle = "<%s>%s</%s>" % (tag, old, tag)
    n = block.count(needle)
    return block.replace(needle, "<%s>%s</%s>" % (tag, new, tag), 1), n


def main():
    xml = open(BASE, encoding="utf-8").read()
    # Extract the single resolutionData block (indentation-anchored, non-greedy).
    m = re.search(r'(?s)([ \t]*<resolutionData>.*?</resolutionData>\n)', xml)
    if not m:
        print("FATAL: no <resolutionData> block found"); return 1
    template = m.group(1)
    ok = True
    blocks = []
    for idx, (name, desc, regs, geom) in enumerate(MODES):
        blk = template
        for addr, data in regs.items():
            blk, n = apply_reg(blk, addr, data)
            if n != 1:
                print("  [%d %s] reg %s matched=%d (want 1)" % (idx, name, addr, n)); ok = False
        for tag, new in geom.items():
            blk, n = apply_geom(blk, tag, new)
            if n != 1:
                print("  [%d %s] geom %s (%s->%s) matched=%d (want 1)" % (idx, name, tag, B[tag], new, n)); ok = False
        # VMAX invariant
        if "<frameLengthLines>2250</frameLengthLines>" not in blk:
            print("  [%d %s] VMAX!=2250" % (idx, name)); ok = False
        # tag the block so the index<->mode mapping is legible in the bin source
        blk = blk.replace("<resolutionData>",
                          "<resolutionData><!-- sensor-mode index %d: %s -->" % (idx, desc), 1)
        # verify the streaming-critical fields
        checks = {"dt": geom.get("dt", B["dt"]), "bitWidth": geom.get("bitWidth", B["bitWidth"]),
                  "frameRate": geom.get("frameRate", B["frameRate"]),
                  "lineLengthPixelClock": geom.get("lineLengthPixelClock", B["lineLengthPixelClock"])}
        for tag, val in checks.items():
            if ("<%s>%s</%s>" % (tag, val, tag)) not in blk:
                print("  [%d %s] verify %s=%s MISSING" % (idx, name, tag, val)); ok = False
        blocks.append(blk)
        print("  [%d] %-8s %s" % (idx, name, desc))
    combined = "".join(blocks)
    xml2 = xml[:m.start(1)] + combined + xml[m.end(1):]
    # sanity: N resolutionData now present
    n_res = xml2.count("<resolutionData>")
    os.makedirs(OUTDIR, exist_ok=True)
    open(OUT, "w", encoding="utf-8", newline="\n").write(xml2)
    print("resolutionData blocks in output: %d (want %d)" % (n_res, len(MODES)))
    print("wrote", OUT, "->", "OK" if (ok and n_res == len(MODES)) else "PROBLEM")
    return 0 if (ok and n_res == len(MODES)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
