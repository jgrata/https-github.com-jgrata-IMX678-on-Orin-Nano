#!/usr/bin/env python3
"""Build the Clear HDR (DCG) sensor XML from the LI baseline + the SRM ClearHDR delta set.

Target: ClearHDR_AllPixel **4-lane / 10-bit / 15 fps** (dual-VC HG+LG) — the config that fits our
4-lane sensor (Standard_Register_Setting_Ver3.0.xlsx cfg 5). Applies the 53-register delta
(clearhdr_allpixel_deltas.csv) as in-place overrides on the baseline + patches the resolutionData
geometry (VMAX/frameRate/bit-depth/height).

FIRST-CUT — validation deferred. Open items before deploy:
  - RDI capture reboots (qsmmuv500 SMMU panic — escalated)
  - dual-VC (HG/LG) over RDI unverified on this CamX/qtiqmmfsrc stack (as DOL); full-height capture
    must not clamp
  - 4-lane Clear HDR is 10-bit; CamX rejected RAW10 at 3856 width (>3840) -> capture likely needs a
    RAW16 container
  - frameDimension height set to 2x2180=4360 (HG+LG stacked); exact rows incl OB/embedded TBC
"""
import os, re, csv

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "baseline", "cmk_imx678_sensor.xml")
DELTAS = os.path.join(HERE, "clearhdr_allpixel_deltas.csv")
OUT = os.path.join(HERE, "generated", "cmk_imx678_cam0_chdr_sensor.xml")


def apply_reg(xml, addr, data):
    pat = re.compile(r'(<registerAddr>' + re.escape(addr) +
                     r'</registerAddr><registerData>)0x[0-9A-Fa-f]+(</registerData>)')
    return pat.subn(r'\g<1>' + data + r'\g<2>', xml)


def patch_once(xml, pat, repl, label):
    xml2, n = re.subn(pat, repl, xml, count=1)
    print("  geom %-26s matched=%d" % (label, n))
    return xml2


def main():
    xml = open(BASE, encoding="utf-8").read()
    applied, problems = 0, []
    with open(DELTAS) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#") or row[0] == "addr":
                continue
            addr, _ap, ch = row[0], row[1], row[2]
            data = "0x%02X" % int(ch.lower().replace("0x", ""), 16)
            xml, n = apply_reg(xml, addr, data)
            if n == 1:
                applied += 1
            else:
                problems.append((addr, n))
    print("register deltas applied: %d/53  problems: %s" % (applied, problems or "none"))
    # geometry (mode metadata) for 4-lane/10-bit/15fps Clear HDR
    xml = patch_once(xml, r'<frameLengthLines>2250</frameLengthLines>',
                     '<frameLengthLines>4500</frameLengthLines>', 'frameLengthLines 2250->4500 (VMAX)')
    xml = patch_once(xml, r'<frameRate>30</frameRate>', '<frameRate>15</frameRate>', 'frameRate 30->15')
    xml = patch_once(xml, r'<dt>44</dt>', '<dt>43</dt>', 'dt 44(RAW12)->43(RAW10)')
    xml = patch_once(xml, r'<bitWidth>12</bitWidth>', '<bitWidth>10</bitWidth>', 'bitWidth 12->10')
    xml = patch_once(xml, r'(<width>3856</width>\s*<height>)2180(</height>)', r'\g<1>4360\g<2>',
                     'frameDim height 2180->4360 (HG+LG)')
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, "w", encoding="utf-8", newline="\n").write(xml)
    # verify a couple of signature registers landed
    for a, want in [("0x301A", "0x08"), ("0x3028", "0x94"), ("0x3029", "0x11"), ("0x3081", "0x02")]:
        ok = ("<registerAddr>%s</registerAddr><registerData>%s</registerData>" % (a, want)) in xml
        print("  verify %s=%s : %s" % (a, want, "OK" if ok else "MISSING"))
    print("wrote", OUT)
    return 0 if applied == 53 and not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
