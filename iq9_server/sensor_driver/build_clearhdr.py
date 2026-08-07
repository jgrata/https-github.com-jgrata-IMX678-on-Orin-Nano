#!/usr/bin/env python3
"""Build the Clear HDR (DCG) sensor XML from the LI baseline + the SRM ClearHDR delta set.

Target: ClearHDR_AllPixel 4-lane / 10-bit / 15 fps (dual-VC HG+LG), the config that fits our
4-lane sensor (Standard_Register_Setting_Ver3.0.xlsx cfg 5). Applies the 53-register delta
(clearhdr_allpixel_deltas.csv) + rewrites the resolutionData into a proper dual-VC HDR stream.

Stream descriptor (reverse-engineered 2026-08-07 from imx766 DOL + api/sensor/camxsensordriver.xsd,
after the first-cut single-VC / stacked-4360 .bin streamed ZERO frames on the bench):
  - ONE <streamConfiguration> with TWO <vc> children (0=HG, 1=LG); each leg at the NATIVE 2180
    height (NOT stacked 4360). dt=43 (0x2B RAW10), bitWidth 10, type IMAGE. (XSD: vc maxOccurs=2.)
  - resolutionData: VMAX 4500 / 15 fps, exposureInfo x2 (DEFAULT+SHORT), HDRExposureType TWOEXPOSURE,
    capability SHDR. --exp-gain 0 -> EXP_GAIN=0 so the legs differ by conversion gain only (HG=HCG,
    LG=LCG) for the static LCG/HCG comparator.

Open: capture-side geometry (does CamX deliver the two VCs stacked in one RDI buffer, or two?) is
bench-discovered via dcg_demux analyze; the shipping-mode RDI SMMU reboot (escalated) is separate.
"""
import os, re, csv, argparse

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


def sub_once(xml, old, new, label):
    """Literal single-occurrence insert (for structural XML, no regex escaping). Returns (xml, n)."""
    n = xml.count(old)
    print("  struct %-40s matched=%d" % (label, n))
    return (xml.replace(old, new, 1) if n == 1 else xml), n


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Build the Clear HDR (DCG) sensor XML from the LI baseline + SRM deltas")
    ap.add_argument("--exp-gain", type=int, choices=range(0, 6), default=None, metavar="0..5",
                    help="override EXP_GAIN (0x3081): 0..5 = 0/6/12/18/24/30 dB added to the HG "
                         "leg. Use 0 for a clean LCG/HCG comparator (legs differ by conversion "
                         "gain only). Default: keep the CSV value 0x02 (+12 dB).")
    ap.add_argument("--out", default=None, help="output XML path (default auto-named under generated/)")
    args = ap.parse_args(argv)

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
    xml = patch_once(xml, r'<dt>44</dt>', '<dt>43</dt>', 'dt 44(RAW12)->43(RAW10=0x2B)')
    xml = patch_once(xml, r'<bitWidth>12</bitWidth>', '<bitWidth>10</bitWidth>', 'bitWidth 12->10')
    # Dual-VC stream descriptor (reverse-engineered from imx766 DOL + camxsensordriver.xsd):
    # Clear HDR emits HG(VC0)+LG(VC1) in ONE streamConfiguration, each leg at the NATIVE 2180
    # height (NOT stacked 4360 — the earlier first-cut was wrong; that never streamed). Add VC1.
    xml, nvc = sub_once(xml, '<vc range="[0,3]">0</vc>',
                        '<vc range="[0,3]">0</vc>\n          <vc range="[0,3]">1</vc>',
                        'streamConfig: add VC1 (dual-VC HG+LG, per-leg 2180)')
    if nvc != 1:
        problems.append(("vc1", nvc))
    # HDR metadata at resolutionData level (XSD order: cropInfo -> exposureInfo -> HDRExposureType).
    hdr = ("\n      <exposureInfo><exposureType>DEFAULT</exposureType>"
           "<maxLineCount>4494</maxLineCount><minLineCount>8</minLineCount></exposureInfo>"
           "\n      <exposureInfo><exposureType>SHORT</exposureType>"
           "<maxLineCount>4494</maxLineCount><minLineCount>8</minLineCount></exposureInfo>"
           "\n      <HDRExposureType>TWOEXPOSURE</HDRExposureType>")
    xml, nh = sub_once(xml, '</cropInfo>', '</cropInfo>' + hdr,
                       'add exposureInfo x2 + HDRExposureType TWOEXPOSURE')
    if nh != 1:
        problems.append(("hdrmeta", nh))
    # capability NORMAL -> SHDR (triggers CamX's dual-VC HDR stream setup, as in imx766 DOL)
    xml, nc = sub_once(xml, '<capability>NORMAL</capability>', '<capability>SHDR</capability>',
                       'capability NORMAL->SHDR')
    if nc != 1:
        problems.append(("capability", nc))

    # optional EXP_GAIN override. Gain model: LG = GAIN, HG = GAIN + EXP_GAIN.
    variant = "chdr"
    if args.exp_gain is not None:
        xml, n = apply_reg(xml, "0x3081", "0x%02X" % args.exp_gain)
        print("  EXP_GAIN 0x3081 -> 0x%02X (HG +%d dB)  matched=%d"
              % (args.exp_gain, args.exp_gain * 6, n))
        if n != 1:
            problems.append(("0x3081", n))
        variant = "chdr_dcgcal" if args.exp_gain == 0 else "chdr_expg%d" % args.exp_gain

    out = args.out or os.path.join(HERE, "generated", "cmk_imx678_cam0_%s_sensor.xml" % variant)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w", encoding="utf-8", newline="\n").write(xml)
    # verify signature registers landed (0x3081 reflects any override)
    exp_gain_want = "0x%02X" % (args.exp_gain if args.exp_gain is not None else 0x02)
    for a, want in [("0x301A", "0x08"), ("0x3028", "0x94"), ("0x3029", "0x11"), ("0x3081", exp_gain_want)]:
        ok = ("<registerAddr>%s</registerAddr><registerData>%s</registerData>" % (a, want)) in xml
        print("  verify %s=%s : %s" % (a, want, "OK" if ok else "MISSING"))
    # verify the transport-critical dual-VC + HDR stream structure
    for token, desc in (('<vc range="[0,3]">1</vc>', 'VC1 present'),
                        ('<HDRExposureType>TWOEXPOSURE</HDRExposureType>', 'TWOEXPOSURE'),
                        ('<capability>SHDR</capability>', 'capability SHDR')):
        print("  verify %-16s : %s" % (desc, "OK" if token in xml else "MISSING"))
    print("  verify per-leg-height  : %s"
          % ("OK (2180, not stacked)" if "<height>4360</height>" not in xml else "BAD (4360 stacked)"))
    print("wrote", out)
    if args.exp_gain == 0:
        print("gain model: EXP_GAIN=0 -> HG=HCG, LG=LCG at equal analog gain; "
              "Rcg = net mean_HG / mean_LG (expect ~2.4x; datasheet Rcg 2.4-2.9)")
    return 0 if applied == 53 and not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
