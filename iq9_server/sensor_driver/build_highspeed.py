#!/usr/bin/env python3
"""Build high-frame-rate linear IMX678 sensor XMLs from the LI baseline + Sony AllPixel tables.

Adds the two headline high-speed all-pixel modes the DAQ/preview want, matching the eCON eCam
menu. Both are 4-lane, full-resolution (3856x2180), LINEAR (single-VC) — i.e. the SAME usecase
type as the shipping 30fps mode, just faster line timing, so they carry NO is_shdr/dual-RDI
capture blocker (unlike Clear HDR). Source: IMX678_Standard_Register_Setting_Ver3.0.xlsx, sheet
`AllPixel` (Sony), cross-checked against the LI baseline register map.

  10b@72  = AllPixel col20: 4-lane, AD10/Out10, 2079 Mbps, 72.052 fps
  12b@60  = AllPixel col19: 4-lane, AD12/Out12, 1782 Mbps, 60 fps

Timing insight (why VMAX doesn't change): fps = 1/(VMAX x t_line), t_line = HMAX/INCK (INCK
74.25 MHz). Baseline 30fps = VMAX 2250 x HMAX 1100 (t_line 14.815us -> 33.33ms). Both high-speed
modes keep VMAX=2250 (same 2176 active + blanking) and SHORTEN HMAX + raise the data rate:
  10b@72: HMAX 458 (0x01CA) -> t_line 6.169us -> 2250*6.169us = 13.88ms = 72.05fps
  12b@60: HMAX 550 (0x0226) -> t_line 7.407us -> 2250*7.407us = 16.67ms = 60.00fps

So each mode = a tiny, verified register delta (data-rate + HMAX, plus AD-bit for 10-bit) + the
resolutionData geometry (frameRate/dt/bitWidth/lineLengthPixelClock). Exposure (SHR0) is LEFT at
the baseline (900 lines -> 5.55ms @72fps / 6.67ms @60fps, both valid < VMAX) — exposure is a
build-swap / Layer-B axis, not part of the mode definition.

  python build_highspeed.py            # writes generated/cmk_imx678_cam0_{hs10b72,hs12b60}_sensor.xml
  ./compile.sh ; ./deploy.sh cmk_imx678_cam0_hs10b72
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "baseline", "cmk_imx678_sensor.xml")
OUTDIR = os.path.join(HERE, "generated")

# Each mode: register overrides (addr -> col value) + resolutionData geometry. Deltas verified
# against the baseline XML (build_highspeed cross-check: baseline value -> target value).
MODES = {
    "hs10b72": {
        "desc": "4-lane 10-bit 72.052fps all-pixel (Sony AllPixel col20)",
        "regs": {"0x3015": "0x01",   # DATARATE_SEL -> 2079 Mbps
                 "0x3022": "0x00",   # AD/output bit-depth -> 10-bit
                 "0x3023": "0x00",   # AD/output bit-depth -> 10-bit
                 "0x302C": "0xCA",   # HMAX low  -> 0x01CA = 458
                 "0x302D": "0x01"},  # HMAX high
        "geom": {"frameRate": ("30", "72"), "dt": ("44", "43"),        # RAW12(0x2C) -> RAW10(0x2B)
                 "bitWidth": ("12", "10"), "lineLengthPixelClock": ("1100", "458")},
    },
    "hs12b60": {
        "desc": "4-lane 12-bit 60fps all-pixel (Sony AllPixel col19)",
        "regs": {"0x3015": "0x02",   # DATARATE_SEL -> 1782 Mbps
                 "0x302C": "0x26",   # HMAX low  -> 0x0226 = 550
                 "0x302D": "0x02"},  # HMAX high
        "geom": {"frameRate": ("30", "60"), "lineLengthPixelClock": ("1100", "550")},  # dt/bitWidth stay 12-bit
    },
}


def apply_reg(xml, addr, data):
    pat = re.compile(r'(<registerAddr>' + re.escape(addr) +
                     r'</registerAddr><registerData>)0x[0-9A-Fa-f]+(</registerData>)')
    return pat.subn(r'\g<1>' + data + r'\g<2>', xml)


def apply_geom(xml, tag, old, new):
    return xml.replace("<%s>%s</%s>" % (tag, old, tag), "<%s>%s</%s>" % (tag, new, tag), 1), \
        xml.count("<%s>%s</%s>" % (tag, old, tag))


def build(name, spec):
    xml = open(BASE, encoding="utf-8").read()
    ok = True
    for addr, data in spec["regs"].items():
        xml, n = apply_reg(xml, addr, data)
        print("  reg  %s -> %s  matched=%d" % (addr, data, n))
        ok = ok and n == 1
    for tag, (old, new) in spec["geom"].items():
        n = xml.count("<%s>%s</%s>" % (tag, old, tag))
        xml, _ = apply_geom(xml, tag, old, new)
        print("  geom %-22s %s -> %s  matched=%d" % (tag, old, new, n))
        ok = ok and n == 1
    # frameLengthLines (VMAX) MUST stay 2250 for both modes
    ok = ok and "<frameLengthLines>2250</frameLengthLines>" in xml
    out = os.path.join(OUTDIR, "cmk_imx678_cam0_%s_sensor.xml" % name)
    os.makedirs(OUTDIR, exist_ok=True)
    open(out, "w", encoding="utf-8", newline="\n").write(xml)
    # verify the streaming-critical fields landed
    for tag, (_o, new) in spec["geom"].items():
        present = ("<%s>%s</%s>" % (tag, new, tag)) in xml
        print("  verify %-22s = %s : %s" % (tag, new, "OK" if present else "MISSING"))
        ok = ok and present
    print("  wrote", out, "->", "OK" if ok else "PROBLEM")
    return ok


def main():
    allok = True
    for name, spec in MODES.items():
        print("== %s : %s ==" % (name, spec["desc"]))
        allok = build(name, spec) and allok
    print("\nnext: ./compile.sh  (compiles generated/*_sensor.xml -> bins/*.bin via ParameterParser)")
    return 0 if allok else 1


if __name__ == "__main__":
    raise SystemExit(main())
