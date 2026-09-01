"""Run ColorChecker analysis under the .ccvenv (working cv2.mcc 4.11) for the webui, which can't
import a working mcc itself. Reads a frame from a .npy and prints the FULL result JSON (overlay/
swatch pngs intact) on a marker line so the caller can pick it out of any gst/mesa chatter.

  cc_web.py raw  <frame.npy> [maxv]     -> colorchecker.analyze(frame, maxv)         (linear RAW)
  cc_web.py processed <frame.npy>       -> colorchecker.analyze_processed(bgr)       (NV12/ISP BGR)
"""
import sys
import json
sys.path.insert(0, "/root/iq9_server/_shared")
import numpy as np
import colorchecker

mode = sys.argv[1]
frame = np.load(sys.argv[2])
if mode == "raw":
    maxv = int(sys.argv[3]) if len(sys.argv) > 3 else 4095
    res = colorchecker.analyze(frame, maxv)
else:
    res = colorchecker.analyze_processed(frame)
sys.stdout.write("@@CCJSON@@" + json.dumps(res, default=float) + "\n")
sys.stdout.flush()
