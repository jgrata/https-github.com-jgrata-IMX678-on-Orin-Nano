import sys, json
sys.path.insert(0, "/root/iq9_server/_shared")
import numpy as np, cv2, colorchecker as C

# Factory Chromatix CCMs (Scenario.Default/XML/IPE/cc13_ipe_v2.xml), row-major 3x3.
# keys: (lux_bin, cct_bin) ; values applied to WB'd linear RGB (rows sum ~1).
FACTORY = {
 ("lux245-280","A2600-2900"): [1.92476416,-0.7112738,-0.213490382, -1.25936186,2.63456368,-0.3752017, -0.4099119,-2.020071,3.429983],
 ("lux245-280","W3400-3700"): [1.71669841,-0.5972234,-0.119475044, -1.13398027,2.81143069,-0.6774504, -0.1682801,-1.53125215,2.69953227],
 ("lux245-280","D5900-7000"): [2.01467,-0.9591302,-0.05553964, -0.6275375,2.1084516,-0.4809142, 0.06874546,-1.42175055,2.353005],
 ("lux340-360","A2600-2900"): [1.835879,-0.3256696,-0.5102094, -1.40663218,3.12141538,-0.7147832, -0.6011388,-1.4364084,3.037547],
 ("lux340-360","W3400-3700"): [1.69685006,-0.5569026,-0.139947519, -1.21617079,3.00234127,-0.7861705, -0.169343486,-1.51005936,2.67940283],
 ("lux340-360","D5900-7000"): [1.58897352,-0.353940785,-0.235032737, -0.997933447,2.7385447,-0.7406114, -0.230760723,-0.8855468,2.1163075],
 ("lux435-460","A2600-2900"): [1.994788,-0.638571739,-0.356216341, -1.36121607,3.052882,-0.6916658, -0.493949562,-1.64987421,3.14382386],
 ("lux435-460","W3400-3700"): [1.72427177,-0.65672,-0.0675517842, -1.17465067,2.90199065,-0.7273401, -0.133502781,-1.53605223,2.669555],
 ("lux435-460","D5900-7000"): [1.68744528,-0.5044142,-0.183031127, -0.958552957,2.73732114,-0.7787682, -0.197848767,-0.875852942,2.07370162],
}

raw = np.load("/tmp/cc.npy").astype(np.uint16)
maxv, bl = 4095, 105
rgb_lin = C._bin_rggb(raw, maxv, bl)
bgr = C._detect_img(rgb_lin)
det = cv2.mcc.CCheckerDetector_create()
assert det.process(bgr, cv2.mcc.MCC24), "no detect"
cc = det.getListColorChecker()[0]
corners = C._sort_corners(cc.getBox())
chart, ctrs, _ = C._sample_linear(rgb_lin, corners)
ref = C.hdr.colorchecker_linear(); ref_lab = C._lin2lab(ref)
# orient
best = None
for name, perm in C._PERMS.items():
    X = chart[perm]; sc = ref.mean()/max(X.mean(),1e-9)
    A,*_ = np.linalg.lstsq(X*sc, ref, rcond=None)
    r = float(np.sqrt(np.mean((X*sc@A-ref)**2)))
    if best is None or r < best[1]: best = (name, r, perm)
chart = chart[best[2]]
s3 = ref.mean(0)/np.maximum(chart.mean(0),1e-9)   # per-channel gray-world WB (same as vendor path)

def dE_for(ccm):
    out = (chart*s3) @ np.asarray(ccm).reshape(3,3).T
    return C._de2000(C._lin2lab(out), ref_lab)

rows = []
# custom fitted 3x3 (resubstitution) + honest xval
M = C._fit_ccm(chart, ref)
dE_cust = C._de2000(C._lin2lab(chart @ np.asarray(M).T), ref_lab)
rows.append(("custom_fit_3x3", dE_cust.mean(), dE_cust.max()))
dE_xval = C._loo_de2000(chart, ref, ref_lab)
rows.append(("custom_xval_3x3", dE_xval.mean(), dE_xval.max()))
# generic vendor CCM (existing baseline)
dv = dE_for(np.asarray(C.hdr.VENDOR_CCM))
rows.append(("generic_VENDOR_CCM", dv.mean(), dv.max()))
# factory Chromatix CCMs
for k, m in FACTORY.items():
    d = dE_for(m)
    rows.append(("factory_%s_%s" % k, d.mean(), d.max()))

print("%-34s %8s %8s" % ("model","dE_mean","dE_max"))
for name, mn, mx in rows:
    print("%-34s %8.3f %8.3f" % (name, mn, mx))
best_fac = min([r for r in rows if r[0].startswith("factory")], key=lambda r: r[1])
print("\nbest factory: %s  dE_mean %.3f" % (best_fac[0], best_fac[1]))
print("custom xval dE_mean %.3f  |  scene illum ~%.0fK (mixed, NOT rigorous)" %
      (dE_xval.mean(), C._illuminant(np.mean(chart[18:21],axis=0))[0]))
