function test_coloranalysis
%TEST_COLORANALYSIS  Numeric validation of ColorAnalysis (run headless).
%  1) deltaE2000 vs Sharma-Wu-Dalal (2005) CIEDE2000 reference pairs
%  2) 3x3 CCM exact recovery of a known linear transform
%  3) k-fold cross-validation (LOO + leave-one-group-out) runs and is sane
%  4) meterBracket math
addpath(fileparts(mfilename('fullpath')));
pass = true;

% ---- 1) CIEDE2000 reference pairs: [L1 a1 b1  L2 a2 b2  dE00] ----
D = [ ...
 50.0000  2.6772 -79.7751  50.0000  0.0000 -82.7485   2.0425;
 50.0000  0.0000   0.0000  50.0000 -1.0000   2.0000   2.3669;
 50.0000  2.4900  -0.0010  50.0000 -2.4900   0.0009   7.1792;
 50.0000 -0.0010   2.4900  50.0000  0.0009  -2.4900   4.8045;
 50.0000  2.5000   0.0000  50.0000  0.0000  -2.5000   4.3065;
 50.0000  2.5000   0.0000  73.0000 25.0000 -18.0000  27.1492;
 50.0000  2.5000   0.0000  61.0000 -5.0000  29.0000  22.8977;
 50.0000  2.5000   0.0000  56.0000 -27.000  -3.0000  31.9030;
 50.0000  2.5000   0.0000  58.0000 24.0000  15.0000  19.4535;
 60.2574 -34.0099 36.2677  60.4626 -34.1751 39.4387   1.2644;
 63.0109 -31.0961 -5.8663  62.8187 -29.7946 -4.0864   1.2630;
 35.0831 -44.1164  3.7933  35.0232 -40.0716  1.5901   1.8645;
 22.7233 20.0904 -46.6940  23.0331 14.9730 -42.5619   2.0373;
 36.4612 47.8580 18.3852   36.2715 50.5065 21.2231    1.4146;
 90.8027 -2.0831  1.4410    91.1528 -1.6435 0.0447    1.4441;
 90.9257 -0.5406 -0.9208    88.6381 -0.8985 -0.7239   1.5381;
  6.7747 -0.2908 -2.4247     5.8714 -0.0985 -2.2286   0.6377;
  2.0776  0.0795 -1.1350     0.9033 -0.0636 -0.5514   0.9082 ];
got = ColorAnalysis.deltaE2000(D(:,1:3), D(:,4:6));
err = abs(got - D(:,7));
fprintf('1) CIEDE2000 vs Sharma 2005: max err %.2e over %d pairs\n', max(err), size(D,1));
if max(err) > 1e-3
    pass = false;
    for i = find(err > 1e-3)'
        fprintf('   *** pair %d: got %.4f  expected %.4f\n', i, got(i), D(i,7));
    end
end

% ---- 2) CCM exact recovery of a known linear transform ----
rng(0);
ref  = rand(24,3)*0.8 + 0.05;
Mtrue = eye(3) + 0.15*randn(3);
meas = ref / Mtrue.';               % so meas*Mtrue' == ref
M    = ColorAnalysis.fitCCM(meas, ref);
predErr = max(abs(meas*M.' - ref), [], 'all');
fprintf('2) CCM recovery: prediction err %.2e (expect ~0)\n', predErr);
if predErr > 1e-6, pass = false; fprintf('   *** CCM did not recover linear map\n'); end

% ---- 3) cross-validation (LOO + leave-one-group-out) ----
[deFit, deXval] = ColorAnalysis.crossValDE(meas, ref, 'linear', [], []);
fprintf('3) linear LOO on linear data: fit %.3f  xval %.3f (both ~0)\n', mean(deFit), mean(deXval));
if mean(deXval) > 1e-3, pass = false; end
[rf, rx] = ColorAnalysis.crossValDE(meas, ref, 'rootpoly', 2, []);
fprintf('   rootpoly2 LOO on linear data: fit %.3f  xval %.3f\n', mean(rf), mean(rx));
% two "captures" (second = first + noise): leave-one-capture-out must run
meas2 = [meas; meas + 0.01*randn(24,3)];
ref2  = [ref; ref];
fid   = [ones(24,1); 2*ones(24,1)];
[~, gx] = ColorAnalysis.crossValDE(meas2, ref2, 'linear', [], fid);
fprintf('   leave-one-capture-out xval (2 captures): %.3f\n', mean(gx));
if ~isfinite(mean(gx)), pass = false; end

% ---- 4) meterBracket ----
[ts, tl, legs, ratio] = ColorAnalysis.meterBracket(0.65, 0.0175, 1.8e6);
fprintf('4) meterBracket(hi .65, lo .0175, 1.8ms): short %.2f ms  long %.2f ms  ratio %.1f  legs %s ms\n', ...
    ts/1e6, tl/1e6, ratio, mat2str(round(legs/1e4)/100));
if ~(ratio > 5 && ratio < 30), pass = false; fprintf('   *** ratio out of expected chart range\n'); end

fprintf('%s\n', repmat('-',1,40));
if pass, fprintf('ALL PASS\n'); else, fprintf('*** FAILURES\n'); end
end
