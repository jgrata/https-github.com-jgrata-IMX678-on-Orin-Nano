% gen_ref.m -- generate slanted-edge ROIs, run the reference MATLAB jslantedge,
% and save jslant_ref.mat (-v7) for the Python regression (test_mtf_regression.py).
% Run:  matlab -batch "cd('<this dir>'); gen_ref"
addpath('C:\Users\JGrata\Documents\MATLAB\jcode');   % reference jslantedge + deps
outfile = fullfile(fileparts(mfilename('fullpath')), 'jslant_ref.mat');

% specs: {rows, cols, angle_deg, sigma_px, dark_left, osf, pixel}
specs = {
    { 80,100, 4.0, 1.0, true,  4, 1.0 };
    {100,120, 7.0, 1.5, false, 4, 1.0 };
    {120, 90, 3.0, 0.8, true,  3, 1.0 };
    { 96,110, 5.0, 2.0, true,  4, 0.0028 };
    {140,160, 6.0, 1.2, false, 5, 1.0 };
};
nc = numel(specs);
Is=cell(1,nc); osfs=zeros(1,nc); pixels=zeros(1,nc);
freqs=cell(1,nc); mtfs=cell(1,nc); esfs=cell(1,nc); lsfs=cell(1,nc); outs=cell(1,nc);
for k=1:nc
    s=specs{k}; R=s{1}; C=s{2}; ang=s{3}; sig=s{4}; dl=s{5}; osf=s{6}; pixel=s{7};
    I = make_edge(R,C,ang,sig,dl);
    [freq,mtf,esf,lsf,BW,out]=jslantedge(I,osf,pixel);
    Is{k}=I; osfs(k)=osf; pixels(k)=pixel;
    freqs{k}=freq; mtfs{k}=mtf; esfs{k}=esf; lsfs{k}=lsf; outs{k}=out;
    fprintf('case%d: %dx%d ang=%.1f sig=%.1f osf=%d pixel=%.4g -> mtf len=%d out=[%.4f %.4f]\n',...
        k,R,C,ang,sig,osf,pixel,numel(mtf),out(1),out(2));
end
save(outfile,'Is','osfs','pixels','freqs','mtfs','esfs','lsfs','outs','-v7');
fprintf('saved %s\n', outfile);

function I = make_edge(R,C,ang,sig,darkleft)
    [xx,yy]=meshgrid(1:C,1:R);
    dist = xx - (C/2 + (yy-1)*tand(ang));    % signed distance across the edge
    v = 0.5*(1+erf(dist./(sqrt(2)*sig)));    % smooth step, Gaussian LSF
    if ~darkleft, v = 1-v; end
    I = 50 + 900*v;                          % ~12-bit-ish DN, dark~50 bright~950
end
