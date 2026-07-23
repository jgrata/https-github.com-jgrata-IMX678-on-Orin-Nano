classdef ColorAnalysis
    %COLORANALYSIS  Rigorous colour-accuracy math for camera/ISP bring-up.
    %   Static, UI-free, and reusable by ECamCameraGUI and lab-automation scripts.
    %   Ports (and is numerically cross-checked against) the validated web tool:
    %   CIEDE2000, scale-conditioned 3x3 CCM, Finlayson root-polynomial CCM, and
    %   k-fold cross-validation (fold membership is caller-supplied, so it supports
    %   leave-one-out across patches AND leave-one-capture-out across N captures /
    %   poses / illuminant intensities). Also the chart-metered HDR bracket math.
    %
    %   Validate with:  test_coloranalysis   (checks deltaE2000 vs the Sharma 2005
    %   CIEDE2000 reference pairs and the CCM/root-poly/k-fold behaviour).

    methods (Static)

        function dE = deltaE2000(lab1, lab2)
            %DELTAE2000  CIEDE2000 colour difference (kL=kC=kH=1). Nx3 Lab -> Nx1.
            L1=lab1(:,1); a1=lab1(:,2); b1=lab1(:,3);
            L2=lab2(:,1); a2=lab2(:,2); b2=lab2(:,3);
            C1=hypot(a1,b1); C2=hypot(a2,b2); Cbar=(C1+C2)/2;
            G=0.5*(1-sqrt(Cbar.^7./(Cbar.^7+25^7)));
            a1p=(1+G).*a1; a2p=(1+G).*a2;
            C1p=hypot(a1p,b1); C2p=hypot(a2p,b2);
            h1p=mod(atan2d(b1,a1p),360); h2p=mod(atan2d(b2,a2p),360);
            dLp=L2-L1; dCp=C2p-C1p;
            dhp=h2p-h1p;
            dhp(dhp> 180)=dhp(dhp> 180)-360;
            dhp(dhp<-180)=dhp(dhp<-180)+360;
            dhp(C1p.*C2p==0)=0;
            dHp=2*sqrt(C1p.*C2p).*sind(dhp/2);
            Lbarp=(L1+L2)/2; Cbarp=(C1p+C2p)/2;
            hsum=h1p+h2p; hdiff=abs(h1p-h2p);
            hbarp=hsum/2;                                   % hdiff<=180
            m=hdiff>180 & hsum<360;  hbarp(m)=(hsum(m)+360)/2;
            m=hdiff>180 & hsum>=360; hbarp(m)=(hsum(m)-360)/2;
            hbarp(C1p.*C2p==0)=hsum(C1p.*C2p==0);
            T=1-0.17*cosd(hbarp-30)+0.24*cosd(2*hbarp) ...
              +0.32*cosd(3*hbarp+6)-0.20*cosd(4*hbarp-63);
            dTheta=30*exp(-((hbarp-275)/25).^2);
            RC=2*sqrt(Cbarp.^7./(Cbarp.^7+25^7));
            SL=1+(0.015*(Lbarp-50).^2)./sqrt(20+(Lbarp-50).^2);
            SC=1+0.045*Cbarp; SH=1+0.015*Cbarp.*T;
            RT=-sind(2*dTheta).*RC;
            dE=sqrt((dLp./SL).^2+(dCp./SC).^2+(dHp./SH).^2 ...
                    +RT.*(dCp./SC).*(dHp./SH));
        end

        function lab = linToLab(lin)
            %LINTOLAB  linear-sRGB [N,3] in [0,1] -> CIELAB (D65), via IPT.
            lin = min(max(lin,0),1);
            lab = reshape(rgb2lab(lin2rgb(reshape(lin,[],1,3))), [], 3);
        end

        function M = fitCCM(meas, ref)
            %FITCCM  scale-conditioned 3x3, apply as rgb*M' (== GUI lsqCCM).
            sc = mean(ref(:))/max(mean(meas(:)),1e-9);
            A  = (meas*sc)\ref;                             % 3x3
            M  = sc*A';
        end

        function Phi = rootPolyFeatures(rgb, degree)
            %ROOTPOLYFEATURES  Finlayson root-poly terms (homogeneous deg-1 ->
            %   exposure-invariant). deg1=3 (linear), deg2=6, deg3=13.
            R=max(rgb(:,1),0); G=max(rgb(:,2),0); B=max(rgb(:,3),0);
            Phi=[R G B];
            if degree>=2
                Phi=[Phi sqrt(R.*G) sqrt(G.*B) sqrt(R.*B)];
            end
            if degree>=3
                Phi=[Phi nthroot(R.*G.*G,3) nthroot(R.*R.*G,3) nthroot(G.*B.*B,3) ...
                         nthroot(G.*G.*B,3) nthroot(R.*B.*B,3) nthroot(R.*R.*B,3) ...
                         nthroot(R.*G.*B,3)];
            end
        end

        function model = fitRootPoly(meas, ref, degree)
            sc  = mean(ref(:))/max(mean(meas(:)),1e-9);
            Phi = ColorAnalysis.rootPolyFeatures(meas*sc, degree);
            A   = Phi\ref;                                  % Nterms x 3
            model = struct('sc',sc, 'A',A, 'degree',degree);
        end

        function pred = applyRootPoly(rgb, model)
            pred = ColorAnalysis.rootPolyFeatures(rgb*model.sc, model.degree)*model.A;
        end

        function [deFit, deXval] = crossValDE(meas, ref, modelType, degree, foldId)
            %CROSSVALDE  ΔE00 resubstitution (deFit) + cross-validated (deXval).
            %   foldId (N-vector) sets fold membership; each unique fold is held out
            %   in turn. Default foldId = (1:N)' = leave-one-out across patches.
            %   For N captures stacked row-wise, pass foldId = repelem(1:N,24)' for
            %   leave-one-capture-out. modelType: 'linear' | 'rootpoly'.
            n = size(meas,1);
            if nargin<5 || isempty(foldId), foldId=(1:n)'; end
            if nargin<4 || isempty(degree), degree=2; end
            isRP = strcmpi(modelType,'rootpoly');
            fitf = @(X,Y) ColorAnalysis.fitModel(X,Y,isRP,degree);
            appf = @(m,X) ColorAnalysis.applyModel(m,X,isRP);
            refLab = ColorAnalysis.linToLab(ref);
            mdl = fitf(meas, ref);
            deFit = ColorAnalysis.deltaE2000(ColorAnalysis.linToLab(appf(mdl,meas)), refLab);
            deXval = zeros(n,1);
            u = unique(foldId(:));
            for k=1:numel(u)
                te = foldId(:)==u(k); tr = ~te;
                m  = fitf(meas(tr,:), ref(tr,:));
                pr = appf(m, meas(te,:));
                deXval(te) = ColorAnalysis.deltaE2000(ColorAnalysis.linToLab(pr), refLab(te,:));
            end
        end

        function m = fitModel(X, Y, isRP, degree)
            if isRP, m = ColorAnalysis.fitRootPoly(X,Y,degree);
            else,    m = ColorAnalysis.fitCCM(X,Y); end
        end

        function pr = applyModel(m, X, isRP)
            if isRP, pr = ColorAnalysis.applyRootPoly(X,m);
            else,    pr = X*m'; end
        end

        function [tShort, tLong, legs, ratio] = meterBracket(hi, lo, curExpNs, targetHi, targetLo, nLegsMax)
            %METERBRACKET  HDR bracket from measured chart levels. hi = brightest
            %   patch channel, lo = darkest patch mean (both fractions of full
            %   signal at curExpNs). Short leg puts hi at targetHi, long leg puts lo
            %   at targetLo. Returns leg exposures (ns).
            if nargin<4||isempty(targetHi), targetHi=0.90; end
            if nargin<5||isempty(targetLo), targetLo=0.35; end
            if nargin<6||isempty(nLegsMax), nLegsMax=5; end
            MINn=50e3; MAXn=500e6;
            tShort=min(max(curExpNs*targetHi/max(hi,1e-6),MINn),MAXn);
            tLong =min(max(curExpNs*targetLo/max(lo,1e-6),MINn),MAXn);
            if tLong<tShort, tLong=tShort; end
            ratio=tLong/tShort;
            if ratio<1.5, nLegs=1; else, nLegs=max(2,min(nLegsMax,round(log2(ratio))+1)); end
            if nLegs==1, legs=tShort;
            else,        legs=tShort.*(ratio.^((0:nLegs-1)/(nLegs-1))); end
        end

    end
end
