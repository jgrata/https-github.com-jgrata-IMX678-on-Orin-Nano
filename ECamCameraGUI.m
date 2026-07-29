classdef ECamCameraGUI < handle
%ECAMCAMERAGUI  Control panel for the IMX678/Jetson RAW-DAQ camera.
%
%   app = ECamCameraGUI                connect to the default direct-link IP
%   app = ECamCameraGUI('192.168.99.2',9000)
%   app = ECamCameraGUI(existingClient)   wrap an already-connected ECamHDRClient
%
%   A programmatic uifigure app (git-friendly, no .mlapp binary). Menus drive
%   actions; radio-button groups drive the enumerable settings (sensor mode,
%   dark / hot-pixel / CCM correction). A timer-driven live preview + histogram
%   run off the fast raw grab() path. Capture / Calibrate / Tools live in tabs.
%
%   The two Tools:
%     * ColorChecker  — click the 4 chart corners on the preview, solve a CCM
%                       (server deriveCCM) and read out the 24-patch fit.
%     * Rectangles    — find bright/dark rectangles of various sizes; a picked
%                       edge gets a slanted-edge sharpness readout, with an ROI
%                       hand-off hook for external MTF / license-plate tools.
%
%   Requires ECamHDRClient.m (+ EcamReceiver.dll) on the path.

    properties
        cam                         % ECamHDRClient
        Fig                         % uifigure
        h = struct()                % component handles
        LiveTimer                   % timer for the live preview
        LastRaw   = []              % last raw Bayer frame (uint16 [H W])
        LastLuma  = []              % last binned luma (single [H/2 W/2])
        LastRGB   = []              % last processed RGB (uint8 [H W 3]) for tools
        OwnsClient = true           % did we create cam (=> we disconnect it)
        Busy (1,1) logical = false  % a server transaction is in progress (gate live)
        PreviewFullRes (1,1) logical = false % displayed image is native full-res (vs half-res live)
        Squares = []                % found squares: struct array (verts, centroid, feret, angle, ar)
        OverlayH = gobjects(0)      % overlay graphics handles (boxes/centers/ROIs) for fast clear
        OpenCVMSER (1,1) logical = false  % mexopencv cv.MSER present (enables Min diversity)
        LiveMode = 'preview'        % 'preview' (fast color) | 'mtf' (focus-assist MTF loop)
        MTFAx                       % MTF plot axes (embedded in the Preview panel)
        MTFLines = gobjects(0)      % one line per edge ROI in the MTF plot
        MTFTextH = gobjects(0)      % per-edge MTF50 text overlays on the image
        ROIBoxH  = gobjects(0)      % per-edge ROI boxes on the image (for hover highlight)
        HoverIdx (1,1) double = 0   % currently hover-highlighted edge index (0 = none)
        History  = struct([])       % ring buffer of captured entries (last N seconds)
        Paused (1,1) logical = false% live MTF paused -> browsing history
        HistIdx  (1,1) double = 0   % index into History while paused
        ServerTimeOffset (1,1) double = 0  % server_epoch - pc_epoch (seconds)
        ColorHist = struct([])      % ColorChecker measurement history (last N seconds)
        ColorLast = struct([])      % most recent ColorChecker result (for CCM export)
        ColorRad  = []              % last chart capture's linear radiance [H W] (for ROI sampling)
        ColorKfold = struct([])     % last rigorous N-capture k-fold result (ColorAnalysis)
    end

    properties (Constant, Access = private)
        MODES = {'0: 4K 12-bit', '1: 4K 10-bit', '2: 4K 10-bit (alt)'}
        DARK_OPTS = {'off','scalar','auto','measured'}
        HOT_OPTS  = {'on','off'}
        CCM_OPTS  = {'none','vendor','derived'}
        JCODE_PATH  = 'C:\Users\JGrata\Documents\MATLAB\jcode'   % jslantedge + deps
        MTFGUI_PATH = 'C:\Users\JGrata\Documents\MATLAB\mtfgui'  % MSERRegionAnalyzer
        MERGE_OPTS  = {'average','largest','smallest'}
        DEFAULT_SAVEDIR = 'C:\Users\JGrata\Documents\MATLAB\GUIData'
        MAX_HIST = 1200             % hard cap on history entries (memory safety)
        ILLUM_OPTS = {'D65','D50','A','F (TL84)','unknown'}
        STD_ILLUM_NAME = {'A','F (TL84)','D50','D55','D65','D75'}
        STD_ILLUM_CCT  = [2856 4000 5003 5503 6504 7504]
        VENDOR_CCM = [2.14254 -0.97461 -0.16793; -0.57795 1.96805 -0.39010; 0.04265 -1.56983 2.52718]
        SRGB2XYZ   = [0.4124 0.3576 0.1805; 0.2126 0.7152 0.0722; 0.0193 0.1192 0.9505]
    end

    methods
        function app = ECamCameraGUI(host, port)
            if nargin >= 1 && isa(host, 'ECamHDRClient')
                app.cam = host; app.OwnsClient = false;
            else
                if nargin < 1 || isempty(host), host = '192.168.99.2'; end
                if nargin < 2 || isempty(port), port = 9000; end
                app.cam = ECamHDRClient(host, port);
            end
            if exist('jslantedge','file') ~= 2 && isfolder(app.JCODE_PATH)
                addpath(app.JCODE_PATH);           % jslantedge SFR + dependencies
            end
            if exist('MSERRegionAnalyzer','class') ~= 8 && isfolder(app.MTFGUI_PATH)
                addpath(app.MTFGUI_PATH);          % MSER square/rectangle finder
            end
            try, app.OpenCVMSER = (exist('cv.MSER','class')==8) || ~isempty(which('cv.MSER'));
            catch, app.OpenCVMSER = false; end     % mexopencv -> Min diversity slider
            app.buildUI();
            app.Fig.Position=[ 1          49        1280         707];
            if app.cam.IsConnected
                app.onConnected();
            end
        end

        function delete(app)
            app.stopLive();
            try, if ~isempty(app.LiveTimer) && isvalid(app.LiveTimer), delete(app.LiveTimer); end, end
            if app.OwnsClient
                try, app.cam.disconnect(); catch, end
            end
            try, if ~isempty(app.Fig) && isvalid(app.Fig), delete(app.Fig); end, end
        end
    end

    % ═══════════════════════════════ UI BUILD ═══════════════════════════════
    methods (Access = private)
        function buildUI(app)
            app.Fig = uifigure('Name','IMX678 / Jetson Camera Control', ...
                'Position',[80 80 1280 820], 'Color',[0.13 0.13 0.15], ...
                'CloseRequestFcn', @(~,~) app.delete());
            try, app.Fig.WindowButtonMotionFcn = @(~,~) app.onHover(); catch, end  % hover highlight
            app.buildMenus();

            g = uigridlayout(app.Fig, [4 2]);
            g.RowHeight    = {46, '1x', 40, 150};
            g.ColumnWidth  = {400, '1x'};
            g.RowSpacing   = 8;
            g.BackgroundColor = [0.13 0.13 0.15];

            app.buildTopBar(g);        % row1, spans both cols
            app.buildControlTabs(g);   % row2 col1
            app.buildPreview(g);       % row2 col2
            app.buildStatusBand(g);    % row3, spans both cols (large status + CCM badge)
            app.buildHistogram(g);     % row4, spans both cols
        end

        function buildStatusBand(app, g)
            % Large, always-visible status line (progress + results) + a persistent
            % badge showing which CCM the DISPLAYED image is rendered with. Sits in
            % the gap between the preview/panels and the histogram.
            p = uipanel(g,'BorderType','none','BackgroundColor',[0.10 0.10 0.12]);
            p.Layout.Row = 3; p.Layout.Column = [1 2];
            gb = uigridlayout(p,[1 2]); gb.ColumnWidth={'1x','fit'};
            gb.Padding=[12 2 12 2]; gb.BackgroundColor=[0.10 0.10 0.12];
            app.h.statusBand = uilabel(gb,'Text','ready','FontSize',16,'FontWeight','bold', ...
                'FontColor',[0.85 0.90 1.0],'VerticalAlignment','center');
            app.h.statusBand.Layout.Row=1; app.h.statusBand.Layout.Column=1;
            app.h.ccmBadge = uilabel(gb,'Text','Display CCM: —','FontSize',14,'FontWeight','bold', ...
                'FontColor',[0.75 0.82 0.95],'HorizontalAlignment','right','VerticalAlignment','center');
            app.h.ccmBadge.Layout.Row=1; app.h.ccmBadge.Layout.Column=2;
        end
        function setStatus(app, msg, col)
            if nargin<3 || isempty(col), col=[0.85 0.90 1.0]; end
            if isfield(app.h,'statusBand') && isvalid(app.h.statusBand)
                app.h.statusBand.Text = msg; app.h.statusBand.FontColor = col;
            end
            drawnow limitrate;
        end
        function s = ccmStateStr(app)
            s = '—';
            try
                c = app.cam.CCM;
                if ischar(c) || isstring(c),          s = 'vendor';
                elseif isnumeric(c) && ~isempty(c),   s = 'derived 3x3';
                else,                                 s = 'none (gray-world WB)';
                end
            catch
            end
        end
        function updateCCMBadge(app)
            if isfield(app.h,'ccmBadge') && isvalid(app.h.ccmBadge)
                app.h.ccmBadge.Text = ['Display CCM: ' app.ccmStateStr()];
            end
        end

        function buildMenus(app)
            m = uimenu(app.Fig, 'Text','&File');
            uimenu(m,'Text','Connect',    'MenuSelectedFcn',@(~,~)app.doConnect());
            uimenu(m,'Text','Disconnect', 'MenuSelectedFcn',@(~,~)app.doDisconnect());
            uimenu(m,'Text','Save current image...','Separator','on', ...
                   'MenuSelectedFcn',@(~,~)app.doSaveImage());
            uimenu(m,'Text','Close','Separator','on','MenuSelectedFcn',@(~,~)app.delete());

            c = uimenu(app.Fig,'Text','&Capture');
            uimenu(c,'Text','Grab raw frame','MenuSelectedFcn',@(~,~)app.doGrabOnce());
            uimenu(c,'Text','Single processed image','MenuSelectedFcn',@(~,~)app.doSingleCapture());
            uimenu(c,'Text','HDR bracket','MenuSelectedFcn',@(~,~)app.doHDR());
            uimenu(c,'Text','Start live','Separator','on','MenuSelectedFcn',@(~,~)app.startLive());
            uimenu(c,'Text','Stop live','MenuSelectedFcn',@(~,~)app.stopLive());

            k = uimenu(app.Fig,'Text','C&alibrate');
            uimenu(k,'Text','Measure dark (cap lens)','MenuSelectedFcn',@(~,~)app.doMeasureDark());
            uimenu(k,'Text','Build hot-pixel mask','MenuSelectedFcn',@(~,~)app.doBuildHotMask());
            uimenu(k,'Text','Sensor metrics','MenuSelectedFcn',@(~,~)app.doMetrics());

            t = uimenu(app.Fig,'Text','&Tools');
            uimenu(t,'Text','ColorChecker measure','MenuSelectedFcn',@(~,~)app.toolColorChecker());
            uimenu(t,'Text','Find squares (MSER)','MenuSelectedFcn',@(~,~)app.toolFindSquares());
            uimenu(t,'Text','Live MTF (focus assist)','MenuSelectedFcn',@(~,~)app.toggleLiveMTF());
            uimenu(t,'Text','MTF: click an edge','MenuSelectedFcn',@(~,~)app.toolEdgeMTFpick());
            uimenu(t,'Text','MTF: draw ROI','MenuSelectedFcn',@(~,~)app.toolEdgeMTF());

            hm = uimenu(app.Fig,'Text','&Help');
            uimenu(hm,'Text','About','MenuSelectedFcn',@(~,~)uialert(app.Fig, ...
                sprintf(['IMX678/Jetson camera control.\nProgrammatic uifigure app.\n' ...
                         'Drives ECamHDRClient over TCP.']),'About'));
        end

        function buildTopBar(app, g)
            p = uipanel(g,'BackgroundColor',[0.17 0.17 0.2],'BorderType','none');
            p.Layout.Row = 1; p.Layout.Column = [1 2];
            gg = uigridlayout(p,[1 9]);
            gg.ColumnWidth = {40,140,32,64,90,90,24,'1x',260};
            gg.BackgroundColor = [0.17 0.17 0.2]; gg.Padding=[8 6 8 6];
            uilabel(gg,'Text','Host','FontColor','w');
            app.h.host = uieditfield(gg,'text','Value',app.cam.Host);
            uilabel(gg,'Text','Port','FontColor','w');
            app.h.port = uieditfield(gg,'numeric','Value',app.cam.Port,'Limits',[1 65535], ...
                'RoundFractionalValues','on');
            app.h.connBtn = uibutton(gg,'Text','Connect','ButtonPushedFcn',@(~,~)app.doConnect());
            app.h.discBtn = uibutton(gg,'Text','Disconnect','ButtonPushedFcn',@(~,~)app.doDisconnect());
            app.h.lamp = uilabel(gg,'Text','●','FontColor',[0.8 0.2 0.2],'FontSize',20,'HorizontalAlignment','center');
            app.h.status = uilabel(gg,'Text','disconnected','FontColor',[0.8 0.8 0.8]);
            app.h.sensorInfo = uilabel(gg,'Text','','FontColor',[0.7 0.8 1.0],'HorizontalAlignment','right');
        end

        function buildControlTabs(app, g)
            tg = uitabgroup(g); tg.Layout.Row = 2; tg.Layout.Column = 1;
            app.buildCameraTab(uitab(tg,'Title','Camera'));
            app.buildCaptureTab(uitab(tg,'Title','Capture'));
            app.buildCalibrateTab(uitab(tg,'Title','Calibrate'));
            app.buildToolsTab(uitab(tg,'Title','Tools'));
            app.buildColorTab(uitab(tg,'Title','Color'));
        end

        function buildCameraTab(app, tab)
            gl = uigridlayout(tab,[7 1]);
            gl.RowHeight = {120, 150, 72, 72, 72, 'fit', '1x'};
            gl.Scrollable = 'on';

            % Sensor mode radio group
            bg = uibuttongroup(gl,'Title','Sensor mode', ...
                'SelectionChangedFcn',@(s,e)app.onModeChanged(e));
            app.h.modeBtns = gobjects(1,numel(app.MODES));
            for i=1:numel(app.MODES)   % direct children; y lowered so mode 0 isn't clipped
                app.h.modeBtns(i) = uiradiobutton(bg,'Text',app.MODES{i}, ...
                    'Position',[10 66-24*(i-1) 340 22]);
            end
            app.h.modeGroup = bg;

            % Exposure / gain / fps
            pe = uipanel(gl,'Title','Exposure & gain');
            ge = uigridlayout(pe,[3 3]); ge.ColumnWidth={'fit','1x','fit'};
            uilabel(ge,'Text','Exposure (ms)');
            app.h.exp = uieditfield(ge,'numeric','Value',33,'Limits',[0 2000], ...
                'ValueChangedFcn',@(s,~)app.onExposure(s.Value));
            app.h.aeBtn = uibutton(ge,'Text','Auto','ButtonPushedFcn',@(~,~)app.doAutoExp());
            uilabel(ge,'Text','Gain (x)');
            app.h.gain = uieditfield(ge,'numeric','Value',1,'Limits',[0 64], ...
                'ValueChangedFcn',@(s,~)app.onGain(s.Value));
            app.h.agBtn = uibutton(ge,'Text','AG once','ButtonPushedFcn',@(~,~)app.doAGOnce());
            uilabel(ge,'Text','FPS');
            app.h.fps = uieditfield(ge,'numeric','Value',30,'Limits',[1 120], ...
                'ValueChangedFcn',@(s,~)app.onFps(s.Value));
            app.h.lossless = uicheckbox(ge,'Text','Lossless','Value',true, ...
                'ValueChangedFcn',@(s,~)app.onLossless(s.Value));

            % Dark correction radio
            app.h.darkGroup = app.radioPanel(gl,'Dark correction',app.DARK_OPTS, ...
                app.cam.DarkCorrection, @(v)app.onDark(v));
            % Hot-pixel radio
            app.h.hotGroup = app.radioPanel(gl,'Hot-pixel correction',app.HOT_OPTS, ...
                app.cam.HotPixelCorrection, @(v)app.onHot(v));
            % CCM radio
            ccmInit = 'none';
            if ischar(app.cam.CCM)||isstring(app.cam.CCM), ccmInit=char(app.cam.CCM);
            elseif ~isempty(app.cam.CCM), ccmInit='derived'; end
            app.h.ccmGroup = app.radioPanel(gl,'Color matrix (CCM)',app.CCM_OPTS, ...
                ccmInit, @(v)app.onCCM(v));

            % Actual readback
            app.h.actual = uilabel(gl,'Text','actual: —','FontColor',[0.7 0.8 1.0]);
        end

        function bg = radioPanel(app, parent, title, opts, initVal, cb)
            bg = uibuttongroup(parent,'Title',title);
            for i=1:numel(opts)        % direct children (radios cannot live in a nested grid)
                rb = uiradiobutton(bg,'Text',opts{i},'Position',[8+88*(i-1) 8 86 22]);
                if strcmpi(opts{i},char(initVal)), bg.SelectedObject = rb; end
            end
            bg.SelectionChangedFcn = @(s,e) cb(e.NewValue.Text);
        end

        function buildCaptureTab(app, tab)
            gl = uigridlayout(tab,[4 1]); gl.RowHeight={'fit','fit','fit','1x'}; gl.Scrollable='on';
            % Single
            p1 = uipanel(gl,'Title','Single frame');
            g1 = uigridlayout(p1,[1 3]);
            uibutton(g1,'Text','Grab raw','ButtonPushedFcn',@(~,~)app.doGrabOnce());
            uibutton(g1,'Text','Processed image','ButtonPushedFcn',@(~,~)app.doSingleCapture());
            app.h.liveBtn = uibutton(g1,'Text','Start live','ButtonPushedFcn',@(~,~)app.toggleLive());
            % HDR bracket
            p2 = uipanel(gl,'Title','HDR bracket (onboard)');
            g2 = uigridlayout(p2,[3 2]); g2.ColumnWidth={'fit','1x'};
            uilabel(g2,'Text','Exposures (ms, comma-sep)');
            app.h.hdrExps = uieditfield(g2,'text','Value','2, 8, 32, 128');
            uilabel(g2,'Text','Gain (x)');
            app.h.hdrGain = uieditfield(g2,'numeric','Value',1,'Limits',[0 64]);
            uibutton(g2,'Text','Capture HDR','ButtonPushedFcn',@(~,~)app.doHDR());
            app.h.hdrCov = uilabel(g2,'Text','—');
            % Record
            p3 = uipanel(gl,'Title','Record (fast raw DAQ)');
            g3 = uigridlayout(p3,[1 3]); g3.ColumnWidth={'fit','1x','fit'};
            uilabel(g3,'Text','N frames');
            app.h.recN = uieditfield(g3,'numeric','Value',30,'Limits',[1 10000]);
            uibutton(g3,'Text','Record','ButtonPushedFcn',@(~,~)app.doRecord());
        end

        function buildCalibrateTab(app, tab)
            gl = uigridlayout(tab,[4 1]); gl.RowHeight={'fit','fit','fit','1x'}; gl.Scrollable='on';
            p1 = uipanel(gl,'Title','Dark (cap the lens!)');
            g1 = uigridlayout(p1,[2 3]); g1.ColumnWidth={'fit','1x','fit'};
            uilabel(g1,'Text','N frames'); app.h.darkN = uieditfield(g1,'numeric','Value',16);
            uibutton(g1,'Text','Measure dark','ButtonPushedFcn',@(~,~)app.doMeasureDark());
            uilabel(g1,'Text','Gain'); app.h.darkGain = uieditfield(g1,'numeric','Value',1);
            uibutton(g1,'Text','Build hot mask','ButtonPushedFcn',@(~,~)app.doBuildHotMask());
            p2 = uipanel(gl,'Title','Metrics');
            g2 = uigridlayout(p2,[1 3]); g2.ColumnWidth={'fit','1x','fit'};
            uilabel(g2,'Text','N frames'); app.h.metN = uieditfield(g2,'numeric','Value',16);
            uibutton(g2,'Text','Sensor metrics','ButtonPushedFcn',@(~,~)app.doMetrics());
            app.h.calOut = uitextarea(gl,'Editable','off','Value',{'Results appear here.'});
        end

        function buildToolsTab(app, tab)
            gl = uigridlayout(tab,[6 1]); gl.RowHeight={'fit','fit','fit','fit','fit','1x'}; gl.Scrollable='on';
            p1 = uipanel(gl,'Title','ColorChecker');
            g1 = uigridlayout(p1,[1 2]);
            uibutton(g1,'Text','Pick chart & measure','ButtonPushedFcn',@(~,~)app.toolColorChecker());
            app.h.ccApply = uicheckbox(g1,'Text','Apply derived CCM','Value',true);
            % ── Square / rectangle finder (MSER) ──
            p2 = uipanel(gl,'Title','Square / rectangle finder (MSER)');
            g2 = uigridlayout(p2,[7 4]); g2.ColumnWidth={'fit','1x','fit','1x'};
            g2.RowHeight={'fit','fit','fit',48,48,48,'fit'};
            uilabel(g2,'Text','Min area %');  app.h.sqMinPct = uieditfield(g2,'numeric','Value',1,'Limits',[1e-4 100]);
            uilabel(g2,'Text','Max area %');  app.h.sqMaxPct = uieditfield(g2,'numeric','Value',4,'Limits',[1e-3 100]);
            uilabel(g2,'Text','Min aspect');  app.h.sqMinAR  = uieditfield(g2,'numeric','Value',0.7,'Limits',[0 1]);
            uilabel(g2,'Text','Merge');       app.h.sqMerge  = uidropdown(g2,'Items',app.MERGE_OPTS,'Value','average');
            uilabel(g2,'Text','Along-edge');  app.h.sqAlong  = uieditfield(g2,'numeric','Value',0.7,'Limits',[0.05 1]);
            uilabel(g2,'Text','Across-edge'); app.h.sqAcross = uieditfield(g2,'numeric','Value',0.35,'Limits',[0.05 1]);
            % Delta slider (MSER intensity-threshold step)
            app.h.sqDeltaLbl = uilabel(g2,'Text','Delta 2'); app.h.sqDeltaLbl.Layout.Row=4; app.h.sqDeltaLbl.Layout.Column=1;
            app.h.sqDelta = uislider(g2,'Limits',[1 15],'Value',2,'MajorTicks',[1 2 5 10 15]);
            app.h.sqDelta.Layout.Row=4; app.h.sqDelta.Layout.Column=[2 4];
            app.h.sqDelta.ValueChangedFcn = @(s,~) set(app.h.sqDeltaLbl,'Text',sprintf('Delta %d',round(s.Value)));
            % Max variation slider (MSER stability)
            app.h.sqMaxVarLbl = uilabel(g2,'Text','Max var 0.25'); app.h.sqMaxVarLbl.Layout.Row=5; app.h.sqMaxVarLbl.Layout.Column=1;
            app.h.sqMaxVar = uislider(g2,'Limits',[0.05 1],'Value',0.25,'MajorTicks',[0.05 0.25 0.5 1]);
            app.h.sqMaxVar.Layout.Row=5; app.h.sqMaxVar.Layout.Column=[2 4];
            app.h.sqMaxVar.ValueChangedFcn = @(s,~) set(app.h.sqMaxVarLbl,'Text',sprintf('Max var %.2f',s.Value));
            % Min diversity slider (OpenCV cv.MSER only)
            if app.OpenCVMSER
                app.h.sqMinDivLbl = uilabel(g2,'Text','Min div 0.20'); app.h.sqMinDivLbl.Layout.Row=6; app.h.sqMinDivLbl.Layout.Column=1;
                app.h.sqMinDiv = uislider(g2,'Limits',[0 1],'Value',0.2,'MajorTicks',[0 0.2 0.5 1]);
                app.h.sqMinDiv.Layout.Row=6; app.h.sqMinDiv.Layout.Column=[2 4];
                app.h.sqMinDiv.ValueChangedFcn = @(s,~) set(app.h.sqMinDivLbl,'Text',sprintf('Min div %.2f',s.Value));
            else
                ld=uilabel(g2,'Text','Min div'); ld.Layout.Row=6; ld.Layout.Column=1;
                lm=uilabel(g2,'Text','(needs OpenCV cv.MSER)','FontColor',[0.55 0.55 0.55]);
                lm.Layout.Row=6; lm.Layout.Column=[2 4];
            end
            % Buttons
            bfind = uibutton(g2,'Text','Find squares','ButtonPushedFcn',@(~,~)app.toolFindSquares());
            bfind.Layout.Row=7; bfind.Layout.Column=[1 2];
            app.h.sqShowROI = uicheckbox(g2,'Text','Show edge ROIs','Value',true, ...
                'ValueChangedFcn',@(~,~)app.drawSquares());
            app.h.sqShowROI.Layout.Row=7; app.h.sqShowROI.Layout.Column=[3 4];
            % ── Slanted-edge MTF (jslantedge) ──
            p3 = uipanel(gl,'Title','Slanted-edge MTF (jslantedge)');
            g3 = uigridlayout(p3,[3 4]); g3.ColumnWidth={'fit','1x','fit','1x'};
            uilabel(g3,'Text','OSF');        app.h.mtfOSF   = uieditfield(g3,'numeric','Value',4,'Limits',[2 8]);
            uilabel(g3,'Text','Pitch (um)'); app.h.mtfPitch = uieditfield(g3,'numeric','Value',2.0,'Limits',[0.1 20]);
            uilabel(g3,'Text','EFL (mm)');   app.h.mtfEFL   = uieditfield(g3,'numeric','Value',8,'Limits',[0.1 1000]);
            uilabel(g3,'Text','X units');
            bgU = uibuttongroup(g3,'BorderType','none'); app.h.mtfUnits = bgU;
            uiradiobutton(bgU,'Text','mm','Position',[2 2 46 20]);
            rdeg = uiradiobutton(bgU,'Text','deg','Position',[52 2 55 20]); bgU.SelectedObject = rdeg;
            app.h.liveMTFbtn = uibutton(g3,'Text','Live MTF','ButtonPushedFcn',@(~,~)app.toggleLiveMTF());
            uibutton(g3,'Text','MTF: click edge','ButtonPushedFcn',@(~,~)app.toolEdgeMTFpick());
            uibutton(g3,'Text','MTF: draw ROI','ButtonPushedFcn',@(~,~)app.toolEdgeMTF());
            % ── Live capture & history ──
            p4 = uipanel(gl,'Title','Live capture & history');
            g4 = uigridlayout(p4,[4 4]); g4.ColumnWidth={'fit','1x','fit','1x'};
            app.h.pauseBtn = uibutton(g4,'Text','Pause','ButtonPushedFcn',@(~,~)app.togglePause());
            app.h.pauseBtn.Layout.Row=1; app.h.pauseBtn.Layout.Column=1;
            app.h.prevBtn = uibutton(g4,'Text','< Prev','Enable','off','ButtonPushedFcn',@(~,~)app.histStep(-1));
            app.h.prevBtn.Layout.Row=1; app.h.prevBtn.Layout.Column=2;
            app.h.nextBtn = uibutton(g4,'Text','Next >','Enable','off','ButtonPushedFcn',@(~,~)app.histStep(1));
            app.h.nextBtn.Layout.Row=1; app.h.nextBtn.Layout.Column=3;
            app.h.histInfo = uilabel(g4,'Text','history: 0'); app.h.histInfo.Layout.Row=1; app.h.histInfo.Layout.Column=4;
            lN=uilabel(g4,'Text','History (s)'); lN.Layout.Row=2; lN.Layout.Column=1;
            app.h.histN = uieditfield(g4,'numeric','Value',60,'Limits',[1 3600]);
            app.h.histN.Layout.Row=2; app.h.histN.Layout.Column=2;
            lD=uilabel(g4,'Text','Save dir'); lD.Layout.Row=3; lD.Layout.Column=1;
            app.h.savePath = uieditfield(g4,'text','Value',app.DEFAULT_SAVEDIR);
            app.h.savePath.Layout.Row=3; app.h.savePath.Layout.Column=[2 4];
            bSave=uibutton(g4,'Text','Save .mat','ButtonPushedFcn',@(~,~)app.doSaveHistory());
            bSave.Layout.Row=4; bSave.Layout.Column=1;
            bWs=uibutton(g4,'Text','To workspace','ButtonPushedFcn',@(~,~)app.doToWorkspace());
            bWs.Layout.Row=4; bWs.Layout.Column=2;
            app.h.thisBtn=uibutton(g4,'Text','This image','Enable','off','ButtonPushedFcn',@(~,~)app.doSaveThis());
            app.h.thisBtn.Layout.Row=4; app.h.thisBtn.Layout.Column=3;
            % ── Lighting (DMX): Waveform 3082 via ENTTEC Open DMX USB (PC-side, COM5) ──
            % Shells out to lab/dmx_lights.py. Exclusive with QLC+ (close QLC+ first).
            p5 = uipanel(gl,'Title','Lighting (DMX)  ch4=D65  ch5=Tungsten');
            g5 = uigridlayout(p5,[3 4]); g5.ColumnWidth={'fit','1x','fit','fit'}; g5.RowHeight={'fit','fit','fit'};
            app.h.dmxD65Lbl = uilabel(g5,'Text','D65 (ch4) 0'); app.h.dmxD65Lbl.Layout.Row=1; app.h.dmxD65Lbl.Layout.Column=1;
            app.h.dmxD65 = uislider(g5,'Limits',[0 255],'Value',0,'MajorTicks',[0 64 128 192 255]);
            app.h.dmxD65.Layout.Row=1; app.h.dmxD65.Layout.Column=[2 4];
            app.h.dmxD65.ValueChangedFcn = @(s,~) set(app.h.dmxD65Lbl,'Text',sprintf('D65 (ch4) %d',round(s.Value)));
            app.h.dmxTungLbl = uilabel(g5,'Text','Tungsten (ch5) 0'); app.h.dmxTungLbl.Layout.Row=2; app.h.dmxTungLbl.Layout.Column=1;
            app.h.dmxTung = uislider(g5,'Limits',[0 255],'Value',0,'MajorTicks',[0 64 128 192 255]);
            app.h.dmxTung.Layout.Row=2; app.h.dmxTung.Layout.Column=[2 4];
            app.h.dmxTung.ValueChangedFcn = @(s,~) set(app.h.dmxTungLbl,'Text',sprintf('Tungsten (ch5) %d',round(s.Value)));
            bDmxSet = uibutton(g5,'Text','Set lights','ButtonPushedFcn',@(~,~)app.dmxSet());
            bDmxSet.Layout.Row=3; bDmxSet.Layout.Column=1;
            bDmxOff = uibutton(g5,'Text','All off','ButtonPushedFcn',@(~,~)app.dmxOff());
            bDmxOff.Layout.Row=3; bDmxOff.Layout.Column=2;
            app.h.dmxStatus = uilabel(g5,'Text','close QLC+ to control','FontColor',[0.55 0.55 0.55]);
            app.h.dmxStatus.Layout.Row=3; app.h.dmxStatus.Layout.Column=[3 4];
            app.h.toolOut = uitextarea(gl,'Editable','off','Value',{'Tool output appears here.'});
        end

        function buildColorTab(app, tab)
            gl = uigridlayout(tab,[6 1]); gl.RowHeight={'fit','fit','fit','fit','fit','1x'}; gl.Scrollable='on';
            % Acquire
            p1 = uipanel(gl,'Title','Acquire ColorChecker');
            g1 = uigridlayout(p1,[3 4]); g1.ColumnWidth={'fit','1x','fit','1x'};
            bg = uibuttongroup(g1,'BorderType','none'); bg.Layout.Row=1; bg.Layout.Column=[1 2]; app.h.ccMode=bg;
            uiradiobutton(bg,'Text','Single','Position',[2 2 70 20]);
            rh=uiradiobutton(bg,'Text','HDR','Position',[76 2 60 20]); bg.SelectedObject=rh;
            uilabel(g1,'Text','Gain'); app.h.ccGain=uieditfield(g1,'numeric','Value',1,'Limits',[0 64]);
            uilabel(g1,'Text','Exposures (ms)'); app.h.ccExps=uieditfield(g1,'text','Value','2, 8, 32, 128');
            app.h.ccMeasBtn=uibutton(g1,'Text','Measure','ButtonPushedFcn',@(~,~)app.colorMeasure(), ...
                'Tooltip','Capture the chart, locate the 24 patches, and compute CIELAB colour error + a CCM.');
            app.h.ccMeasBtn.Layout.Row=3; app.h.ccMeasBtn.Layout.Column=[1 2];
            app.h.ccAuto=uicheckbox(g1,'Text','Auto-detect chart','Value',true, ...
                'Tooltip','Locate the chart automatically with IPT colorChecker (no clicking). Falls back to 4 manual clicks if it fails.');
            app.h.ccAuto.Layout.Row=3; app.h.ccAuto.Layout.Column=[3 4];
            % Illuminant
            p2 = uipanel(gl,'Title','Illuminant');
            g2 = uigridlayout(p2,[2 1]); g2.RowHeight={28,'fit'};
            bgi = uibuttongroup(g2,'BorderType','none','SelectionChangedFcn',@(~,~)[]); app.h.illum=bgi;
            for i=1:numel(app.ILLUM_OPTS)
                rb=uiradiobutton(bgi,'Text',app.ILLUM_OPTS{i},'Position',[4+68*(i-1) 4 66 20]);
                if strcmp(app.ILLUM_OPTS{i},'D65'), bgi.SelectedObject=rb; end
            end
            app.h.illumEst=uilabel(g2,'Text','Estimated: —  (measure to estimate)','FontColor',[0.7 0.85 1.0],'WordWrap','on');
            % Result — dedicated indicator fields (key/value) so nothing clips in
            % the narrow panel; free-text lines WordWrap. Explicit Layout so the
            % buttons always render (a spanning auto-placed child hides them).
            p3 = uipanel(gl,'Title','Result (dE = CIELAB error)');
            g3 = uigridlayout(p3,[8 3]); g3.ColumnWidth={'fit','1x','1x'};
            g3.RowHeight={'fit','fit','fit','fit','fit','fit','fit','fit'}; g3.RowSpacing=4;
            kc = [0.68 0.74 0.85];                                   % key-label colour
            kN=uilabel(g3,'Text','displayed dE','FontColor',kc, ...
                'Tooltip','dE of the currently DISPLAYED render (with the Display CCM shown in the status band).'); kN.Layout.Row=1; kN.Layout.Column=1;
            app.h.ccDEnow=uilabel(g3,'Text','—','FontWeight','bold'); app.h.ccDEnow.Layout.Row=1; app.h.ccDEnow.Layout.Column=[2 3];
            kW=uilabel(g3,'Text','derived-CCM dE','FontColor',kc, ...
                'Tooltip','dE if the freshly-DERIVED 3x3 were applied to the RAW sensor data (from this measurement).'); kW.Layout.Row=2; kW.Layout.Column=1;
            app.h.ccDEccm=uilabel(g3,'Text','—','FontWeight','bold'); app.h.ccDEccm.Layout.Row=2; app.h.ccDEccm.Layout.Column=[2 3];
            kR=uilabel(g3,'Text','residual','FontColor',kc); kR.Layout.Row=3; kR.Layout.Column=1;
            app.h.ccResid=uilabel(g3,'Text','—'); app.h.ccResid.Layout.Row=3; app.h.ccResid.Layout.Column=[2 3];
            app.h.ccPredict=uilabel(g3,'Text','if Apply CCM: —','FontColor',[0.8 0.85 0.95],'WordWrap','on');
            app.h.ccPredict.Layout.Row=4; app.h.ccPredict.Layout.Column=[1 3];
            b1=uibutton(g3,'Text','CCM->ws','ButtonPushedFcn',@(~,~)app.ccmToWorkspace(), ...
                'Tooltip','Copy the derived 3x3 CCM to the MATLAB base workspace as variable ''userCCM''.');
            b1.Layout.Row=5; b1.Layout.Column=1;
            b2=uibutton(g3,'Text','Apply CCM','ButtonPushedFcn',@(~,~)app.applyDerivedCCM(), ...
                'Tooltip','Set cam.CCM to the derived matrix so processed / HDR captures render corrected colour.');
            b2.Layout.Row=5; b2.Layout.Column=2;
            bp=uibutton(g3,'Text','Preview CCM','ButtonPushedFcn',@(~,~)app.previewCCM(), ...
                'Tooltip','Re-capture rendered WITH the derived CCM (does NOT change cam.CCM) and report the true rendered dE.');
            bp.Layout.Row=5; bp.Layout.Column=3;
            b3=uibutton(g3,'Text','Save .mat','ButtonPushedFcn',@(~,~)app.doSaveColor(), ...
                'Tooltip','Save the ColorChecker history (colours, dE, CCM, config) to a .mat in the Save dir.');
            b3.Layout.Row=6; b3.Layout.Column=1;
            app.h.ccStatus=uilabel(g3,'Text','pick a chart and Measure','FontColor',[0.7 0.8 1.0],'WordWrap','on');
            app.h.ccStatus.Layout.Row=7; app.h.ccStatus.Layout.Column=[1 3];
            bDn=uibutton(g3,'Text','Show none','ButtonPushedFcn',@(~,~)app.colorRenderCCM('none'), ...
                'Tooltip','Re-render the current capture with gray-world WB only (no CCM). Instant — same pixels, CCM swapped.');
            bDn.Layout.Row=8; bDn.Layout.Column=1;
            bDv=uibutton(g3,'Text','Show vendor','ButtonPushedFcn',@(~,~)app.colorRenderCCM('vendor'), ...
                'Tooltip','Re-render the current capture with the VENDOR CCM — revert from a derived-CCM display.');
            bDv.Layout.Row=8; bDv.Layout.Column=2;
            bDd=uibutton(g3,'Text','Show derived','ButtonPushedFcn',@(~,~)app.colorRenderCCM('derived'), ...
                'Tooltip','Re-render with the DERIVED CCM. Note: a chart-fit CCM extrapolates poorly outside the chart gamut (bright metal/specular can go magenta).');
            bDd.Layout.Row=8; bDd.Layout.Column=3;
            % History
            p4 = uipanel(gl,'Title','ColorChecker history');
            g4 = uigridlayout(p4,[2 4]); g4.ColumnWidth={'fit','1x','fit','1x'}; g4.RowHeight={'fit','fit'};
            lh=uilabel(g4,'Text','History (s)'); lh.Layout.Row=1; lh.Layout.Column=1;
            app.h.ccHistN=uieditfield(g4,'numeric','Value',10,'Limits',[1 600]); app.h.ccHistN.Layout.Row=1; app.h.ccHistN.Layout.Column=2;
            app.h.ccHistInfo=uilabel(g4,'Text','history: 0'); app.h.ccHistInfo.Layout.Row=1; app.h.ccHistInfo.Layout.Column=[3 4];
            bw=uibutton(g4,'Text','To workspace','ButtonPushedFcn',@(~,~)app.doColorToWorkspace(), ...
                'Tooltip','Copy the whole ColorChecker history struct to the base workspace.');
            bw.Layout.Row=2; bw.Layout.Column=[1 2];
            bt=uibutton(g4,'Text','This measurement','ButtonPushedFcn',@(~,~)app.doColorThis(), ...
                'Tooltip','Copy just the current measurement (colours, dE, CCM, config) to the base workspace.');
            bt.Layout.Row=2; bt.Layout.Column=[3 4];
            % Capture setup + rigorous k-fold (CIEDE2000, cross-validated)
            p5 = uipanel(gl,'Title','Capture setup + rigorous k-fold (CIEDE2000)');
            g5 = uigridlayout(p5,[5 4]); g5.ColumnWidth={'fit','1x','fit','1x'};
            g5.RowHeight={'fit','fit','fit','fit','fit'}; g5.RowSpacing=4;
            bMet=uibutton(g5,'Text','Auto-expose (meter chart)','ButtonPushedFcn',@(~,~)app.colorAutoExpose(), ...
                'Tooltip',['Meter the chart''s own patches: drop exposure until the brightest patch channel is ' ...
                'unclipped, then set an HDR bracket (brightest ~90% FS, darkest ~35% FS). Sets Exposures.']);
            bMet.Layout.Row=1; bMet.Layout.Column=[1 2];
            bDk=uibutton(g5,'Text','Check dark (cap lens)','ButtonPushedFcn',@(~,~)app.colorDarkCheck(), ...
                'Tooltip',['CAP THE LENS first. Captures a short + long dark and tests light-tightness ' ...
                '(exposure-invariant mean, uniform, ~pedestal) before trusting a measured black level.']);
            bDk.Layout.Row=1; bDk.Layout.Column=[3 4];
            lN=uilabel(g5,'Text','N captures'); lN.Layout.Row=2; lN.Layout.Column=1;
            app.h.ccN=uieditfield(g5,'numeric','Value',3,'Limits',[2 30],'RoundFractionalValues',true);
            app.h.ccN.Layout.Row=2; app.h.ccN.Layout.Column=2;
            lRt=uilabel(g5,'Text','Route'); lRt.Layout.Row=2; lRt.Layout.Column=3;
            app.h.ccRoute=uidropdown(g5,'Items',{'repeats','poses','intensity'},'Value','repeats', ...
                'Tooltip',['repeats = same framing (noise/repeatability); poses = reposition/rotate the ' ...
                'chart between captures (placement/glare); intensity = change illuminant level between ' ...
                'captures (sensor linearity / CCM intensity-invariance).']);
            app.h.ccRoute.Layout.Row=2; app.h.ccRoute.Layout.Column=4;
            lM=uilabel(g5,'Text','CCM model'); lM.Layout.Row=3; lM.Layout.Column=1;
            app.h.ccModel=uidropdown(g5,'Items',{'3x3 linear','+ root-poly deg2','+ root-poly deg3'}, ...
                'Value','3x3 linear','Tooltip','Root-poly is analysis only (compare xval to the 3x3); the exported CCM stays 3x3.');
            app.h.ccModel.Layout.Row=3; app.h.ccModel.Layout.Column=2;
            app.h.ccKfoldBtn=uibutton(g5,'Text','Run k-fold','ButtonPushedFcn',@(~,~)app.colorKfold(), ...
                'Tooltip','Acquire N captures, fit + leave-one-capture-out cross-validate, and report vendor vs derived ΔE00.');
            app.h.ccKfoldBtn.Layout.Row=3; app.h.ccKfoldBtn.Layout.Column=[3 4];
            app.h.ccKfoldRes=uilabel(g5,'Text','—','WordWrap','on','FontWeight','bold');
            app.h.ccKfoldRes.Layout.Row=4; app.h.ccKfoldRes.Layout.Column=[1 4];
            app.h.ccKfoldVerdict=uilabel(g5,'Text','','WordWrap','on','FontColor',[0.7 0.8 1.0]);
            app.h.ccKfoldVerdict.Layout.Row=5; app.h.ccKfoldVerdict.Layout.Column=[1 4];
            % Swatch comparison (measured top / reference bottom)
            app.h.swatchAx = uiaxes(gl); app.h.swatchAx.XTick=[]; app.h.swatchAx.YTick=[];
            title(app.h.swatchAx,'24 patches: top = measured+CCM, bottom = reference','Color','w');
            app.h.swatchAx.Color=[0.15 0.15 0.17]; app.h.swatchAx.Title.Color='w';
        end

        function buildPreview(app, g)
            p = uipanel(g,'Title','Preview','BackgroundColor',[0.1 0.1 0.12], ...
                'ForegroundColor','w','BorderType','none');
            p.Layout.Row = 2; p.Layout.Column = 2;
            gl = uigridlayout(p,[2 1]); gl.RowHeight={26,'1x'}; gl.Padding=[2 2 2 2]; gl.RowSpacing=2;
            cs = uigridlayout(gl,[1 3]); cs.ColumnWidth={110,80,'1x'}; cs.Padding=[2 0 2 0];
            app.h.stretch = uicheckbox(cs,'Text','Auto-stretch','Value',false, ...
                'ValueChangedFcn',@(~,~)app.refreshDisplay());
            app.h.color = uicheckbox(cs,'Text','Color','Value',true, ...
                'ValueChangedFcn',@(~,~)app.refreshDisplay());
            uilabel(cs,'Text','');
            % image (left) + MTF plot (right), side-by-side inside the Preview
            body = uigridlayout(gl,[1 2]); body.ColumnWidth={'2x','1x'};
            body.Padding=[0 0 0 0]; body.ColumnSpacing=6; body.BackgroundColor=[0.1 0.1 0.12];
            app.h.ax = uiaxes(body); app.h.ax.Toolbar.Visible='on';
            app.h.ax.XTick=[]; app.h.ax.YTick=[];
            app.h.img = imagesc(app.h.ax, zeros(2,2)); axis(app.h.ax,'image');
            colormap(app.h.ax, gray(256));
            title(app.h.ax,'no image');
            axm = uiaxes(body);                          % dark theme so ticks/labels show
            axm.Color=[0.15 0.15 0.17]; axm.XColor=[.88 .88 .88]; axm.YColor=[.88 .88 .88];
            axm.GridColor=[.75 .75 .75]; axm.GridAlpha=0.35; axm.Box='on'; axm.FontSize=9;
            grid(axm,'on');
            title(axm,'MTF','Color','w');
            xlabel(axm,'freq','Color',[.88 .88 .88]); ylabel(axm,'MTF','Color',[.88 .88 .88]);
            ylim(axm,[0 1.05]);
            app.h.mtfPlotAx = axm; app.MTFAx = axm;
        end

        function buildHistogram(app, g)
            p = uipanel(g,'BackgroundColor',[0.13 0.13 0.15],'BorderType','none');
            p.Layout.Row = 4; p.Layout.Column = [1 2];
            gl = uigridlayout(p,[1 2]); gl.ColumnWidth={'1x',300};
            app.h.hax = uiaxes(gl); title(app.h.hax,'histogram');
            app.h.stats = uilabel(gl,'Text','min/max/mean/sat: —','FontColor','w', ...
                'VerticalAlignment','top');
        end
    end

    % ═══════════════════════════════ CONNECTION ═════════════════════════════
    methods (Access = private)
        function doConnect(app)
            try
                if ~app.cam.IsConnected
                    host = app.h.host.Value; port = app.h.port.Value;
                    % Host/Port are read-only after construction -> make a fresh
                    % client if the operator changed either field.
                    if ~strcmp(host, app.cam.Host) || port ~= app.cam.Port
                        app.cam = ECamHDRClient(host, port);
                        app.OwnsClient = true;
                    end
                    app.cam.connect();
                end
                app.onConnected();
            catch e
                uialert(app.Fig, e.message, 'Connect failed');
            end
        end

        function doDisconnect(app)
            app.stopLive();
            if app.OwnsClient, try, app.cam.disconnect(); catch, end, end
            app.h.lamp.FontColor=[0.8 0.2 0.2]; app.h.status.Text='disconnected';
        end

        function onConnected(app)
            app.h.lamp.FontColor=[0.2 0.8 0.3]; app.h.status.Text='connected';
            ci = app.cam.CameraInfo;
            if isstruct(ci) && isfield(ci,'sensor')
                app.h.sensorInfo.Text = sprintf('%s  %dx%d  %s', ...
                    ci.sensor, ci.width, ci.height, ci.method);
            end
            app.syncControlsFromCam();
        end

        function syncControlsFromCam(app)
            try
                m = app.cam.sensormode;
                if m+1>=1 && m+1<=numel(app.h.modeBtns)
                    app.h.modeGroup.SelectedObject = app.h.modeBtns(m+1);
                end
                app.h.gain.Value = max(app.cam.gain,0);
                if app.cam.exposure_ns>0, app.h.exp.Value = app.cam.exposure_ns/1e6; end
                app.h.fps.Value = app.cam.fps;
                app.h.lossless.Value = logical(app.cam.lossless);
                app.updateActual();
            catch
            end
        end

        function updateActual(app)
            try
                ae = app.cam.actual_exposure_ns; ag = app.cam.actual_gain_value;
                app.h.actual.Text = sprintf('actual: exp %.2f ms   gain %.3fx', ae/1e6, ag);
            catch
            end
        end
    end

    % ═══════════════════════════════ SETTINGS CB ════════════════════════════
    methods (Access = private)
        function onModeChanged(app, e)
            idx = find(app.h.modeBtns == e.NewValue) - 1;
            app.withBusy(sprintf('switching to mode %d...',idx), @() app.setMode(idx));
        end
        function setMode(app, idx)
            app.cam.sensormode = idx;   % blocks during pipeline re-init
            app.syncControlsFromCam();
        end
        function onExposure(app,ms)
            try, app.cam.exposure_ns = round(ms*1e6); app.updateActual();
            catch e, uialert(app.Fig,e.message,'Setting'); end
        end
        function onGain(app,v)
            try, app.cam.gain = v; app.updateActual();
            catch e, uialert(app.Fig,e.message,'Setting'); end
        end
        function onFps(app,v)
            try, app.cam.fps = v; catch e, uialert(app.Fig,e.message,'Setting'); end
        end
        function onLossless(app,v)
            try, app.cam.lossless = logical(v); catch e, uialert(app.Fig,e.message,'Setting'); end
        end
        function onDark(app,v),       app.cam.DarkCorrection=v; end
        function onHot(app,v),        app.cam.HotPixelCorrection=v; end
        function onCCM(app,v)
            switch v
                case 'none',    app.cam.CCM = [];
                case 'vendor',  app.cam.CCM = 'vendor';
                case 'derived'
                    if isnumeric(app.cam.CCM) && ~isempty(app.cam.CCM)
                        % keep existing derived matrix
                    else
                        uialert(app.Fig,'No derived CCM yet — run Tools > ColorChecker.','CCM');
                    end
            end
            app.updateCCMBadge();
        end
    end

    % ═══════════════════════════════ CAPTURE ════════════════════════════════
    methods (Access = private)
        function doGrabOnce(app)
            app.stopLive();
            app.guard(@() app.showRaw(app.cam.grab()));
        end

        function doSingleCapture(app)
            app.stopLive();                     % free the socket + keep the result on screen
            app.withBusy('processed capture...', @() app.singleCapture());
        end
        function singleCapture(app)
            [img,meta] = app.cam.captureImage(app.h.exp.Value*1e6, 'gain', app.h.gain.Value);
            if ~isempty(img), app.showRGB(img); end
            app.LastRGB = img;
            app.updateActual();
            app.setCal(sprintf('captured: black=%s  hotpix=%d', ...
                num2str(getfielddef(meta,'black_level_used','?')), ...
                getfielddef(meta,'hotpix_corrected',0)));
        end

        function doHDR(app)
            app.stopLive();
            app.withBusy('HDR bracket...', @() app.hdrCapture());
        end
        function hdrCapture(app)
            exps = str2double(strsplit(app.h.hdrExps.Value, ',')) * 1e6;
            exps = exps(~isnan(exps));
            if isempty(exps), uialert(app.Fig,'Enter exposures in ms.','HDR'); return; end
            [~,meta] = app.cam.captureHDROnboard(exps,'gain',app.h.hdrGain.Value, ...
                'fullpreview',true,'preview',false);
            if isfield(meta,'fullPreviewImage')
                app.showRGB(meta.fullPreviewImage); app.LastRGB = meta.fullPreviewImage;
            end
            cov = 100*meta.coverage.composite_covered_frac;
            app.h.hdrCov.Text = sprintf('coverage %.0f%%  %.2fs  hotpix %d', ...
                cov, meta.capture_s, getfielddef(meta,'hotpix_corrected',0));
        end

        function doRecord(app)
            app.stopLive();
            n = round(app.h.recN.Value);
            app.withBusy(sprintf('recording %d frames...',n), @() app.recordN(n));
        end
        function recordN(app, n)
            [frames,stats] = app.cam.recordFramesFast(n);
            app.setCal(sprintf('recorded %d frames, %.1f fps', size(frames,3), ...
                getfielddef(stats,'fps',NaN)));
            if ~isempty(frames), app.showRaw(frames(:,:,end)); end
        end

        function doAutoExp(app)
            app.withBusy('auto exposure...', @() app.autoExpRun());
        end
        function autoExpRun(app)
            e = app.cam.autoExposure(); app.h.exp.Value = e/1e6; app.updateActual();
        end
        function doAGOnce(app)
            app.withBusy('AG once...', @() app.agRun());
        end
        function agRun(app)
            g = app.cam.agOnce(); app.h.gain.Value = g; app.updateActual();
        end
    end

    % ═══════════════════════════════ LIVE ═══════════════════════════════════
    methods (Access = private)
        function toggleLive(app)
            if ~isempty(app.LiveTimer) && isvalid(app.LiveTimer) && ...
                    strcmp(app.LiveTimer.Running,'on')
                app.stopLive();
            else
                app.startLive();
            end
        end
        function startLive(app)
            if ~app.cam.IsConnected, uialert(app.Fig,'Connect first.','Live'); return; end
            app.stopLive();
            app.LiveTimer = timer('ExecutionMode','fixedSpacing','Period',0.25, ...
                'BusyMode','drop','TimerFcn',@(~,~)app.liveTick());
            start(app.LiveTimer);
            if isfield(app.h,'liveBtn'), app.h.liveBtn.Text='Stop live'; end
        end
        function killLiveTimer(app)
            if ~isempty(app.LiveTimer) && isvalid(app.LiveTimer)
                try, stop(app.LiveTimer); catch, end
                try, delete(app.LiveTimer); catch, end
            end
            app.LiveTimer = [];
        end
        function stopLive(app)
            app.killLiveTimer();
            app.LiveMode = 'preview'; app.Paused = false;
            if isfield(app.h,'liveBtn') && isvalid(app.h.liveBtn), app.h.liveBtn.Text='Start live'; end
            if isfield(app.h,'liveMTFbtn') && isvalid(app.h.liveMTFbtn), app.h.liveMTFbtn.Text='Live MTF'; end
            if isfield(app.h,'pauseBtn') && isvalid(app.h.pauseBtn), app.h.pauseBtn.Text='Pause'; end
            try, app.updateHistUI(); catch, end
        end
        function liveTick(app)
            if app.Busy, return; end           % a server op owns the socket — skip this tick
            try
                if strcmp(app.LiveMode,'mtf'), app.liveMTFtick();
                else,                          app.showRaw(app.cam.grab()); end
            catch
                app.stopLive();
            end
        end
        function tf = isLiveRunning(app)
            tf = ~isempty(app.LiveTimer) && isvalid(app.LiveTimer) && ...
                 strcmp(app.LiveTimer.Running,'on');
        end

        % ── live MTF focus-assist ──
        function toggleLiveMTF(app)
            if app.isLiveRunning() && strcmp(app.LiveMode,'mtf'), app.stopLive();
            else, app.startLiveMTF(); end
        end
        function startLiveMTF(app)
            if ~app.cam.IsConnected, uialert(app.Fig,'Connect first.','Live MTF'); return; end
            if ~app.mserAvailable()
                uialert(app.Fig,['MSER not available — needs the Computer Vision Toolbox ' ...
                    '(detectMSERFeatures) or mexopencv cv.MSER.'],'Live MTF'); return
            end
            app.stopLive();
            app.Paused = false; app.serverTimeInit();
            app.LiveMode = 'mtf';
            app.LiveTimer = timer('ExecutionMode','fixedSpacing','Period',0.2, ...
                'BusyMode','drop','TimerFcn',@(~,~)app.liveTick());
            start(app.LiveTimer);
            if isfield(app.h,'liveMTFbtn')&&isvalid(app.h.liveMTFbtn), app.h.liveMTFbtn.Text='Stop MTF'; end
            app.updateHistUI();
            app.setTool(['Live MTF running (squares re-found every frame). Adjust focus to maximise ' ...
                'MTF50 (yellow numbers / the plot). Pause to browse history; Save to write a .mat.']);
        end
        function liveMTFtick(app)
            % Per-frame: grab, re-find squares on THIS frame (so the ROIs stay
            % valid if the camera drifts during focus), compute per-edge MTF, and
            % refresh image + boxes + MTF50 labels + the MTF-curve plot.
            f = app.cam.grab();
            fp = single(f);
            R = fp(1:2:end,1:2:end);
            G = (fp(1:2:end,2:2:end)+fp(2:2:end,1:2:end))/2;
            B = fp(2:2:end,2:2:end);
            fs = 2^app.cam.bit_depth - 1;
            luma = 0.299*R + 0.587*G + 0.114*B;
            app.LastRaw = f; app.LastLuma = luma; app.PreviewFullRes = false;
            mr=mean(R(:)); mg=mean(G(:)); mb=mean(B(:)); mgr=mean([mr mg mb])+eps;
            img = cat(3, R*(mgr/(mr+eps)), G*(mgr/(mg+eps)), B*(mgr/(mb+eps)));   % WB (unscaled)
            app.Squares = app.findSquaresOn(luma);
            [m50, curves, fq, nyq, units] = app.computeEdgeMTFs(app.Squares, luma);
            dimg = app.applyStretch(img);                % imagesc-style full-range scaling
            app.setDisplay(dimg);
            title(app.h.ax, sprintf('live %dx%d  (%d squares)', size(dimg,2), size(dimg,1), numel(app.Squares)));
            app.drawSquaresWithMTF(app.Squares, m50);
            app.updateMTFPlot(curves, fq, nyq, units);
            app.updateHistogram(luma, fs);
            app.appendHistory(uint8(255*dimg), app.Squares, m50, curves, fq, nyq, units);
        end
        function [m50, curves, fq, nyq, units] = computeEdgeMTFs(app, sq, luma)
            [osf, pixel, units] = app.mtfScale(false);   % live is half-res
            nyq = 1/(2*pixel); fq = linspace(0, nyq, 100);
            ne = 4*numel(sq); m50 = nan(ne,1); curves = nan(ne,100);
            al = app.h.sqAlong.Value; ac = app.h.sqAcross.Value; idx = 0;
            for i=1:numel(sq)
                V = sq(i).verts;
                for e=1:4
                    idx = idx + 1;
                    roi = app.extractRoi(luma, app.edgeRoiCorners(V, e, al, ac));
                    if numel(roi) >= 64
                        try
                            [ff, mm] = jslantedge(roi, osf, pixel);
                            curves(idx,:) = interp1(ff, mm, fq, 'linear', 0);
                            m50(idx) = app.mtf50(ff, mm);
                        catch
                        end
                    end
                end
            end
        end
        function drawSquaresWithMTF(app, sq, m50)
            app.drawSquares();                            % boxes + centers + ROIs (clearOverlay + fresh)
            if isempty(sq), return; end
            al = app.h.sqAlong.Value; ac = app.h.sqAcross.Value; idx = 0;
            th = gobjects(4*numel(sq),1);
            hold(app.h.ax,'on');
            for i=1:numel(sq)
                V = sq(i).verts;
                for e=1:4
                    idx = idx + 1; M = mean(app.edgeRoiCorners(V,e,al,ac),1);
                    s = '--'; if idx<=numel(m50) && ~isnan(m50(idx)), s = sprintf('%.2f',m50(idx)); end
                    th(idx) = text(app.h.ax, M(1), M(2), s, 'Color','y', 'FontSize',9, ...
                        'FontWeight','bold', 'HorizontalAlignment','center', 'Clipping','on');
                end
            end
            hold(app.h.ax,'off');
            app.OverlayH = [app.OverlayH(isgraphics(app.OverlayH)); th(isgraphics(th))];
            app.MTFTextH = th;
            app.applyHover(app.HoverIdx);                  % keep highlight across live refresh
        end
        function updateMTFPlot(app, curves, fq, nyq, units)
            ax = app.MTFAx; if isempty(ax) || ~isvalid(ax), return; end
            LC = [.88 .88 .88]; ne = size(curves,1);
            if numel(app.MTFLines) ~= ne || any(~isgraphics(app.MTFLines(:)))
                cla(ax); hold(ax,'on'); grid(ax,'on');
                cm = lines(max(ne,1)); app.MTFLines = gobjects(ne,1);
                for k=1:ne, app.MTFLines(k) = plot(ax, nan, nan, '-', 'Color', cm(k,:), 'LineWidth',1.2); end
                yline(ax, 0.5, 'Color',[.6 .6 .6], 'LineStyle',':');
                xlabel(ax, units,'Color',LC); ylabel(ax,'MTF','Color',LC); ylim(ax,[0 1.05]);
            end
            for k=1:ne
                if isgraphics(app.MTFLines(k)), set(app.MTFLines(k), 'XData', fq, 'YData', curves(k,:)); end
            end
            xlim(ax, [0 max(nyq,eps)]);
            title(ax, sprintf('MTF  0..Nyq %.3g %s', nyq, units), 'Color','w');
            app.applyHover(app.HoverIdx);        % preserve hover highlight across refresh
            drawnow limitrate;
        end
        function clearMTFText(app)
            try, delete(app.MTFTextH(isgraphics(app.MTFTextH))); catch, end
            app.MTFTextH = gobjects(0);
        end

        % ── hover highlight (curve <-> ROI box) ──
        function applyHover(app, idx)
            for k=1:numel(app.MTFLines)
                if isgraphics(app.MTFLines(k)), app.MTFLines(k).LineWidth = 1.2; end
            end
            for k=1:numel(app.ROIBoxH)
                if isgraphics(app.ROIBoxH(k)), app.ROIBoxH(k).LineWidth = 1; end
            end
            if idx >= 1
                if idx<=numel(app.MTFLines) && isgraphics(app.MTFLines(idx)), app.MTFLines(idx).LineWidth = 3; end
                if idx<=numel(app.ROIBoxH) && isgraphics(app.ROIBoxH(idx)), app.ROIBoxH(idx).LineWidth = 3; end
            end
        end
        function onHover(app)
            if isempty(app.MTFLines) && isempty(app.ROIBoxH), return; end
            idx = app.hoverIndex();
            if idx == app.HoverIdx, return; end   % only redraw on change
            app.HoverIdx = idx; app.applyHover(idx);
        end
        function tf = ptInAxes(app, ax)           % is the mouse over this axes?
            tf = false;
            try
                p = getpixelposition(ax, true); cp = app.Fig.CurrentPoint;
                tf = cp(1)>=p(1) && cp(1)<=p(1)+p(3) && cp(2)>=p(2) && cp(2)<=p(2)+p(4);
            catch
            end
        end
        function idx = hoverIndex(app)
            idx = 0;
            if app.ptInAxes(app.h.ax)             % over image -> point in an ROI box?
                cp = app.h.ax.CurrentPoint; x = cp(1,1); y = cp(1,2);
                for k=1:numel(app.ROIBoxH)
                    if isgraphics(app.ROIBoxH(k)) && ...
                            inpolygon(x, y, app.ROIBoxH(k).XData, app.ROIBoxH(k).YData)
                        idx = k; return
                    end
                end
            end
            if app.ptInAxes(app.MTFAx)            % over plot -> nearest curve
                cp = app.MTFAx.CurrentPoint; x = cp(1,1); y = cp(1,2);
                xr = max(diff(app.MTFAx.XLim),eps); yr = max(diff(app.MTFAx.YLim),eps);
                best = inf; bidx = 0;
                for k=1:numel(app.MTFLines)
                    if isgraphics(app.MTFLines(k))
                        d = min(((app.MTFLines(k).XData-x)/xr).^2 + ((app.MTFLines(k).YData-y)/yr).^2);
                        if d < best, best = d; bidx = k; end
                    end
                end
                if sqrt(best) < 0.04, idx = bidx; end
            end
        end
    end

    % ═══════════════════════════════ HISTORY / SAVE ═════════════════════════
    methods (Access = private)
        function dmxSet(app)
            app.dmxSend(round(app.h.dmxD65.Value), round(app.h.dmxTung.Value));
        end
        function dmxOff(app)
            app.h.dmxD65.Value = 0; app.h.dmxTung.Value = 0;
            set(app.h.dmxD65Lbl,'Text','D65 (ch4) 0');
            set(app.h.dmxTungLbl,'Text','Tungsten (ch5) 0');
            app.dmxSend(0, 0);
        end
        function dmxSend(app, d65, tung)
            %DMXSEND  Set Waveform-3082 LED levels via lab/dmx_lights.py (ENTTEC Open
            %  DMX USB on COM5). Exclusive with QLC+ -- close QLC+ first.
            script = fullfile(fileparts(which('ECamCameraGUI')), 'lab', 'dmx_lights.py');
            if ~isfile(script)
                app.h.dmxStatus.Text = 'dmx_lights.py not found'; return
            end
            app.h.dmxStatus.Text = 'setting…'; drawnow;
            cmd = sprintf('python "%s" --d65 %d --tungsten %d --hold 1.5', script, d65, tung);
            [st, out] = system(cmd); out = strtrim(out);
            if st == 0
                app.h.dmxStatus.Text = sprintf('D65=%d  Tung=%d  set', d65, tung);
            elseif contains(lower(out),'denied') || contains(out,'Access')
                app.h.dmxStatus.Text = 'COM5 busy — close QLC+';
            else
                app.h.dmxStatus.Text = ['DMX err: ' out(1:min(48,numel(out)))];
            end
        end
        function togglePause(app)
            if ~strcmp(app.LiveMode,'mtf')
                uialert(app.Fig,'Start Live MTF first, then Pause to browse.','Pause'); return
            end
            if app.Paused, app.resumeLiveMTF(); else, app.pauseLiveMTF(); end
        end
        function pauseLiveMTF(app)
            app.killLiveTimer(); app.Paused = true;
            app.HistIdx = numel(app.History);
            if isfield(app.h,'pauseBtn'), app.h.pauseBtn.Text='Resume'; end
            if app.HistIdx>=1, app.showHistEntry(app.HistIdx); end
            app.updateHistUI();
        end
        function resumeLiveMTF(app)
            app.Paused = false;
            if isfield(app.h,'pauseBtn'), app.h.pauseBtn.Text='Pause'; end
            app.updateHistUI();
            app.LiveTimer = timer('ExecutionMode','fixedSpacing','Period',0.2, ...
                'BusyMode','drop','TimerFcn',@(~,~)app.liveTick());
            start(app.LiveTimer);
        end
        function histStep(app, d)
            if ~app.Paused || isempty(app.History), return; end
            app.HistIdx = min(max(app.HistIdx + d, 1), numel(app.History));
            app.showHistEntry(app.HistIdx); app.updateHistUI();
        end
        function showHistEntry(app, idx)
            if idx < 1 || idx > numel(app.History), return; end
            e = app.History(idx);
            app.Squares = e.squares;
            app.setDisplay(e.image);                 % stored uint8 colour (already scaled)
            title(app.h.ax, e.title, 'Interpreter','none');
            app.drawSquaresWithMTF(e.squares, e.mtf.m50);
            app.updateMTFPlot(e.mtf.curves, e.mtf.fq, e.mtf.nyq, e.mtf.units);
            app.updateHistogram(single(rgb2gray(e.image)), 255);
        end
        function appendHistory(app, img8, sq, m50, curves, fq, nyq, units)
            if app.Paused, return; end
            tsec = app.serverEpoch();
            e = struct('timestamp',tsec, 'title',app.entryTitle(tsec), 'image',img8, ...
                'squares',{sq}, 'mtf',struct('m50',m50,'curves',curves,'fq',fq,'nyq',nyq,'units',units), ...
                'cam',app.camSnapshot(), 'squareParams',app.sqParamSnapshot(), ...
                'mtfParams',app.mtfParamSnapshot());
            if isempty(app.History), app.History = e; else, app.History(end+1) = e; end
            ts = [app.History.timestamp];             % prune to last N seconds + hard cap
            app.History = app.History(ts >= (tsec - app.h.histN.Value));
            if numel(app.History) > app.MAX_HIST
                app.History = app.History(end-app.MAX_HIST+1:end);
            end
            app.updateHistUI();
        end
        function updateHistUI(app)
            n = numel(app.History);
            if isfield(app.h,'histInfo') && isvalid(app.h.histInfo)
                if app.Paused && n>0
                    app.h.histInfo.Text = sprintf('entry %d/%d', app.HistIdx, n);
                else
                    app.h.histInfo.Text = sprintf('history: %d', n);
                end
            end
            en = 'off'; if app.Paused && n>0, en = 'on'; end
            for f = {'prevBtn','nextBtn','thisBtn'}
                if isfield(app.h,f{1}) && isvalid(app.h.(f{1})), app.h.(f{1}).Enable = en; end
            end
        end
        function doSaveHistory(app)
            if isempty(app.History), uialert(app.Fig,'No history to save.','Save'); return; end
            dpath = app.h.savePath.Value; if isempty(dpath), dpath = app.DEFAULT_SAVEDIR; end
            if ~isfolder(dpath)
                try, mkdir(dpath); catch, uialert(app.Fig,['Cannot create ' dpath],'Save'); return; end
            end
            tsec = app.serverEpoch(); fn = fullfile(dpath, [app.entryFile(tsec) '.mat']);
            MTFData = struct('history',app.History, 'saved',app.entryTitle(tsec), ...
                'cam',app.camSnapshot(), 'squareParams',app.sqParamSnapshot(), ...
                'mtfParams',app.mtfParamSnapshot()); %#ok<NASGU>
            try
                save(fn,'MTFData','-v7.3');
                app.setTool(sprintf('Saved %d entries -> %s', numel(app.History), fn));
            catch e, uialert(app.Fig, e.message, 'Save'); end
        end
        function doToWorkspace(app)
            if isempty(app.History), uialert(app.Fig,'No history.','Workspace'); return; end
            vn = matlab.lang.makeValidName(app.entryTitle(app.serverEpoch()));
            MTFData = struct('history',app.History, 'cam',app.camSnapshot(), ...
                'squareParams',app.sqParamSnapshot(), 'mtfParams',app.mtfParamSnapshot());
            assignin('base', vn, MTFData);
            app.setTool(['History -> base workspace variable: ' vn]);
        end
        function doSaveThis(app)
            if ~app.Paused || isempty(app.History) || app.HistIdx<1
                uialert(app.Fig,'Pause and select an entry first.','This image'); return
            end
            e = app.History(app.HistIdx); vn = matlab.lang.makeValidName(e.title);
            assignin('base', vn, e);
            app.setTool(['Entry -> base workspace variable: ' vn]);
        end
        function serverTimeInit(app)
            app.ServerTimeOffset = 0;
            try
                si = app.cam.getInfo();
                if isstruct(si) && isfield(si,'server_time')
                    app.ServerTimeOffset = si.server_time - posixtime(datetime('now','TimeZone','local'));
                end
            catch
            end
        end
        function t = serverEpoch(app)
            t = posixtime(datetime('now','TimeZone','local')) + app.ServerTimeOffset;
        end
        function s = entryTitle(~, tsec, prefix)
            if nargin < 3, prefix = 'MTF Data'; end
            dt = datetime(tsec,'ConvertFrom','posixtime','TimeZone','local');
            s = [prefix ' ' char(datetime(dt,'Format','MM dd yyyy HH:mm:ss.SS'))];
        end
        function s = entryFile(~, tsec, prefix)
            if nargin < 3, prefix = 'MTF Data'; end
            dt = datetime(tsec,'ConvertFrom','posixtime','TimeZone','local');
            s = [prefix ' ' char(datetime(dt,'Format','MM dd yyyy HH-mm-ss.SS'))];
        end
        function s = camSnapshot(app)
            c = app.cam; s = struct();
            try, if isstruct(c.CameraInfo), s.info = c.CameraInfo; end, catch, end
            for f = {'sensormode','exposure_ns','gain','fps','bit_depth','hdr_exp_ratio', ...
                     'sat_threshold','lossless','actual_exposure_ns','actual_gain_value','Host','Port', ...
                     'DarkCorrection','HotPixelCorrection','CCM'}
                try, s.(f{1}) = c.(f{1}); catch, end
            end
        end
        function s = sqParamSnapshot(app)
            s = struct('minAreaPct',app.h.sqMinPct.Value, 'maxAreaPct',app.h.sqMaxPct.Value, ...
                'minAspect',app.h.sqMinAR.Value, 'merge',app.h.sqMerge.Value, ...
                'alongEdge',app.h.sqAlong.Value, 'acrossEdge',app.h.sqAcross.Value, ...
                'delta',round(app.h.sqDelta.Value), 'maxVariation',app.h.sqMaxVar.Value);
            if isfield(app.h,'sqMinDiv'), s.minDiversity = app.h.sqMinDiv.Value; end
        end
        function s = mtfParamSnapshot(app)
            s = struct('osf',round(app.h.mtfOSF.Value), 'pitch_um',app.h.mtfPitch.Value, ...
                'EFL_mm',app.h.mtfEFL.Value, 'units',app.mtfUnitStr());
        end
    end

    % ═══════════════════════════════ COLORCHECKER ═══════════════════════════
    methods (Access = private)
        function colorMeasure(app)
            if ~app.cam.IsConnected, uialert(app.Fig,'Connect first.','ColorChecker'); return; end
            app.stopLive();
            exps = str2double(strsplit(app.h.ccExps.Value,',')) * 1e6; exps = exps(~isnan(exps));
            if isempty(exps), uialert(app.Fig,'Enter exposures in ms.','ColorChecker'); return; end
            if strcmp(app.h.ccMode.SelectedObject.Text,'Single'), exps = exps(1); end
            g = app.h.ccGain.Value;
            % capture + display the chart
            app.withBusy('capturing chart...', @() app.colorCapture(exps, g));
            if isempty(app.LastRGB), return; end
            if app.h.ccAuto.Value                          % IPT colorChecker auto-detect + measureColor
                [chart, sc] = app.detectChartObj(app.LastRGB);
                if ~isempty(chart)
                    app.setColorStatus('Chart auto-detected (colorChecker) — measuring colour...',[0.6 0.9 0.6]);
                    app.withBusy('measuring colour...', @() app.colorRunChart(chart, sc, exps, g));
                    return
                end
                app.setColorStatus('Auto-detect failed — pick the 4 corners manually.',[1 .6 .3]);
            end
            corners = [];
            if isempty(corners)                            % manual fallback: 4 discrete clicks
                lbl = {'TL (dark-skin)','TR','BR','BL'}; pts = zeros(4,2); hpts = gobjects(4,1); ok = true;
                for k=1:4
                    app.setColorStatus(sprintf('Click corner %d/4 on the image: %s', k, lbl{k}), [1 1 0]);
                    try, pt = drawpoint(app.h.ax,'Color','y'); catch, ok=false; break; end
                    if ~isvalid(pt) || isempty(pt.Position), ok=false; break; end
                    pts(k,:) = pt.Position; hpts(k) = pt;                  % [x y]; keep marker visible
                end
                try, delete(hpts(isgraphics(hpts))); catch, end
                if ~ok, app.setColorStatus('corner selection cancelled',[1 .6 .3]); return; end
                corners = app.reorderForLongEdge([pts(:,2), pts(:,1)]);    % [row col], 6-edge on TL->TR
            end
            app.withBusy('deriving CCM + colour error...', @() app.colorRun(corners, exps, g));
        end
        function [chart, sc] = detectChartObj(app, rgb)
            % Detect the chart with IPT colorChecker, returning the OBJECT (which
            % carries ColorROIs + measureColor + orientation-correct patch order).
            % Detect on a downsampled + contrast-normalised copy (colorChecker is
            % unreliable on full 4K); sc is the downsample factor so ColorROIs can
            % be mapped back to full-res. chart = [] on failure.
            chart = []; sc = 1;
            if exist('colorChecker','file') ~= 2, return; end
            I = im2uint8(rgb); W = size(I,2);
            if W > 1400, sc = 1400/W; I = imresize(I, sc); end
            cand = {I};
            try, cand{end+1} = imadjust(I, stretchlim(I,0.01), []); catch, end
            for k = 1:numel(cand)
                try
                    c = colorChecker(cand{k});
                    if ~isempty(c.RegistrationPoints) && all(isfinite(c.RegistrationPoints(:)))
                        chart = c; return
                    end
                catch
                end
            end
        end
        function colorRunChart(app, chart, sc, exps, g)
            % Auto path: MATLAB does the correspondence. measureColor gives the dE
            % of the CURRENT rendering (matches what you see) + reference; we sample
            % the LINEAR radiance at chart.ColorROIs (same order) to fit the raw CCM.
            rad = app.ColorRad;
            dEcur = []; refLin = []; usedMC = false;
            try
                T = measureColor(chart);
                dEcur  = double(T.Delta_E);
                refLin = rgb2lin(lab2rgb([double(T.Reference_L) double(T.Reference_a) double(T.Reference_b)]));
                usedMC = true;
            catch
            end
            rois = [];                                       % ColorROIs is a 24x1 struct (field ROI=[x y w h])
            try, rois = vertcat(chart.ColorROIs.ROI) / sc; catch, end   % -> full-res coords
            if ~usedMC || isempty(rad) || size(rois,1) ~= 24
                % can't do the measureColor+linear path -> fall back to the corner path
                app.colorRun(app.reorderForLongEdge(app.regToCorners(chart, sc)), exps, g); return
            end
            meas = app.sampleROIsBayer(rad, rois);           % 24x3 raw linear, ColorROIs order
            [ccm, ~] = app.lsqCCM(meas, refLin);             % raw 3x3 (correspondence guaranteed)
            afterLin = min(max(meas * ccm', 0), 1);
            % residual on the CLIPPED values (consistent with dE): a bright patch
            % overshooting to >1 then clipping to white is correct colour, so it
            % must NOT inflate the reported residual (the unclipped RMS did).
            resid = sqrt(mean((afterLin - refLin).^2, 'all'));
            Lref = app.labFromLinSRGB(refLin);
            dEafter = sqrt(sum((app.labFromLinSRGB(afterLin) - Lref).^2, 2));
            [cct, name] = app.illuminantEstimate(mean(meas(19:21,:),1));
            R = struct('measured',meas, 'reference',refLin, 'ccm',ccm, ...
                'refSRGB',app.linToSRGB(refLin), 'afterSRGB',app.linToSRGB(afterLin), ...
                'dEcurrent',dEcur, 'dEafter',dEafter, ...
                'meanCurrent',mean(dEcur,'omitnan'), 'meanAfter',mean(dEafter,'omitnan'), ...
                'maxAfter',max(dEafter), 'illumCCT',cct, 'illumEst',name, ...
                'rois',rois, 'corners',app.regToCorners(chart, sc), 'residual',resid, ...
                'exposures_ns',exps, 'gain',g, 'illumSel',app.h.illum.SelectedObject.Text);
            app.ColorLast = R; app.colorDisplay(R); app.appendColorHist(R);
        end
        function c = regToCorners(~, chart, sc)
            rp = double(chart.RegistrationPoints) / sc;      % 4x2 [x y] -> [row col]
            c = [rp(:,2) rp(:,1)];
        end
        function meas = sampleROIsBayer(~, rad, rois)
            % median linear RGB in the central 60% of each ROI, straight from the
            % Bayer-domain radiance (RGGB phase planes -> no demosaic needed).
            [H,W]=size(rad); rad=rad(1:2*floor(H/2), 1:2*floor(W/2));   % ensure even dims
            R=rad(1:2:end,1:2:end); Gr=rad(1:2:end,2:2:end); Gb=rad(2:2:end,1:2:end); B=rad(2:2:end,2:2:end);
            G=(Gr+Gb)/2; [h2,w2]=size(R);
            n=size(rois,1); meas=zeros(n,3);
            for k=1:n
                x=rois(k,1); y=rois(k,2); w=rois(k,3); hh=rois(k,4);
                cx0=max(round((x+0.2*w)/2),1);  cx1=min(round((x+0.8*w)/2),w2);
                cy0=max(round((y+0.2*hh)/2),1); cy1=min(round((y+0.8*hh)/2),h2);
                if cx1<cx0||cy1<cy0, meas(k,:)=NaN; continue; end
                rr=R(cy0:cy1,cx0:cx1); gg=G(cy0:cy1,cx0:cx1); bb=B(cy0:cy1,cx0:cx1);
                meas(k,:)=[median(rr(:)) median(gg(:)) median(bb(:))];
            end
        end
        function c = reorderForLongEdge(~, c)
            % manual fallback: put the 6-patch (long) edge on TL->TR so the grid
            % lands on the patches even if the chart was picked portrait.
            if norm(c(2,:)-c(1,:)) < norm(c(4,:)-c(1,:))
                c = c([1 4 3 2],:);                        % rotate labelling 90 deg
            end
        end
        function colorCapture(app, exps, g)
            [rad, meta] = app.cam.captureHDROnboard(exps, 'gain', g, 'fullpreview', true, 'preview', false);
            app.ColorRad = rad;                              % linear radiance for ROI sampling
            if isfield(meta,'fullPreviewImage'), app.showRGB(meta.fullPreviewImage); end
        end
        function colorRenderCCM(app, which)
            %COLORRENDERCCM  Re-render the LAST captured chart radiance with a chosen
            %  display CCM (none/vendor/derived) — instant, same pixels, no capture.
            %  Also commits cam.CCM so future server renders match. Lets you flip
            %  back to vendor after a derived-CCM display corrupts out-of-gamut areas.
            if isempty(app.ColorRad)
                app.setColorStatus('Capture/Measure a chart first (no radiance to re-render).',[1 .6 .3]); return
            end
            switch which
                case 'derived'
                    if isempty(app.ColorLast) || ~isfield(app.ColorLast,'ccm') || ...
                            ~isnumeric(app.ColorLast.ccm) || isempty(app.ColorLast.ccm)
                        app.setColorStatus('No derived CCM yet — run Measure or k-fold first.',[1 .6 .3]); return
                    end
                    app.cam.CCM = app.ColorLast.ccm;
                case 'vendor', app.cam.CCM = 'vendor';
                otherwise,     app.cam.CCM = [];
            end
            app.showRGB(app.renderRadiance(app.ColorRad, app.cam.CCM));
            try                                            % keep the Camera-tab radio in sync
                nm = 'none';
                if ischar(app.cam.CCM), nm = 'vendor';
                elseif isnumeric(app.cam.CCM) && ~isempty(app.cam.CCM), nm = 'derived'; end
                rb = findall(app.h.ccmGroup,'Type','uiradiobutton','Text',nm);
                if ~isempty(rb), app.h.ccmGroup.SelectedObject = rb(1); end
            catch
            end
            app.setColorStatus(sprintf('Display CCM -> %s (re-rendered current capture).', app.ccmStateStr()),[0.6 0.9 0.6]);
        end
        function rgb8 = renderRadiance(app, rad, ccm)
            % Fast client render matching the server's neutral/vendor/derived logic
            % (2x2 RGGB bin -> WB or CCM -> robust normalise + gamma). Display only.
            [H,W]=size(rad); He=2*floor(H/2); We=2*floor(W/2); f=double(rad(1:He,1:We));
            R=f(1:2:end,1:2:end); Gr=f(1:2:end,2:2:end); Gb=f(2:2:end,1:2:end); B=f(2:2:end,2:2:end);
            rgb=cat(3,R,0.5*(Gr+Gb),B);
            if isempty(ccm)                              % none: gray-world WB
                rgb = app.grayWorld(rgb);
            elseif ischar(ccm) || isstring(ccm)          % vendor: WB then vendor CCM
                rgb = app.applyCCMimg(app.grayWorld(rgb), app.VENDOR_CCM);
            else                                         % derived 3x3 (folds WB)
                rgb = app.applyCCMimg(rgb, ccm);
            end
            rgb = max(rgb,0);
            v = sort(rgb(:)); n = v(max(1,round(numel(v)*0.995)));   % 99.5 pct (no Stats tbx)
            img = min(rgb / max(n,eps), 1) .^ (1/2.2);
            rgb8 = uint8(img*255);
        end
        function rgb = grayWorld(~, rgb)
            mu = squeeze(mean(mean(rgb,1),2));
            rgb(:,:,1)=rgb(:,:,1)*mu(2)/max(mu(1),eps);
            rgb(:,:,3)=rgb(:,:,3)*mu(2)/max(mu(3),eps);
        end
        function out = applyCCMimg(~, rgb, ccm)
            sz=size(rgb); out=reshape(reshape(rgb,[],3)*ccm', sz);
        end
        function colorAutoExpose(app)
            if ~app.cam.IsConnected, uialert(app.Fig,'Connect first.','Auto-expose'); return; end
            app.stopLive();
            app.withBusy('metering chart...', @() app.colorAutoExposeRun());
        end
        function colorAutoExposeRun(app)
            % Closed loop: drop exposure until the brightest patch channel is
            % unclipped, then set an HDR bracket (ColorAnalysis.meterBracket) that
            % puts the brightest ~90% FS (short leg) and darkest ~35% FS (long leg).
            maxv = 2^double(app.cam.native_bpp) - 1; black = 200; if maxv < 2000, black = 50; end
            curExp = double(app.cam.exposure_ns); hi = 1; lo = 0.01; ok = false;
            for it = 1:5
                raw = double(app.cam.grab());
                [rgbLin, rgb8] = app.binLinearAndPreview(raw, maxv, black);
                [chart, sc] = app.detectChartObj(rgb8);
                if isempty(chart)
                    app.setColorStatus('meter: chart not detected — frame the chart and retry.',[1 .6 .3]); return
                end
                rois = vertcat(chart.ColorROIs.ROI) / sc;
                [hi, lo] = app.meterLevels(rgbLin, rois); ok = true;
                if hi >= 0.97 && curExp > 1e5
                    curExp = max(round(curExp*0.5), 1e5); app.cam.exposure_ns = curExp; pause(0.8); continue
                end
                break
            end
            if ~ok, return; end
            [~, ~, legs, ratio] = ColorAnalysis.meterBracket(hi, lo, curExp);
            legsMs = legs/1e6;
            app.h.ccExps.Value = char(strjoin(compose('%.3g', legsMs), ', '));
            app.setColorStatus(sprintf('metered: chart DR %.0f:1 -> bracket %.1f:1, exposures set: %s ms', ...
                hi/max(lo,1e-6), ratio, char(strjoin(compose('%.3g',legsMs), ', '))), [0.6 0.9 0.6]);
        end
        function [rgbLin, rgb8] = binLinearAndPreview(~, raw, maxv, black)
            [H,W] = size(raw); He=2*floor(H/2); We=2*floor(W/2);
            f = max(double(raw(1:He,1:We)) - black, 0);
            R=f(1:2:end,1:2:end); Gr=f(1:2:end,2:2:end); Gb=f(2:2:end,1:2:end); B=f(2:2:end,2:2:end);
            rgbLin = cat(3,R,0.5*(Gr+Gb),B) / (maxv - black);
            w = rgbLin; mu = squeeze(mean(mean(w,1),2));
            w(:,:,1)=w(:,:,1)*mu(2)/max(mu(1),eps); w(:,:,3)=w(:,:,3)*mu(2)/max(mu(3),eps);
            v = sort(w(:)); n = v(max(1,round(numel(v)*0.995)));
            rgb8 = uint8(min(w/max(n,eps),1).^(1/2.2)*255);
        end
        function [hi, lo] = meterLevels(~, rgbLin, rois)
            % Sample central 50% of each ROI (in rgbLin's half-res coords). hi = the
            % brightest single channel (clip risk); lo = the darkest patch mean.
            [H,W,~] = size(rgbLin); n = size(rois,1); pm = nan(n,3);
            for k = 1:n
                x=rois(k,1); y=rois(k,2); w=rois(k,3); h=rois(k,4);
                x0=max(round(x+0.25*w),1); x1=min(round(x+0.75*w),W);
                y0=max(round(y+0.25*h),1); y1=min(round(y+0.75*h),H);
                if x1<x0 || y1<y0, continue; end
                pat = rgbLin(y0:y1, x0:x1, :);
                pm(k,:) = [median(reshape(pat(:,:,1),[],1)) median(reshape(pat(:,:,2),[],1)) ...
                           median(reshape(pat(:,:,3),[],1))];
            end
            pm = pm(~any(isnan(pm),2),:);
            if isempty(pm), hi=1; lo=0.01; return; end
            hi = max(pm(:)); lo = min(mean(pm,2));
        end
        function colorDarkCheck(app)
            if ~app.cam.IsConnected, uialert(app.Fig,'Connect first.','Dark check'); return; end
            app.stopLive();
            sel = uiconfirm(app.Fig,'Cap the lens now, then Continue.','Dark check', ...
                'Options',{'Continue','Cancel'},'DefaultOption',1,'CancelOption',2);
            if strcmp(sel,'Cancel'), return; end
            app.withBusy('dark check (short + long exposure)...', @() app.colorDarkCheckRun());
        end
        function colorDarkCheckRun(app)
            maxv = 2^double(app.cam.native_bpp) - 1; ped = 200; if maxv < 2000, ped = 50; end
            orig = double(app.cam.exposure_ns);
            ss = app.darkStats(0.2e6); ll = app.darkStats(30e6);
            app.cam.exposure_ns = round(orig);
            leak = ll.mean - ss.mean;
            uniform   = ll.blockspread < 8 && abs(ll.gradV) < 4 && abs(ll.gradH) < 4;
            invariant = leak < 2;
            r = {};
            if ~invariant, r{end+1} = sprintf('mean rises %.1f DN (0.2->30 ms, %.3f DN/ms) — light accumulating', leak, leak/29.8); end %#ok<AGROW>
            if ~uniform,   r{end+1} = sprintf('non-uniform (block spread %.1f, grad %.1f/%.1f) — directional leak', ll.blockspread, ll.gradV, ll.gradH); end %#ok<AGROW>
            if abs(ss.mean-ped) > 6, r{end+1} = sprintf('mean %.1f vs pedestal %.0f', ss.mean, ped); end %#ok<AGROW>
            if ~(invariant && uniform)
                app.setColorStatus(sprintf('DARK CHECK: possible LEAK — use the scalar pedestal, not a measured dark.  %s', strjoin(r,'; ')), [0.92 0.30 0.22]);
            else
                app.setColorStatus(sprintf('DARK CHECK: light-tight — leak %.2f DN over ramp, mean %.1f≈pedestal %.0f, std %.1f, uniform.  Measured dark is trustworthy.', leak, ss.mean, ped, ll.std), [0.35 0.80 0.40]);
            end
        end
        function s = darkStats(app, expNs)
            app.cam.exposure_ns = round(expNs); pause(1.0); app.cam.grab();   % settle + discard a frame
            f = double(app.cam.grab());
            He=2*floor(size(f,1)/2); We=2*floor(size(f,2)/2); f = f(1:He,1:We);
            s.mean = mean(f(:)); s.std = std(f(:));
            bh = floor(He/8); bw = floor(We/8); bl = zeros(8,8);
            for i=1:8, for j=1:8, bl(i,j) = mean(f((i-1)*bh+(1:bh), (j-1)*bw+(1:bw)), 'all'); end, end
            s.blockspread = max(bl(:)) - min(bl(:));
            s.gradV = mean(f(1:floor(He/2),:),'all') - mean(f(floor(He/2)+1:end,:),'all');
            s.gradH = mean(f(:,1:floor(We/2)),'all') - mean(f(:,floor(We/2)+1:end),'all');
        end
        function colorKfold(app)
            %COLORKFOLD  Rigorous N-capture measure with leave-one-capture-out
            %  cross-validation (CIEDE2000). Reuses the proven one-shot capture/
            %  detect/sample path per capture; all colour math via ColorAnalysis.
            %  Route: repeats (noise) | poses (reposition) | intensity (illuminant).
            if ~app.cam.IsConnected, uialert(app.Fig,'Connect first.','k-fold'); return; end
            app.stopLive();
            exps = str2double(strsplit(app.h.ccExps.Value,',')) * 1e6; exps = exps(~isnan(exps));
            if isempty(exps), uialert(app.Fig,'Enter exposures in ms.','k-fold'); return; end
            if strcmp(app.h.ccMode.SelectedObject.Text,'Single'), exps = exps(1); end
            N = round(app.h.ccN.Value); route = app.h.ccRoute.Value;
            g = app.h.ccGain.Value; model = app.h.ccModel.Value;
            refC = ColorAnalysis.colorcheckerLinear();
            measAll = []; foldId = []; ok = 0;
            for k = 1:N
                if k > 1 && ~strcmp(route,'repeats')
                    if strcmp(route,'poses'), what = 'Reposition / rotate the chart';
                    else,                     what = 'Change the illuminant intensity'; end
                    sel = uiconfirm(app.Fig, sprintf('Capture %d of %d: %s, then Continue.', k, N, what), ...
                        'k-fold', 'Options',{'Continue','Stop'}, 'DefaultOption',1, 'CancelOption',2);
                    if strcmp(sel,'Stop'), break; end
                end
                app.setColorStatus(sprintf('k-fold capture %d/%d...', k, N),[1 1 0]); drawnow;
                try
                    app.colorCapture(exps, g);
                    if isempty(app.ColorRad) || isempty(app.LastRGB), continue; end
                    [chart, sc] = app.detectChartObj(app.LastRGB);
                    if isempty(chart)
                        app.setColorStatus(sprintf('capture %d: chart not detected — skipped', k),[1 .6 .3]); continue
                    end
                    rois = vertcat(chart.ColorROIs.ROI) / sc;
                    if size(rois,1) ~= 24, continue; end
                    meas = app.sampleROIsBayer(app.ColorRad, rois);        % 24x3 linear (ROI order)
                    [~, measC, ~] = app.fitCCMoriented(meas, refC);        % orient to canonical order
                    measAll = [measAll; measC]; foldId = [foldId; k*ones(24,1)]; ok = ok + 1; %#ok<AGROW>
                catch e
                    app.setColorStatus(sprintf('capture %d error: %s', k, e.message),[1 .4 .3]);
                end
            end
            if ok < 2
                app.setColorStatus(sprintf('k-fold needs >=2 good captures (got %d)', ok),[1 .4 .3]); return
            end
            refAll = repmat(refC, ok, 1);
            isRP = contains(model,'root-poly'); degree = 2 + double(contains(model,'deg3'));
            [deFitLin, deXvalLin] = ColorAnalysis.crossValDE(measAll, refAll, 'linear', [], foldId);
            deFitRP = []; deXvalRP = [];
            if isRP
                [deFitRP, deXvalRP] = ColorAnalysis.crossValDE(measAll, refAll, 'rootpoly', degree, foldId);
            end
            s3 = mean(refAll) ./ max(mean(measAll),1e-9);              % per-channel gray-world
            vend = (measAll .* s3) * app.VENDOR_CCM';
            deVend = ColorAnalysis.deltaE2000(ColorAnalysis.linToLab(min(max(vend,0),1)), ...
                                              ColorAnalysis.linToLab(refAll));
            R = struct('nCaptures',ok, 'route',route, 'model',model, 'degree',degree, ...
                'deVendor',deVend, 'deFitLin',deFitLin, 'deXvalLin',deXvalLin, ...
                'deFitRP',deFitRP, 'deXvalRP',deXvalRP, ...
                'measAll',measAll, 'refAll',refAll, 'foldId',foldId);
            app.ColorKfold = R; app.colorKfoldDisplay(R);
        end
        function colorKfoldDisplay(app, R)
            v = mean(R.deVendor); xl = mean(R.deXvalLin); fl = mean(R.deFitLin);
            txt = sprintf('%d captures (%s) — ΔE00 mean:  vendor %.2f  |  3×3 xval %.2f (fit %.2f)', ...
                R.nCaptures, R.route, v, xl, fl);
            best = xl; bestName = 'derived 3×3';
            if ~isempty(R.deXvalRP)
                xr = mean(R.deXvalRP); fr = mean(R.deFitRP);
                txt = [txt sprintf('  |  root-poly deg%d xval %.2f (fit %.2f)', R.degree, xr, fr)];
                if xr < best, best = xr; bestName = sprintf('root-poly deg%d', R.degree); end
            end
            app.h.ccKfoldRes.Text = [txt '   [computed from RAW sensor data — independent of the Display CCM / Apply CCM]'];
            if best < v
                app.h.ccKfoldVerdict.Text = sprintf('%s wins vs vendor by %.2f ΔE00 (cross-validated).', bestName, v-best);
                app.h.ccKfoldVerdict.FontColor = [0.35 0.80 0.40];
            else
                app.h.ccKfoldVerdict.Text = sprintf('vendor is as good or better (by %.2f) — derived does not generalize past it.', best-v);
                app.h.ccKfoldVerdict.FontColor = [0.90 0.80 0.20];
            end
            app.setStatus(sprintf('k-fold: %d captures — derived %.2f vs vendor %.2f ΔE00 (from raw)', ...
                R.nCaptures, best, v), app.deColor(best));
        end
        function colorRun(app, corners, exps, g)
            [~, info] = app.cam.deriveCCM(corners, 'exposures', exps, 'gain', g);
            if ~isfield(info,'patches')
                app.setColorStatus('Server returned no patch samples (redeploy image_server.py).',[1 .4 .3]); return
            end
            meas = double(info.patches); ref = double(info.reference);   % 24x3 raw-linear / linear-sRGB
            [ccm, meas, ~] = app.fitCCMoriented(meas, ref);              % auto-orient + fit on client
            R = app.colorAnalyze(meas, ref, ccm);                        % sets clip-consistent R.residual
            R.corners = corners; R.exposures_ns = exps; R.gain = g;
            R.illumSel = app.h.illum.SelectedObject.Text;
            app.ColorLast = R;
            app.colorDisplay(R);
            app.appendColorHist(R);
        end
        function [ccm, meas2, resid] = fitCCMoriented(app, meas, ref)
            % The 6x4 chart has 4 patch-orderings that keep its shape (identity,
            % flip-LR, flip-UD, rot-180). Try each, keep the lowest-residual fit --
            % robust to whatever corner order colorChecker / the user gave.
            best = inf; ccm = eye(3); meas2 = meas; resid = inf;
            for m = {'id','lr','ud','rot180'}
                Mk = app.flipGrid(meas, m{1});
                [A, r] = app.lsqCCM(Mk, ref);
                if r < best, best = r; ccm = A; meas2 = Mk; resid = r; end
            end
        end
        function R = colorAnalyze(app, meas, ref, ccm)
            % meas,ref: 24x3 linear. WB from light-grey neutrals (patches 19-22),
            % dE76 in CIELAB before (WB only) and after the CCM. Illuminant est
            % from the neutrals via the vendor CCM -> XYZ -> CCT.
            neutral = meas(19:22,:);                       % white..mid grey
            wb = mean(neutral(:)) ./ max(mean(neutral,1), 1e-9);
            wbm = meas .* wb;
            s = mean(ref(19,:)) / max(mean(wbm(19,:)),1e-9);  % scale white to reference white
            beforeLin = min(max(wbm*s,0),1);
            afterLin  = min(max(meas * ccm', 0), 1);          % CCM folds WB -> linear sRGB
            Lref = app.labFromLinSRGB(ref);
            dEbefore = sqrt(sum((app.labFromLinSRGB(beforeLin)-Lref).^2, 2));
            dEafter  = sqrt(sum((app.labFromLinSRGB(afterLin) -Lref).^2, 2));
            resid = sqrt(mean((afterLin - ref).^2, 'all'));   % clip-consistent (matches dE)
            [cct, name] = app.illuminantEstimate(mean(meas(19:21,:),1));
            R = struct('measured',meas, 'reference',ref, 'ccm',ccm, 'wb',wb, ...
                'beforeSRGB',app.linToSRGB(beforeLin), 'afterSRGB',app.linToSRGB(afterLin), ...
                'refSRGB',app.linToSRGB(ref), 'dEbefore',dEbefore, 'dEafter',dEafter, ...
                'meanBefore',mean(dEbefore,'omitnan'), 'meanAfter',mean(dEafter,'omitnan'), ...
                'maxAfter',max(dEafter), 'illumCCT',cct, 'illumEst',name, 'residual',resid);
        end
        function [cct, name] = illuminantEstimate(app, grayLin)
            % approximate (uncalibrated): vendor CCM as camera->linear sRGB, then
            % sRGB->XYZ->xy->McCamy CCT, nearest standard illuminant.
            srgbLin = grayLin * app.VENDOR_CCM';
            XYZ = max(srgbLin,0) * app.SRGB2XYZ';
            if sum(XYZ) <= 0, cct = NaN; name = 'unknown'; return; end
            xy = XYZ(1:2) / sum(XYZ);
            cct = app.mccamyCCT(xy(1), xy(2));
            [~,k] = min(abs(app.STD_ILLUM_CCT - cct)); name = app.STD_ILLUM_NAME{k};
        end
        function colorDisplay(app, R)
            app.clearOverlay();
            % overlay patch centres (from ROIs if auto, else interpolated corners)
            if isfield(R,'rois') && ~isempty(R.rois)
                ctrs = [R.rois(:,2)+R.rois(:,4)/2, R.rois(:,1)+R.rois(:,3)/2];   % [row col]
            else
                ctrs = app.patchCenters(R.corners);
            end
            hold(app.h.ax,'on');
            hc = plot(app.h.ax, ctrs(:,2), ctrs(:,1), 'o', 'Color',[1 1 0], 'MarkerSize',5, 'LineWidth',1);
            app.OverlayH = hc; hold(app.h.ax,'off');
            % primary dE = rendered accuracy (auto/measureColor) if available, else best-CCM
            hasCur = isfield(R,'dEcurrent') && ~isempty(R.dEcurrent);
            if hasCur, primary=R.dEcurrent; meanP=R.meanCurrent; second=R.dEafter;  plab='current (rendered)'; slab='with new CCM';
            else,      primary=R.dEafter;   meanP=R.meanAfter;   second=R.dEbefore; plab='with CCM';           slab='WB only'; end
            ax = app.MTFAx; LC=[.88 .88 .88];
            if ~isempty(ax) && isvalid(ax)
                cla(ax); hold(ax,'on'); grid(ax,'on');
                b = bar(ax, 1:24, primary(:), 'FaceColor','flat');
                for p=1:24, b.CData(p,:) = app.deColor(primary(p)); end
                plot(ax, 1:24, second(:), '-o', 'Color',[.6 .6 .6], 'MarkerSize',3, 'LineWidth',0.5);
                yline(ax, 1, 'Color',[.3 .85 .3], 'LineStyle',':');
                yline(ax, 3, 'Color',[.95 .35 .2], 'LineStyle',':');
                xlabel(ax,'patch','Color',LC); ylabel(ax,'dE (CIELAB)','Color',LC);
                ymax = max([primary(:); second(:); 3.5]);
                xlim(ax,[0 25]); ylim(ax,[0 ymax*1.1]);                 % <-- was clipped to [0 1.05]
                title(ax, sprintf('dE %s (grey=%s)  mean %.2f', plab, slab, meanP), 'Color','w');
                hold(ax,'off'); app.MTFLines = gobjects(0);
            end
            app.drawSwatches(R);
            col = app.deColor(meanP);
            if hasCur
                app.h.ccDEnow.Text = sprintf('%.2f   (max %.2f)', R.meanCurrent, max(R.dEcurrent));
                app.h.ccDEccm.Text = sprintf('%.2f', R.meanAfter);
                app.h.ccDEnow.FontColor = app.deColor(R.meanCurrent);
            else
                app.h.ccDEnow.Text = sprintf('%.2f   (WB only)', R.meanBefore);
                app.h.ccDEccm.Text = sprintf('%.2f   (max %.2f)', R.meanAfter, R.maxAfter);
                app.h.ccDEnow.FontColor = app.deColor(R.meanBefore);
            end
            app.h.ccDEccm.FontColor = app.deColor(R.meanAfter);
            app.h.ccResid.Text = sprintf('%.4f', R.residual);
            if meanP < 1,      verdict = 'dE<1: colour accurate, no CCM update needed';
            elseif meanP < 3,  verdict = 'dE 1-3: ok, CCM may need updating';
            else,              verdict = 'dE>3: colour off, CCM needs update'; end
            app.setColorStatus(verdict, col);
            % predicted dE if the DERIVED CCM were applied to the raw (linear-fit estimate)
            dApp = R.meanAfter;
            if hasCur
                if dApp < R.meanCurrent - 1
                    rec = sprintf('if Apply CCM: dE ~ %.2f  (now %.2f)  -> likely improves', dApp, R.meanCurrent);
                else
                    rec = sprintf('if Apply CCM: dE ~ %.2f  (now %.2f)  -> little gain (illuminant-limited)', dApp, R.meanCurrent);
                end
            else
                rec = sprintf('if Apply CCM: dE ~ %.2f  (WB-only %.2f)', dApp, R.meanBefore);
            end
            if isfield(app.h,'ccPredict') && isvalid(app.h.ccPredict)
                app.h.ccPredict.Text = [rec '   (est.)']; app.h.ccPredict.FontColor = app.deColor(dApp);
            end
            app.h.illumEst.Text = sprintf('Estimated: %s  (~%.0f K)   [selected: %s]', ...
                R.illumEst, R.illumCCT, R.illumSel);
        end
        function setColorStatus(app, msg, col)
            if nargin < 3, col = [0.7 0.8 1.0]; end
            if isfield(app.h,'ccStatus') && isvalid(app.h.ccStatus)
                app.h.ccStatus.Text = msg; app.h.ccStatus.FontColor = col;
            end
            app.setStatus(msg, col);            % mirror to the large status band
            drawnow limitrate;
        end
        function drawSwatches(app, R)
            ax = app.h.swatchAx; if isempty(ax)||~isvalid(ax), return; end
            ph=24; pw=24; img=zeros(4*2*ph, 6*pw, 3);   % each cell: meas(top ph) / ref(bottom ph)
            for p=1:24
                i=ceil(p/6); j=mod(p-1,6)+1;
                r0=(i-1)*2*ph; c0=(j-1)*pw;
                mc=reshape(min(max(R.afterSRGB(p,:),0),1),1,1,3);
                rc=reshape(min(max(R.refSRGB(p,:),0),1),1,1,3);
                img(r0+(1:ph),   c0+(1:pw), :) = repmat(mc, ph, pw);
                img(r0+ph+(1:ph),c0+(1:pw), :) = repmat(rc, ph, pw);
            end
            imshow(img,'Parent',ax);
            title(ax,'24 patches (chart layout): top half = measured+CCM, bottom half = reference','Color','w');
        end
        function c = patchCenters(~, corners)
            TL=corners(1,:); TR=corners(2,:); BR=corners(3,:); BL=corners(4,:);
            c=zeros(24,2); idx=0;
            for i=1:4
                v=(i-0.5)/4;
                for j=1:6
                    u=(j-0.5)/6; idx=idx+1;
                    top=TL+(TR-TL)*u; bot=BL+(BR-BL)*u;
                    c(idx,:)=top+(bot-top)*v;                 % [row col]
                end
            end
        end
        function appendColorHist(app, R)
            tsec = app.serverEpoch();
            thumb = []; try, thumb = imresize(app.LastRGB, [NaN 480]); catch, thumb = app.LastRGB; end
            e = struct('timestamp',tsec, 'title',app.entryTitle(tsec,'ColorChecker Data'), ...
                'thumb',thumb, 'measured',R.measured, 'reference',R.reference, 'ccm',R.ccm, ...
                'dEcurrent',{getfielddef(R,'dEcurrent',[])}, 'dEafter',R.dEafter, ...
                'meanCurrent',getfielddef(R,'meanCurrent',NaN), 'meanAfter',R.meanAfter, ...
                'dEbefore',{getfielddef(R,'dEbefore',[])}, 'meanBefore',getfielddef(R,'meanBefore',NaN), ...
                'illumEst',R.illumEst, 'illumCCT',R.illumCCT, ...
                'illumSel',R.illumSel, 'residual',R.residual, 'cam',app.camSnapshot());
            if isempty(app.ColorHist), app.ColorHist = e; else, app.ColorHist(end+1) = e; end
            ts = [app.ColorHist.timestamp];
            app.ColorHist = app.ColorHist(ts >= (tsec - app.h.ccHistN.Value));
            app.updateColorHistUI();
        end
        function updateColorHistUI(app)
            if isfield(app.h,'ccHistInfo') && isvalid(app.h.ccHistInfo)
                app.h.ccHistInfo.Text = sprintf('history: %d', numel(app.ColorHist));
            end
        end
        function ccmToWorkspace(app)
            if isempty(app.ColorLast), app.setColorStatus('Measure first.',[1 .6 .3]); return; end
            assignin('base','userCCM', app.ColorLast.ccm);
            uialert(app.Fig, sprintf(['Copied the derived 3x3 CCM to the MATLAB base workspace as ' ...
                '''userCCM''.\n\nIt maps raw linear sensor RGB -> linear sRGB (white balance folded in) ' ...
                'to minimise the ColorChecker colour error (chart mean dE = %.2f).\n\nUse it in code, e.g.:' ...
                '\n   [img,m] = cam.captureImage(20e6, ''ccm'', userCCM);'], app.ColorLast.meanAfter), ...
                'CCM -> workspace', 'Icon','info');
            app.setColorStatus('CCM copied to workspace var ''userCCM''.',[0.6 0.9 0.6]);
        end
        function applyDerivedCCM(app)
            if isempty(app.ColorLast), app.setColorStatus('Measure first.',[1 .6 .3]); return; end
            app.cam.CCM = app.ColorLast.ccm;
            if isfield(app.h,'ccmGroup')
                rb = findall(app.h.ccmGroup,'Type','uiradiobutton','Text','derived');
                if ~isempty(rb), app.h.ccmGroup.SelectedObject = rb(1); end
            end
            dE = app.ColorLast.meanAfter;
            if dE >= 3
                uialert(app.Fig, sprintf(['Applied the derived CCM to cam.CCM.\n\nWARNING: chart mean ' ...
                    'dE = %.1f (>3) is a poor fit (usually chart mis-alignment or a poor capture), so ' ...
                    'colour may look wrong. Re-measure for a green result before relying on it.'], dE), ...
                    'Apply CCM', 'Icon','warning');
            else
                uialert(app.Fig, sprintf(['Applied the derived CCM to cam.CCM (chart mean dE = %.2f).\n\n' ...
                    'Processed and HDR captures (captureImage / captureHDROnboard) now render corrected ' ...
                    'colour with this matrix. Set CCM = ''none'' on the Camera tab to revert.'], dE), ...
                    'Apply CCM', 'Icon','success');
            end
            app.setColorStatus(sprintf('Applied derived CCM to cam.CCM (dE %.2f) — future captures render with it.',dE), app.deColor(dE));
            app.updateCCMBadge();
        end
        function previewCCM(app)
            if isempty(app.ColorLast) || ~isnumeric(app.ColorLast.ccm) || isempty(app.ColorLast.ccm)
                app.setColorStatus('Measure first (need a derived CCM to preview).',[1 .6 .3]); return
            end
            app.stopLive();
            app.withBusy('preview: re-rendering with the derived CCM...', @() app.previewCCMrun());
        end
        function previewCCMrun(app)
            % Non-destructive: re-capture rendered WITH the derived CCM (passed per
            % call, cam.CCM untouched), then measureColor -> the TRUE rendered dE.
            R = app.ColorLast;
            [~, meta] = app.cam.captureHDROnboard(R.exposures_ns, 'gain', R.gain, ...
                'fullpreview', true, 'preview', false, 'ccm', R.ccm);
            if ~isfield(meta,'fullPreviewImage'), app.setColorStatus('Preview capture failed.',[1 .4 .3]); return; end
            app.showRGB(meta.fullPreviewImage);              % show the CCM-corrected image
            dE = NaN;
            chart = app.detectChartObj(meta.fullPreviewImage);
            if ~isempty(chart), try, T = measureColor(chart); dE = mean(T.Delta_E); catch, end, end
            if isnan(dE)
                app.setColorStatus('Preview image shown, but the chart could not be re-detected to measure dE.',[1 .6 .3]); return
            end
            cur = getfielddef(R,'meanCurrent',NaN);
            if isnan(cur), cur = getfielddef(R,'meanBefore',NaN); end
            c = app.deColor(dE);
            app.h.ccPredict.Text = sprintf('Preview: TRUE rendered dE with new CCM = %.2f   (est. was %.2f)', dE, R.meanAfter);
            app.h.ccPredict.FontColor = c;
            if ~isnan(cur) && dE < cur - 1
                app.setColorStatus(sprintf('Preview rendered dE %.2f vs current %.2f -> Apply improves. (preview only — click Apply to keep)', dE, cur), c);
            else
                app.setColorStatus(sprintf('Preview rendered dE %.2f vs current %.2f -> little gain. (preview only)', dE, cur), c);
            end
        end
        function doSaveColor(app)
            if isempty(app.ColorHist), app.setColorStatus('No ColorChecker history.',[1 .6 .3]); return; end
            dpath = app.h.savePath.Value; if isempty(dpath), dpath = app.DEFAULT_SAVEDIR; end
            if ~isfolder(dpath), try, mkdir(dpath); catch, app.setColorStatus(['Cannot create ' dpath],[1 .4 .3]); return; end, end
            tsec = app.serverEpoch(); fn = fullfile(dpath, [app.entryFile(tsec,'ColorChecker Data') '.mat']);
            ColorData = struct('history',app.ColorHist, 'saved',app.entryTitle(tsec,'ColorChecker Data'), ...
                'cam',app.camSnapshot()); %#ok<NASGU>
            try, save(fn,'ColorData','-v7.3'); app.setColorStatus(sprintf('Saved %d entries -> %s', numel(app.ColorHist), fn),[0.6 0.9 0.6]);
            catch e, app.setColorStatus(e.message,[1 .4 .3]); end
        end
        function doColorToWorkspace(app)
            if isempty(app.ColorHist), app.setColorStatus('No ColorChecker history.',[1 .6 .3]); return; end
            vn = matlab.lang.makeValidName(app.entryTitle(app.serverEpoch(),'ColorChecker Data'));
            assignin('base', vn, app.ColorHist); app.setColorStatus(['History -> workspace var: ' vn],[0.6 0.9 0.6]);
        end
        function doColorThis(app)
            if isempty(app.ColorLast), app.setColorStatus('Measure first.',[1 .6 .3]); return; end
            vn = matlab.lang.makeValidName(app.entryTitle(app.serverEpoch(),'ColorChecker Data'));
            assignin('base', vn, app.ColorLast); app.setColorStatus(['Last result -> workspace var: ' vn],[0.6 0.9 0.6]);
        end
    end

    methods (Access = private, Static)
        function L = labFromLinSRGB(lin)
            im = reshape(min(max(lin,0),1), [], 1, 3);
            L = reshape(rgb2lab(lin2rgb(im)), [], 3);       % linear sRGB -> gamma -> CIELAB (D65)
        end
        function s = linToSRGB(lin)
            s = reshape(lin2rgb(reshape(min(max(lin,0),1),[],1,3)), [], 3);
        end
        function [ccm, resid] = lsqCCM(meas, ref)
            % least-squares 3x3 mapping raw-linear meas -> linear-sRGB ref, with a
            % scalar conditioning. ccm applied as rgb*ccm' (server: rgb @ ccm.T).
            scale = mean(ref(:)) / max(mean(meas(:)), 1e-9);
            X = meas * scale; A = X \ ref;                 % 24x3 \ 24x3 -> 3x3, X*A ~= ref
            resid = sqrt(mean((X*A - ref).^2, 'all'));     % RMS in linear-sRGB [0,1]
            ccm = (scale * A)';
        end
        function M2 = flipGrid(M, mode)
            % reorder 24 patches (row-major 4x6) under a shape-preserving symmetry
            G = reshape(1:24, [6 4])';                     % 4x6 row-major indices
            switch mode
                case 'lr',     Gp = fliplr(G);
                case 'ud',     Gp = flipud(G);
                case 'rot180', Gp = rot90(G, 2);
                otherwise,     Gp = G;
            end
            M2 = M(reshape(Gp', [], 1), :);
        end
        function c = deColor(dE)                 % dE<1 green, 1-3 yellow, >3 red
            if dE < 1,      c = [0.30 0.80 0.30];
            elseif dE < 3,  c = [0.90 0.80 0.15];
            else,           c = [0.92 0.30 0.22]; end
        end
        function cct = mccamyCCT(x, y)
            n = (x - 0.3320) / (0.1858 - y);
            cct = 449*n.^3 + 3525*n.^2 + 6823.3*n + 5520.33;
        end
    end

    % ═══════════════════════════════ DISPLAY ════════════════════════════════
    methods (Access = private)
        function showRaw(app, frame)
            % Fast raw preview: bin the Bayer quad -> half-res. Display only:
            % gray-world WB (colour) + optional stretch. The DATA paths (capture/
            % HDR) are untouched; CCM applies only to processed captures, not here.
            app.LastRaw = frame; app.PreviewFullRes = false;
            app.clearOverlay();                             % a new frame invalidates old squares
            f = single(frame);
            R = f(1:2:end,1:2:end);
            G = (f(1:2:end,2:2:end) + f(2:2:end,1:2:end)) / 2;
            B = f(2:2:end,2:2:end);
            fs = 2^app.cam.bit_depth - 1;
            luma = 0.299*R + 0.587*G + 0.114*B;             % for stats + MTF (unstretched)
            app.LastLuma = luma;
            if ~isfield(app.h,'color') || app.h.color.Value % gray-world WB (display only)
                mr=mean(R(:)); mg=mean(G(:)); mb=mean(B(:)); mgr=mean([mr mg mb])+eps;
                img = cat(3, R*(mgr/(mr+eps)), G*(mgr/(mg+eps)), B*(mgr/(mb+eps))) / fs;
            else
                img = luma / fs;
            end
            img = app.applyStretch(img);
            if size(img,3)==1, colormap(app.h.ax,gray(256)); end
            app.setDisplay(img);
            title(app.h.ax, sprintf('raw preview %dx%d (%d-bit)', ...
                size(frame,2), size(frame,1), app.cam.bit_depth));
            app.updateHistogram(luma, fs);
        end
        function showRGB(app, rgb)
            app.LastRGB = rgb; app.PreviewFullRes = true;
            app.clearOverlay();
            img = app.applyStretch(single(rgb)/255);
            app.setDisplay(img);
            title(app.h.ax, sprintf('processed %dx%d  |  Display CCM: %s', ...
                size(rgb,2), size(rgb,1), app.ccmStateStr()));
            app.updateCCMBadge();
            luma = single(rgb(:,:,1))*0.299+single(rgb(:,:,2))*0.587+single(rgb(:,:,3))*0.114;
            app.LastLuma = luma; app.updateHistogram(luma, 255);
        end
        function setDisplay(app, img)
            % Update the image + refit the axes to the (possibly new) size — the
            % SquareDrawerGUI technique. Fixes "does not fit on mode switch".
            set(app.h.img,'CData',img,'XData',[1 size(img,2)],'YData',[1 size(img,1)]);
            if size(img,3)==1, app.h.ax.CLim=[0 1]; end
            app.h.ax.XLim = [0.5, size(img,2)+0.5];
            app.h.ax.YLim = [0.5, size(img,1)+0.5];
            axis(app.h.ax,'image');
        end
        function clearOverlay(app)
            try, delete(app.OverlayH(isgraphics(app.OverlayH))); catch, end
            app.OverlayH = gobjects(0);
            app.clearMTFText();          % remove per-edge MTF50 labels too
        end
        function img = applyStretch(app, img)
            % Scale to the full display range like imagesc. Default: true data
            % min/max; Auto-stretch on: robust 1-99.5 pct (clips outliers/hot px).
            if isfield(app.h,'stretch') && app.h.stretch.Value
                s = sort(img(:)); n = numel(s);
                lo = s(max(round(0.01*n),1)); hi = s(max(round(0.995*n),1));
            else
                lo = min(img(:)); hi = max(img(:));
            end
            img = (img - lo) / max(hi - lo, 1e-6);
            img = min(max(img,0),1);
        end
        function refreshDisplay(app)
            if app.PreviewFullRes && ~isempty(app.LastRGB), app.showRGB(app.LastRGB);
            elseif ~isempty(app.LastRaw), app.showRaw(app.LastRaw); end
        end
        function updateHistogram(app, luma, fs)
            edges = linspace(0, fs, 128);
            hc = histcounts(luma(:), edges);
            centers = (edges(1:end-1)+edges(2:end))/2;
            cla(app.h.hax);
            bar(app.h.hax, centers, hc, 1, 'FaceColor',[0.4 0.6 0.9],'EdgeColor','none');
            app.h.hax.YScale='log';
            xlim(app.h.hax,[0 fs]); title(app.h.hax,'histogram');
            satfrac = mean(luma(:) >= 0.98*fs)*100;
            app.h.stats.Text = sprintf(['min  %.0f\nmax  %.0f\nmean %.1f\nsat  %.2f%%\n' ...
                'depth %d-bit'], min(luma(:)), max(luma(:)), mean(luma(:)), satfrac, ...
                app.cam.bit_depth);
        end
        function doSaveImage(app)
            if isempty(app.LastRGB) && isempty(app.LastLuma)
                uialert(app.Fig,'No image to save.','Save'); return; end
            [f,p] = uiputfile({'*.png';'*.tif'},'Save current image');
            if isequal(f,0), return; end
            if ~isempty(app.LastRGB), imwrite(app.LastRGB, fullfile(p,f));
            else, imwrite(uint8(255*app.LastLuma/max(app.LastLuma(:))), fullfile(p,f)); end
        end
    end

    % ═══════════════════════════════ CALIBRATE ══════════════════════════════
    methods (Access = private)
        function doMeasureDark(app)
            app.withBusy('measuring dark...', @() app.measureDarkRun());
        end
        function measureDarkRun(app)
            st = app.cam.measureDark(round(app.h.darkN.Value), app.h.darkGain.Value);
            app.setCal(sprintf(['dark @gain %.2f, %d-bit: black %.1f DN (vendor %.1f, %+.1f)\n' ...
                'DSNU R/Gr/Gb/B = %.2f/%.2f/%.2f/%.2f DN'], st.gain, st.bit_depth, ...
                st.phases.global, st.vendor_optical_black, st.delta_vs_vendor, ...
                st.dsnu.R, st.dsnu.Gr, st.dsnu.Gb, st.dsnu.B));
        end
        function doBuildHotMask(app)
            app.withBusy('building hot-pixel mask...', @() app.buildHotRun());
        end
        function buildHotRun(app)
            st = app.cam.buildHotMask(app.h.darkGain.Value, 16);
            app.setCal(sprintf('hot-pixel mask: %d px (%.4f%%) mode %d, n_sigma %.0f — persisted', ...
                st.count, st.frac_pct, st.mode, st.n_sigma));
        end
        function doMetrics(app)
            app.withBusy('sensor metrics...', @() app.metricsRun());
        end
        function metricsRun(app)
            m = app.cam.sensorMetrics(round(app.h.metN.Value));
            g = m.phases.global;
            app.setCal(sprintf(['metrics (%d-bit, gain %.2f):\n  read noise %.3f DN\n' ...
                '  black %.1f DN\n  DR %.2f stops (%.1f dB)'], m.bit_depth, m.gain, ...
                g.read_noise_dn, g.black_dn, g.dynamic_range_stops, g.dynamic_range_db));
        end
    end

    % ═══════════════════════════════ TOOLS ══════════════════════════════════
    methods (Access = private)
        function toolColorChecker(app)
            app.stopLive();
            if isempty(app.LastRGB)
                uialert(app.Fig,'Capture a processed image first (Capture > Processed image).','ColorChecker');
                return
            end
            app.setTool('Draw the chart outline: click TL, TR, BR, BL, then double-click.');
            try
                roi = drawpolygon(app.h.ax,'Color','y');
            catch
                uialert(app.Fig,'Corner selection unavailable on this axes.','ColorChecker'); return
            end
            pos = roi.Position;                 % [x y] rows in click order
            if size(pos,1) < 4, uialert(app.Fig,'Need 4 corners.','ColorChecker'); return; end
            corners = [pos(1:4,2), pos(1:4,1)]; % -> [row col] TL,TR,BR,BL
            app.withBusy('solving CCM...', @() app.ccmRun(corners));
        end
        function ccmRun(app, corners)
            [ccm, info] = app.cam.deriveCCM(corners, 'gain', app.h.gain.Value);
            app.setTool(sprintf('CCM residual %.4f (linear sRGB), %d patches.\nCCM =\n%s', ...
                info.residual, getfielddef(info,'n_patches',24), mat2str(ccm,4)));
            if isfield(app.h,'ccApply') && app.h.ccApply.Value
                app.cam.CCM = ccm;
                if isfield(app.h,'ccmGroup')
                    rb = findall(app.h.ccmGroup,'Type','uiradiobutton','Text','derived');
                    if ~isempty(rb), app.h.ccmGroup.SelectedObject = rb(1); end
                end
            end
        end

        function toolFindSquares(app)
            app.stopLive();
            if ~app.mserAvailable()
                uialert(app.Fig,['MSER not available — needs the Computer Vision Toolbox ' ...
                    '(detectMSERFeatures) or mexopencv cv.MSER.'],'Squares'); return
            end
            L = app.currentLuma();
            if isempty(L), uialert(app.Fig,'No image. Capture or grab first.','Squares'); return; end
            app.withBusy('finding squares (MSER)...', @() app.findSquaresRun(L));
        end
        function tf = mserAvailable(app)
            tf = app.OpenCVMSER || exist('detectMSERFeatures','file')==2;
        end
        function sq = findSquaresOn(app, L)
            % MSER square/rectangle detection on a 2D intensity image -> merged
            % struct array. Pure (no UI) so both the button and the live loop use it.
            % Params come from the sliders (delta, max variation, + OpenCV min
            % diversity); the tight Feret box is the rotating-calipers min-area
            % rect (same algorithm as MSERRegionAnalyzer/FeretMserDetector).
            sq = [];
            if ~app.mserAvailable(), return; end
            npx  = numel(L);
            minA = max(round(app.h.sqMinPct.Value/100 * npx), 30);
            maxA = max(round(app.h.sqMaxPct.Value/100 * npx), minA+1);
            I8   = uint8(mat2gray(L) * 255);
            try, I8 = adapthisteq(I8); catch, end              % CLAHE aids MSER (per mtfgui)
            delta  = max(round(app.h.sqDelta.Value), 1);
            maxVar = app.h.sqMaxVar.Value;
            minDiv = 0.2; if isfield(app.h,'sqMinDiv'), minDiv = app.h.sqMinDiv.Value; end
            pls    = app.mserPixelLists(I8, minA, maxA, delta, maxVar, minDiv);
            minAR  = app.h.sqMinAR.Value;
            res = struct('Vertices',{},'Centroid',{},'FeretWidth',{},'FeretHeight',{},'RotationAngle',{});
            for i = 1:numel(pls)
                pts = pls{i}; if size(pts,1) < 3, continue; end
                try
                    k = convhull(pts(:,1), pts(:,2), 'Simplify', true); hp = pts(k(1:end-1),:);
                catch, continue; end
                [V, dims] = app.minAreaRect(hp);
                mn = min(dims); mx = max(dims);
                if mn/max(mx,eps) < minAR, continue; end
                res(end+1) = struct('Vertices',V, 'Centroid',mean(pts,1), ...
                    'FeretWidth',mn, 'FeretHeight',mx, ...
                    'RotationAngle',atan2d(V(2,2)-V(1,2), V(2,1)-V(1,1))); %#ok<AGROW>
            end
            sq = app.mergeSquares(res, app.h.sqMerge.Value);
        end
        function pls = mserPixelLists(app, I8, minA, maxA, delta, maxVar, minDiv)
            % Region pixel lists ([x y], 1-based). OpenCV cv.MSER when present
            % (supports Min diversity), else MATLAB detectMSERFeatures.
            pls = {};
            if app.OpenCVMSER
                try
                    m = cv.MSER('Delta',delta, 'MinArea',minA, 'MaxArea',maxA, ...
                                'MaxVariation',maxVar, 'MinDiversity',minDiv);
                    regs = m.detectRegions(I8);            % cell of Nx2 [x y], 0-based
                    pls  = cellfun(@(r) double(r)+1, regs, 'UniformOutput', false);
                    return
                catch
                end
            end
            [regs, ~] = detectMSERFeatures(I8, 'RegionAreaRange',[minA maxA], ...
                'ThresholdDelta',delta, 'MaxAreaVariation',maxVar);
            pls = cell(1, regs.Count);
            for i = 1:regs.Count, pls{i} = double(regs.PixelList{i}); end
        end
        function [corners, dims] = minAreaRect(~, pts)
            % Minimum-area (Feret) bounding rectangle via rotating calipers over
            % convex-hull edges. corners = 4x2 [x y]; dims = [side1 side2].
            N = size(pts,1);
            minArea = inf; bR = eye(2); bmn = [0 0]; bmx = [0 0]; dims = [0 0];
            for i = 1:N
                e = pts(mod(i,N)+1,:) - pts(i,:); elen = norm(e); if elen < eps, continue; end
                u = e/elen; R = [u; -u(2) u(1)]; pr = (R*pts')';
                mn = min(pr); mx = max(pr); wid = mx(1)-mn(1); hgt = mx(2)-mn(2);
                if wid*hgt < minArea, minArea = wid*hgt; bR = R; bmn = mn; bmx = mx; dims = [wid hgt]; end
            end
            rr = [bmn(1) bmn(2); bmx(1) bmn(2); bmx(1) bmx(2); bmn(1) bmx(2)];
            corners = (bR' * rr')';
        end
        function findSquaresRun(app, L)
            app.Squares = app.findSquaresOn(L);
            app.drawSquares();
            sq = app.Squares;
            if isempty(sq)
                app.setTool('No squares found. Widen Min/Max area %, lower Min aspect, or fix exposure.');
            else
                ln = arrayfun(@(k) sprintf('  #%d  %.0fx%.0f px  AR %.2f  ang %.1f', ...
                    k, sq(k).feret(2), sq(k).feret(1), sq(k).ar, sq(k).angle), ...
                    1:numel(sq), 'uni', 0);
                app.setTool(sprintf(['%d square(s) [merge=%s]:\n%s\n' ...
                    'Then "MTF: click an edge" and click near a green edge.'], ...
                    numel(sq), app.h.sqMerge.Value, strjoin(ln(1:min(numel(ln),12)), newline)));
            end
        end
        function sq = mergeSquares(app, res, mode)
            sq = struct('verts',{},'centroid',{},'feret',{},'ar',{},'angle',{});
            if isempty(res), return; end
            n = numel(res); bb = zeros(n,4);
            for i=1:n
                v = res(i).Vertices; bb(i,:) = [min(v(:,1)) min(v(:,2)) max(v(:,1)) max(v(:,2))];
            end
            used = false(1,n);
            for i=1:n
                if used(i), continue; end
                g = i; used(i) = true; changed = true;
                while changed                              % greedily absorb overlaps (IoU>0.3)
                    changed = false;
                    for j = find(~used)
                        if any(arrayfun(@(k) app.iou(bb(k,:),bb(j,:)) > 0.3, g))
                            g(end+1) = j; used(j) = true; changed = true; %#ok<AGROW>
                        end
                    end
                end
                areas = arrayfun(@(k) res(k).FeretWidth*res(k).FeretHeight, g);
                switch mode
                    case 'largest',  [~,ix] = max(areas); k = g(ix); V = app.orderVerts(res(k).Vertices);
                                     FW = res(k).FeretWidth; FH = res(k).FeretHeight; ANG = res(k).RotationAngle;
                    case 'smallest', [~,ix] = min(areas); k = g(ix); V = app.orderVerts(res(k).Vertices);
                                     FW = res(k).FeretWidth; FH = res(k).FeretHeight; ANG = res(k).RotationAngle;
                    otherwise                              % average the ordered vertices
                        Vs = zeros(4,2,numel(g));
                        for m=1:numel(g), Vs(:,:,m) = app.orderVerts(res(g(m)).Vertices); end
                        V = mean(Vs,3);
                        FW = mean(arrayfun(@(k)res(k).FeretWidth,g));
                        FH = mean(arrayfun(@(k)res(k).FeretHeight,g));
                        ANG = mean(arrayfun(@(k)res(k).RotationAngle,g));
                end
                sq(end+1) = struct('verts',V,'centroid',mean(V,1), ...
                    'feret',[min(FW,FH) max(FW,FH)], 'ar',min(FW,FH)/max(max(FW,FH),eps), ...
                    'angle',ANG); %#ok<AGROW>
            end
        end
        function drawSquares(app)
            app.clearOverlay();
            sq = app.Squares; app.ROIBoxH = gobjects(0);
            if isempty(sq), return; end
            al = app.h.sqAlong.Value; ac = app.h.sqAcross.Value; showroi = app.h.sqShowROI.Value;
            bx=[]; by=[]; cx=zeros(numel(sq),1); cy=zeros(numel(sq),1);
            for i=1:numel(sq)
                V = sq(i).verts; P = [V; V(1,:)];
                bx=[bx;P(:,1);NaN]; by=[by;P(:,2);NaN]; %#ok<AGROW>
                cx(i)=sq(i).centroid(1); cy(i)=sq(i).centroid(2);
            end
            hold(app.h.ax,'on');
            hb = plot(app.h.ax, bx, by, '-', 'Color',[0 1 0], 'LineWidth',1.2);
            hc = plot(app.h.ax, cx, cy, 'o', 'Color',[0 1 0], 'MarkerSize',6, 'LineWidth',1);
            app.OverlayH = [hb; hc];
            if showroi                              % per-edge ROI boxes (individually hoverable)
                app.ROIBoxH = gobjects(4*numel(sq),1); idx=0;
                for i=1:numel(sq)
                    V=sq(i).verts;
                    for e=1:4
                        idx=idx+1; R=app.edgeRoiCorners(V,e,al,ac); Rc=[R;R(1,:)];
                        app.ROIBoxH(idx)=plot(app.h.ax, Rc(:,1), Rc(:,2), '-', 'Color',[1 1 0], 'LineWidth',1);
                    end
                end
                app.OverlayH = [app.OverlayH; app.ROIBoxH(isgraphics(app.ROIBoxH))];
            end
            hold(app.h.ax,'off');
        end
        function R = edgeRoiCorners(~, V, e, along, across)
            % ROI rectangle centred on edge e (V(e)->V(e+1)): along-edge extent =
            % along*L, across-edge extent = across*L, where L is the edge length.
            A = V(e,:); B = V(mod(e,4)+1,:); d = B - A; L = hypot(d(1),d(2));
            if L < eps, R = repmat(A,4,1); return; end
            u = d / L; nrm = [-u(2) u(1)]; M = (A + B)/2;
            ha = along*L/2; hc = across*L/2;
            R = [M-ha*u-hc*nrm; M+ha*u-hc*nrm; M+ha*u+hc*nrm; M-ha*u+hc*nrm];
        end
        function toolEdgeMTFpick(app)
            app.stopLive();
            if isempty(app.Squares), uialert(app.Fig,'Find squares first.','MTF'); return; end
            [src, fullres] = app.mtfSource(); if isempty(src), return; end
            app.setTool('Click near the square edge to measure.');
            try, pt = drawpoint(app.h.ax); catch, return; end
            xy = pt.Position; delete(pt);
            best = inf; bV = []; be = 0;
            for i=1:numel(app.Squares)
                V = app.Squares(i).verts;
                for e=1:4
                    dm = app.pt2seg(xy, V(e,:), V(mod(e,4)+1,:));
                    if dm < best, best = dm; bV = V; be = e; end
                end
            end
            if isempty(bV), return; end
            R   = app.edgeRoiCorners(bV, be, app.h.sqAlong.Value, app.h.sqAcross.Value);
            roi = app.extractRoi(src, R);
            if numel(roi) < 64, uialert(app.Fig,'Edge ROI too small — enlarge along/across or the square.','MTF'); return; end
            [osf, pixel, units] = app.mtfScale(fullres);
            app.withBusy('slanted-edge MTF...', @() app.runJSlantEdge(roi, osf, pixel, units, fullres));
        end
        function roi = extractRoi(~, src, R)
            x0=max(floor(min(R(:,1))),1); x1=min(ceil(max(R(:,1))),size(src,2));
            y0=max(floor(min(R(:,2))),1); y1=min(ceil(max(R(:,2))),size(src,1));
            roi = src(y0:y1, x0:x1);
            adir = R(2,:) - R(1,:);              % along-edge direction
            if abs(adir(1)) > abs(adir(2)), roi = roi'; end   % horizontal edge -> make it vertical
        end
        function [src, fullres] = mtfSource(app)
            if app.PreviewFullRes && ~isempty(app.LastRGB)
                src = double(rgb2gray(app.LastRGB)); fullres = true;
            elseif ~isempty(app.LastLuma)
                src = double(app.LastLuma); fullres = false;
            else
                src = []; fullres = false; uialert(app.Fig,'No image.','MTF');
            end
        end
        function [osf, pixel, units] = mtfScale(app, fullres)
            % pixel = jslantedge 3rd arg (sample spacing). deg: atand(pitch_mm/EFL_mm)
            % per the user's convention (e.g. atand(.002/7.6465)); mm: sensor pitch.
            osf   = round(app.h.mtfOSF.Value);
            pitch = app.h.mtfPitch.Value;              % um
            if ~fullres, pitch = pitch*2; end          % half-res preview -> pitch doubles
            if strcmp(app.mtfUnitStr(),'deg')
                pixel = atand((pitch/1000) / max(app.h.mtfEFL.Value, eps));  % deg/pixel
                units = 'cyc/deg';
            else
                pixel = pitch/1000;                    % mm/pixel  -> cyc/mm (lp/mm)
                units = 'cyc/mm';
            end
        end
        function u = mtfUnitStr(app)
            u = 'mm';
            try, u = app.h.mtfUnits.SelectedObject.Text; catch, end
        end

        function toolEdgeMTF(app)
            app.stopLive();
            % Sample the UNSTRETCHED source at its true resolution (display coords
            % match: processed = full-res LastRGB; raw preview = half-res LastLuma).
            if app.PreviewFullRes && ~isempty(app.LastRGB)
                src = double(rgb2gray(app.LastRGB)); fullres = true;
            elseif ~isempty(app.LastLuma)
                src = double(app.LastLuma); fullres = false;
            else
                uialert(app.Fig,'No image. Capture a (full-res) processed image first.','MTF'); return
            end
            app.setTool('Draw a box across ONE near-vertical slanted edge, then double-click.');
            try, r = drawrectangle(app.h.ax,'Color','g'); catch, return; end
            p = round(r.Position);                    % [x y w h] in displayed-image pixels
            W = size(src,2); H = size(src,1);
            cols = max(p(1),1):min(p(1)+p(3), W);
            rows = max(p(2),1):min(p(2)+p(4), H);
            if numel(cols) < 8 || numel(rows) < 8
                uialert(app.Fig,'ROI too small — draw a larger box across the edge.','MTF'); return
            end
            roi = src(rows, cols);
            [osf, pixel, units] = app.mtfScale(fullres);

            if exist('jslantedge','file') ~= 2      % fallback to the built-in proxy
                [rise, m50] = app.slantedEdge(roi);
                app.setTool(sprintf(['jslantedge not on path (%s).\nProxy: 10-90%% rise %.2f px, ' ...
                    'MTF50 ~ %.3f cyc/px'], app.JCODE_PATH, rise, m50)); return
            end
            app.withBusy('computing slanted-edge MTF...', ...
                @() app.runJSlantEdge(roi, osf, pixel, units, fullres));
        end

        function runJSlantEdge(app, roi, osf, pixel, units, fullres)
            [freq, mtf, esf, lsf, ~, out] = jslantedge(roi, osf, pixel);
            nyq   = 1/(2*pixel);                         % Nyquist in the chosen units
            mtf50 = app.mtf50(freq, mtf);
            try, mtfNyq = interp1(freq, mtf, nyq); catch, mtfNyq = NaN; end
            app.plotMTF(freq, mtf, esf, lsf, units, mtf50, nyq);
            note = '';
            if ~fullres, note = [newline '(half-res preview: pitch doubled; use a processed image for native-res MTF)']; end
            app.setTool(sprintf(['slanted-edge MTF (jslantedge, osf=%d)' newline '  ROI %dx%d px' newline ...
                '  edge slope=%.3f  intercept=%.1f' newline '  MTF50 = %.4g %s' newline ...
                '  MTF@Nyquist(%.4g) = %.3f'], osf, size(roi,2), size(roi,1), out(1), out(2), ...
                mtf50, units, nyq, mtfNyq));
        end

        function plotMTF(app, freq, mtf, ~, ~, units, mtf50, nyq)
            % single-shot MTF -> the embedded Preview MTF axes (not a popup figure)
            ax = app.MTFAx; if isempty(ax) || ~isvalid(ax), return; end
            LC = [.88 .88 .88];
            cla(ax); hold(ax,'on'); grid(ax,'on');
            plot(ax, freq, mtf, 'LineWidth', 1.5, 'Color',[0.3 0.7 1]);
            if ~isnan(mtf50), xline(ax, mtf50, 'r--'); end
            xline(ax, nyq, 'm--'); yline(ax, 0.5, 'Color',[.6 .6 .6],'LineStyle',':');
            xlabel(ax, units,'Color',LC); ylabel(ax, 'MTF','Color',LC);
            xlim(ax, [0 max(nyq,eps)]); ylim(ax, [0 1.05]);
            title(ax, sprintf('MTF50 = %.3g %s  (Nyq %.3g)', mtf50, units, nyq),'Color','w');
            hold(ax,'off');
            app.MTFLines = gobjects(0);      % axes reused -> invalidate live-line cache
        end
    end

    % ═══════════════════════════════ TOOL MATH ══════════════════════════════
    methods (Access = private, Static)
        function v = iou(a, b)               % axis-aligned box IoU; a,b=[x1 y1 x2 y2]
            ix = max(0, min(a(3),b(3)) - max(a(1),b(1)));
            iy = max(0, min(a(4),b(4)) - max(a(2),b(2)));
            inter = ix*iy;
            uni = (a(3)-a(1))*(a(4)-a(2)) + (b(3)-b(1))*(b(4)-b(2)) - inter;
            v = inter / max(uni, eps);
        end
        function V = orderVerts(V)            % order 4 corners CCW about the centroid
            c = mean(V,1); [~,o] = sort(atan2(V(:,2)-c(2), V(:,1)-c(1))); V = V(o,:);
        end
        function d = pt2seg(p, a, b)          % distance from point p to segment a-b
            ab = b - a; t = max(0, min(1, dot(p-a,ab)/max(dot(ab,ab),eps)));
            q = a + t*ab; d = hypot(p(1)-q(1), p(2)-q(2));
        end
        function m = mtf50(freq, mtf)          % first downward 0.5 crossing of the MTF
            i = find(mtf(:) <= 0.5, 1, 'first');
            if isempty(i) || i < 2, m = NaN;
            else, m = interp1(mtf(i-1:i), freq(i-1:i), 0.5); end
        end
        function [rise10_90, mtf50] = slantedEdge(roi)
            % quick sharpness proxy: average edge-spread across the shorter axis,
            % differentiate for the line-spread, report 10-90% rise (px) and a
            % rough MTF50. Not a certified sfrmat, but a fast in-GUI indicator.
            if size(roi,1) > size(roi,2), roi = roi'; end   % edge ~ vertical
            esf = mean(roi,1);
            esf = (esf - min(esf)) / max(esf-min(esf)+eps);
            % 10-90 rise
            x = 1:numel(esf);
            try
                x10 = interp1(esf, x, 0.1, 'linear'); x90 = interp1(esf, x, 0.9, 'linear');
                rise10_90 = abs(x90 - x10);
            catch, rise10_90 = NaN; end
            lsf = diff(esf); lsf = lsf/ (sum(abs(lsf))+eps);
            M = abs(fft(lsf, 256)); M = M/ (M(1)+eps);
            f = (0:255)/256;
            idx = find(M(1:128) <= 0.5, 1, 'first');
            if isempty(idx), mtf50 = NaN; else, mtf50 = f(idx); end
        end
    end

    % ═══════════════════════════════ HELPERS ════════════════════════════════
    methods (Access = private)
        function L = currentLuma(app)
            if ~isempty(app.LastLuma), L = app.LastLuma;
            elseif ~isempty(app.LastRGB), L = single(rgb2gray(app.LastRGB));
            else, L = []; end
        end
        function redrawOverlay(app)
            if ~isempty(app.LastRGB), app.showRGB(app.LastRGB);
            elseif ~isempty(app.LastRaw), app.showRaw(app.LastRaw); end
        end
        function withBusy(app, msg, fn)
            app.Busy = true;                    % gate the live timer off the shared socket
            d = []; try, d = uiprogressdlg(app.Fig,'Message',msg,'Indeterminate','on'); catch, end
            c = onCleanup(@() app.endBusy(d));
            app.guard(fn);
        end
        function endBusy(app, d)
            app.Busy = false;
            try, if ~isempty(d)&&isvalid(d), close(d); end, catch, end
        end
        function guard(app, fn)
            try, fn(); catch e, uialert(app.Fig, e.message, 'Error'); end
        end
        function trySet(app, fn)
            try, fn(); catch e, uialert(app.Fig, e.message, 'Setting'); end
        end
        function setCal(app, s), app.h.calOut.Value = strsplit(s, newline); end
        function setTool(app, s), app.h.toolOut.Value = strsplit(s, newline); end
    end
end

% ── free helpers (file-local) ──────────────────────────────────────────────
function v = getfielddef(s, f, d)
    if isstruct(s) && isfield(s,f), v = s.(f); else, v = d; end
end
