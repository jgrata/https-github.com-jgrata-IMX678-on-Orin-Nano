classdef ECamHDRClient < handle
    %ECAMHDRCLIENT  True 10-bit RAW acquisition from e-CAM86_CUONX (IMX678).
    %
    %
    %  QUICK START:
    %    cam = ECamHDRClient('10.70.0.27');
    %    cam.connect();
    %    cam.exposure_ns = 33000000;      % 33 ms
    %    cam.bit_depth   = 10;
    %    frame = cam.capture();           % uint16 [H x W], 0-1023
    %    rgb   = cam.captureRGB();        % demosaiced colour
    %    cam.showParams();
    %    cam.disconnect();
    %
    %  PARAMETER ASSIGNMENT (equivalent):
    %    cam.exposure_ns = 33000000
    %    cam.setParams('exposure_ns', 33000000)
    %
    %  PARAMETERS:
    %    sensormode      1=plain RAW10,  3=software HDR
    %    exposure_ns     nanoseconds     (0 = auto AE)
    %    gain            multiplier      (0 = auto AGC)
    %    fps             frame rate
    %    bit_depth       8, 10, or 12
    %    hdr_exp_ratio   long/short exposure ratio for HDR
    %    sat_threshold   HDR blend threshold 0-1
    %
    %  READ-ONLY ACTUALS (populated after first capture):
    %    actual_exposure_ns   actual sensor exposure (ns)
    %    actual_gain_value    actual sensor gain multiplier
    %
    %  No Java dependencies — works with -nojvm MATLAB.

    % ── Public read-only ──────────────────────────────────────────────────────
    properties (SetAccess = private)
        Host         (1,:) char    = '10.70.0.27'
        Port         (1,1) double  = 9000
        IsConnected  (1,1) logical = false
        CameraInfo   (1,1) struct
        Debug        (1,1) logical = false
    end

    % ── Dependent settable parameters ─────────────────────────────────────────
    properties (Dependent)
        sensormode
        exposure_ns
        gain
        fps
        bit_depth
        hdr_exp_ratio
        sat_threshold
        lossless
        actual_exposure_ns   % read-only actual
        actual_gain_value    % read-only actual
    end

    % ── Internal cache ────────────────────────────────────────────────────────
    properties (Access = private)
        p_sensormode    (1,1) double = 1
        p_exposure_ns   (1,1) double = 33000000
        p_gain          (1,1) double = 0.0
        p_fps           (1,1) double = 30
        p_bit_depth     (1,1) double = 10
        p_hdr_exp_ratio (1,1) double = 4
        p_sat_threshold (1,1) double = 0.95
        p_lossless      (1,1) logical = true
    end

    % ── Private infrastructure ────────────────────────────────────────────────
    properties (Access = private)
        Socket
        StreamTimer
        StreamCallback
        StreamFrameCount (1,1) double = 0
    end

    % ── Constants ─────────────────────────────────────────────────────────────
    properties (Constant, Access = private)
        CMD_CAPTURE    = uint32(1)
        CMD_STREAM_ON  = uint32(2)
        CMD_STREAM_OFF = uint32(3)
        CMD_SET_PARAMS = uint32(4)
        CMD_GET_INFO   = uint32(5)
        CMD_PING       = uint32(6)

        RECV_TIMEOUT_S  = 300    % 5 min — accommodates pipeline init
        CLEANUP_TIMEOUT = 2
        POLL_INTERVAL   = 0.001  % 1ms
    end

    % ═════════════════════════════════════════════════════════════════════════
    methods   % Public

        % ── Constructor ───────────────────────────────────────────────────────
        function obj = ECamHDRClient(host, port, debug)
            if nargin >= 1 && ~isempty(host),  obj.Host  = host;  end
            if nargin >= 2 && ~isempty(port),  obj.Port  = port;  end
            if nargin >= 3 && ~isempty(debug), obj.Debug = debug; end
        end

        % ── connect ───────────────────────────────────────────────────────────
        function connect(obj)
            if obj.IsConnected
                warning('ECamHDRClient:alreadyConnected','Already connected.');
                return
            end
            fprintf('[ECam] Connecting to %s:%d ...\n', obj.Host, obj.Port);
            obj.Socket = tcpclient(obj.Host, obj.Port, ...
                'Timeout',   obj.RECV_TIMEOUT_S, ...
                'ByteOrder', 'little-endian');
            pause(0.15);
            if obj.Socket.NumBytesAvailable > 0
                read(obj.Socket, obj.Socket.NumBytesAvailable, 'uint8');
            end
            obj.IsConnected = true;
            fprintf('[ECam] Connected.\n');
            obj.ping();
            obj.CameraInfo = obj.getInfo();
            obj.syncFromServer();
            fprintf('[ECam] Sensor  : %s\n', obj.CameraInfo.sensor);
            fprintf('[ECam] Res     : %d x %d\n', ...
                obj.CameraInfo.width, obj.CameraInfo.height);
            fprintf('[ECam] Method  : %s\n', obj.CameraInfo.method);
            fprintf('[ECam] Note    : %s\n', obj.CameraInfo.note);
        end

        % ── disconnect ────────────────────────────────────────────────────────
        function disconnect(obj)
            if ~obj.IsConnected, return; end
            obj.killTimer();
            try
                obj.Socket.Timeout = obj.CLEANUP_TIMEOUT;
                obj.sendCmd(obj.CMD_STREAM_OFF);
                pause(0.05);
            catch
            end
            try; delete(obj.Socket); catch; end
            obj.Socket      = [];
            obj.IsConnected = false;
            fprintf('[ECam] Disconnected.\n');
        end

        % ── delete ────────────────────────────────────────────────────────────
        function delete(obj)
            obj.killTimer();
            try
                if ~isempty(obj.Socket) && isvalid(obj.Socket)
                    obj.Socket.Timeout = 1;
                    delete(obj.Socket);
                end
            catch
            end
            obj.IsConnected = false;
        end

        % ── setDebug ──────────────────────────────────────────────────────────
        function setDebug(obj, tf)
            %SETDEBUG  Toggle verbose per-frame timing/statistics at runtime.
            %  cam.setDebug(true) then cam.capture() prints the network
            %  throughput (MB/s), unpack time, and info round-trip time.
            obj.Debug = logical(tf);
        end

        % ── ping ──────────────────────────────────────────────────────────────
        function ping(obj)
            obj.requireConnected();
            obj.sendCmd(obj.CMD_PING);
            resp = obj.recvResp();
            if ~strcmpi(strtrim(char(resp(:)')), 'PONG')
                error('ECamHDRClient:pingFailed', ...
                    'Bad ping response: %s', char(resp(:)'));
            end
            fprintf('[ECam] Ping OK.\n');
        end

        % ── getInfo ───────────────────────────────────────────────────────────
        function info = getInfo(obj)
            obj.requireConnected();
            obj.sendCmd(obj.CMD_GET_INFO);
            data = obj.recvResp();
            info = jsondecode(char(data(:)'));
        end

        % ── showParams ────────────────────────────────────────────────────────
        function showParams(obj)
            obj.requireConnected();
            obj.CameraInfo = obj.getInfo();
            obj.syncFromServer();

            mx = 2^obj.p_bit_depth - 1;

            if obj.p_exposure_ns == 0
                exp_set = '0  (auto AE)';
            else
                exp_set = sprintf('%d ns  (%.2f ms)', ...
                    obj.p_exposure_ns, obj.p_exposure_ns/1e6);
            end

            has_ae = isfield(obj.CameraInfo,'actual_exposure_ns') && ...
                obj.CameraInfo.actual_exposure_ns > 0;
            if has_ae
                exp_act = sprintf('%d ns  (%.2f ms)', ...
                    obj.CameraInfo.actual_exposure_ns, ...
                    obj.CameraInfo.actual_exposure_ns/1e6);
            else
                exp_act = 'not yet measured';
            end

            if obj.p_gain == 0.0
                gain_set = '0  (auto AGC)';
            else
                gain_set = sprintf('%.4fx', obj.p_gain);
            end

            has_ag = isfield(obj.CameraInfo,'actual_gain') && ...
                obj.CameraInfo.actual_gain > 0;
            if has_ag
                gain_act = sprintf('%.4fx', obj.CameraInfo.actual_gain);
            else
                gain_act = 'not yet measured';
            end

            long_ns  = max(obj.p_exposure_ns, 1);
            short_ns = max(450000, long_ns / obj.p_hdr_exp_ratio);

            fprintf('\n');
            fprintf('======================================================\n');
            fprintf('  e-CAM86_CUONX  |  %s:%d\n', obj.Host, obj.Port);
            fprintf('======================================================\n');
            fprintf('  sensor         : %s\n', obj.CameraInfo.sensor);
            fprintf('  resolution     : %d x %d\n', ...
                obj.CameraInfo.width, obj.CameraInfo.height);
            fprintf('  method         : %s\n', obj.CameraInfo.method);
            fprintf('------------------------------------------------------\n');

            if obj.p_sensormode == 1
                sm_tag = '(plain RAW10)';
            elseif obj.p_sensormode == 3
                sm_tag = '(software HDR)';
            else
                sm_tag = '';
            end
            fprintf('  %-22s : %d  %s\n', 'sensormode', ...
                obj.p_sensormode, sm_tag);
            fprintf('\n');
            fprintf('  %-22s : %s\n', 'exposure_ns (set)',    exp_set);
            fprintf('  %-22s : %s\n', 'actual_exposure_ns',   exp_act);
            fprintf('    Range: 450000 to 400000000 ns\n\n');
            fprintf('  %-22s : %s\n', 'gain (set)',    gain_set);
            fprintf('  %-22s : %s\n', 'actual_gain',   gain_act);
            fprintf('    Range: 1.0x to 31.6x\n\n');
            fprintf('  %-22s : %d fps\n\n', 'fps', obj.p_fps);
            fprintf('  %-22s : %d-bit  (0-%d)\n', ...
                'bit_depth', obj.p_bit_depth, mx);
            fprintf('    Valid: 8, 10, 12\n\n');
            fprintf('  %-22s : %d\n', 'hdr_exp_ratio', obj.p_hdr_exp_ratio);
            if obj.p_exposure_ns > 0
                fprintf('    long=%.1fms  short=%.1fms\n', ...
                    long_ns/1e6, short_ns/1e6);
            end
            fprintf('\n');
            fprintf('  %-22s : %.2f\n', 'sat_threshold', obj.p_sat_threshold);
            fprintf('------------------------------------------------------\n');
            fprintf('  SYNTAX:\n');
            fprintf('    cam.exposure_ns = 33000000\n');
            fprintf('    cam.bit_depth   = 10\n');
            fprintf('    cam.sensormode  = 1\n');
            fprintf('    cam.gain        = 0   %% auto\n');
            fprintf('======================================================\n\n');
        end

        % ── setParams ─────────────────────────────────────────────────────────
        function setParams(obj, varargin)
            obj.requireConnected();
            p = inputParser();
            addParameter(p, 'sensormode',    [], @isnumeric);
            addParameter(p, 'fps',           [], @isnumeric);
            addParameter(p, 'exposure_ns',   [], @isnumeric);
            addParameter(p, 'gain',          [], @isnumeric);
            addParameter(p, 'bit_depth',     [], @isnumeric);
            addParameter(p, 'sat_threshold', [], @isnumeric);
            addParameter(p, 'hdr_exp_ratio', [], @isnumeric);
            addParameter(p, 'lossless',      [], @(x)islogical(x)||isnumeric(x));
            parse(p, varargin{:});

            params = struct();
            flds = {'sensormode','fps','exposure_ns','gain', ...
                    'bit_depth','sat_threshold','hdr_exp_ratio','lossless'};
            for k = 1:numel(flds)
                f = flds{k};
                if ~isempty(p.Results.(f))
                    if strcmp(f,'lossless')
                        params.(f) = logical(p.Results.(f));
                    else
                        params.(f) = p.Results.(f);
                    end
                end
            end
            if isempty(fieldnames(params))
                warning('ECamHDRClient:noParams','No parameters given.');
                return
            end

            obj.sendCmd(obj.CMD_SET_PARAMS, uint8(jsonencode(params)));
            resp = obj.recvResp();
            if ~strcmpi(strtrim(char(resp(:)')), 'OK')
                error('ECamHDRClient:setFailed', ...
                    'setParams failed: %s', char(resp(:)'));
            end
            obj.CameraInfo = obj.getInfo();
            obj.syncFromServer();

            fprintf('[ECam] Params updated:\n');
            for k = 1:numel(flds)
                f = flds{k};
                if ~isempty(p.Results.(f))
                    fprintf('  %-20s = %s\n', f, obj.fmtParam(f));
                end
            end
        end

        % ── capture ───────────────────────────────────────────────────────────
        function frame = capture(obj)
            %CAPTURE  Capture one RAW frame.
            %  Returns uint16 [H x W].
            %  Range: 0-255 (8-bit), 0-1023 (10-bit), 0-4095 (12-bit).
            %  Bayer pattern RGGB — use captureRGB() for colour.
            obj.requireConnected();

            if obj.p_exposure_ns == 0
                exp_s = 'auto';
            else
                exp_s = sprintf('%.0fms', obj.p_exposure_ns/1e6);
            end
            fprintf('[ECam] Capture  mode=%d  %d-bit  exp=%s\n', ...
                obj.p_sensormode, obj.p_bit_depth, exp_s);

            t_frame = tic;
            obj.sendCmd(obj.CMD_CAPTURE);
            frame = obj.recvFrame();
            t_frame = toc(t_frame);

            t_info = tic;
            obj.CameraInfo = obj.getInfo();
            obj.syncFromServer();
            t_info = toc(t_info);

            if obj.Debug
                % Full diagnostics (distinct/mean are O(n log n) / heavy
                % allocations over ~8.3M pixels — Debug-only, not per-frame).
                fprintf('[ECam] Frame    %dx%d  [%d,%d]  mean=%.1f  distinct=%d\n', ...
                    size(frame,1), size(frame,2), ...
                    min(frame(:)), max(frame(:)), ...
                    mean(double(frame(:))), numel(unique(frame(:))));
                fprintf('[ECam] Timing   frame=%.3fs  info=%.3fs\n', ...
                    t_frame, t_info);
            else
                fprintf('[ECam] Frame    %dx%d  (%.3fs)\n', ...
                    size(frame,1), size(frame,2), t_frame + t_info);
            end

            has_ae = isfield(obj.CameraInfo,'actual_exposure_ns') && ...
                obj.CameraInfo.actual_exposure_ns > 0;
            if has_ae
                fprintf('[ECam] Actual   exp=%.2fms  gain=%.4fx\n', ...
                    obj.CameraInfo.actual_exposure_ns/1e6, ...
                    obj.CameraInfo.actual_gain);
            end
        end

        % ── grab ──────────────────────────────────────────────────────────────
        function frame = grab(obj)
            %GRAB  Fast single-frame capture for high-throughput DAQ loops.
            %  Returns uint16 [H x W], identical pixels to capture().
            %  Unlike capture() it skips the per-frame getInfo() round-trip,
            %  the syncFromServer() JSON decode, and all statistics/printing —
            %  everything not needed to move pixels. Use this in tight
            %  acquisition loops; call capture() (or showParams()) when you
            %  need the refreshed actual exposure/gain.
            obj.requireConnected();
            obj.sendCmd(obj.CMD_CAPTURE);
            frame = obj.recvFrame();
        end

        % ── captureNormalized ─────────────────────────────────────────────────
        function img = captureNormalized(obj)
            img = double(obj.capture()) / double(2^obj.p_bit_depth - 1);
        end

        % ── captureRGB ────────────────────────────────────────────────────────
        function rgb = captureRGB(obj)
            %CAPTURERGB  Demosaic RAW Bayer RGGB → uint16 RGB [H x W x 3].
            frame   = obj.capture();
            scale   = double(2^16-1) / double(2^obj.p_bit_depth-1);
            frame16 = uint16(double(frame) * scale);
            rgb     = demosaic(frame16, 'rggb');
        end

        % ── startStream ───────────────────────────────────────────────────────
        function startStream(obj, callback, fps)
            obj.requireConnected();
            if nargin < 3 || isempty(fps), fps = obj.p_fps; end
            obj.StreamCallback   = callback;
            obj.StreamFrameCount = 0;
            obj.sendCmd(obj.CMD_STREAM_ON);
            resp = obj.recvResp();
            if ~strcmpi(strtrim(char(resp(:)')), 'STREAMING')
                error('ECamHDRClient:streamFail', ...
                    'Stream failed: %s', char(resp(:)'));
            end
            period = max(1/fps, 0.1);
            obj.StreamTimer = timer( ...
                'ExecutionMode', 'fixedRate', ...
                'Period',         period, ...
                'TimerFcn',       @(~,~) obj.streamPoll(), ...
                'ErrorFcn',       @(~,e) obj.streamError(e));
            start(obj.StreamTimer);
            fprintf('[ECam] Streaming at ~%.1f fps.\n', fps);
        end

        % ── stopStream ────────────────────────────────────────────────────────
        function stopStream(obj)
            obj.killTimer();
            if obj.IsConnected
                try
                    obj.Socket.Timeout = obj.CLEANUP_TIMEOUT;
                    obj.sendCmd(obj.CMD_STREAM_OFF);
                    pause(0.05);
                    obj.Socket.Timeout = obj.RECV_TIMEOUT_S;
                catch
                end
            end
            fprintf('[ECam] Stream stopped. Frames: %d\n', ...
                obj.StreamFrameCount);
        end

        % ── streamPull ──────────────────────────────────────────────────────────
        function stats = streamPull(obj, callback, nFrames, depth)
            %STREAMPULL  High-throughput pipelined acquisition.
            %  stats = cam.streamPull(cb, N)         keep 2 requests in flight
            %  stats = cam.streamPull(cb, N, depth)  keep `depth` in flight
            %
            %  Keeps `depth` capture requests in flight at once so the network
            %  transfer of one frame overlaps the sensor/CUDA capture of the
            %  next (and your callback processing the previous). This hides the
            %  request round-trip that limits a plain capture() loop to
            %  ~1/RTT fps, letting throughput approach the link bandwidth.
            %
            %  callback is invoked as cb(frame, k) for each frame k=1..N, where
            %  frame is uint16 [H x W] (identical to capture()/grab()). Runs as
            %  a blocking foreground loop — deterministic, best for recording.
            %  Returns struct(frames, elapsed_s, fps).
            %
            %  No getInfo() round-trip or per-frame stats — pure pixel path.
            %  Uses CMD_CAPTURE, so it needs no special server stream mode.
            obj.requireConnected();
            if nargin < 4 || isempty(depth), depth = 2; end
            depth = max(1, min(round(depth), nFrames));

            sent = 0; got = 0;
            for i = 1:depth                       % prime the pipeline
                obj.sendCmd(obj.CMD_CAPTURE); sent = sent + 1;
            end

            t0 = tic;
            while got < nFrames
                frame = obj.recvFrame();
                got   = got + 1;
                if sent < nFrames                 % refill to keep `depth` in flight
                    obj.sendCmd(obj.CMD_CAPTURE); sent = sent + 1;
                end
                if ~isempty(callback), callback(frame, got); end
            end
            elapsed = toc(t0);

            stats = struct('frames', got, 'elapsed_s', elapsed, ...
                           'fps', got / max(elapsed, eps));
            fprintf('[ECam] streamPull: %d frames in %.2fs = %.1f fps\n', ...
                got, elapsed, stats.fps);
        end

        % ── recordFrames ────────────────────────────────────────────────────────
        function [frames, stats] = recordFrames(obj, nFrames, depth)
            %RECORDFRAMES  Acquire N frames into a uint16 [H x W x N] array.
            %  [frames, stats] = cam.recordFrames(N)
            %  Convenience wrapper over streamPull for burst DAQ. Note memory:
            %  each 4K 10-bit frame is ~16.6MB in uint16, so N frames need
            %  ~16.6*N MB of RAM.
            obj.requireConnected();
            if nargin < 3 || isempty(depth), depth = 2; end
            H = double(obj.CameraInfo.height);
            W = double(obj.CameraInfo.width);
            frames = zeros(H, W, nFrames, 'uint16');
            stats  = obj.streamPull(@store, nFrames, depth);

            function store(f, k)
                frames(:,:,k) = f;
            end
        end

    end % public

    % ═════════════════════════════════════════════════════════════════════════
    % GET
    methods

        function v = get.sensormode(obj),    v = obj.p_sensormode;    end
        function v = get.fps(obj),           v = obj.p_fps;           end
        function v = get.bit_depth(obj),     v = obj.p_bit_depth;     end
        function v = get.hdr_exp_ratio(obj), v = obj.p_hdr_exp_ratio; end
        function v = get.sat_threshold(obj), v = obj.p_sat_threshold; end
        function v = get.lossless(obj),      v = obj.p_lossless;      end

        function v = get.exposure_ns(obj)
            if obj.p_exposure_ns == 0 && ...
                    ~isempty(fieldnames(obj.CameraInfo)) && ...
                    isfield(obj.CameraInfo,'actual_exposure_ns') && ...
                    obj.CameraInfo.actual_exposure_ns > 0
                v = obj.CameraInfo.actual_exposure_ns;
            else
                v = obj.p_exposure_ns;
            end
        end

        function v = get.gain(obj)
            if obj.p_gain == 0.0 && ...
                    ~isempty(fieldnames(obj.CameraInfo)) && ...
                    isfield(obj.CameraInfo,'actual_gain') && ...
                    obj.CameraInfo.actual_gain > 0
                v = obj.CameraInfo.actual_gain;
            else
                v = obj.p_gain;
            end
        end

        function v = get.actual_exposure_ns(obj)
            if ~isempty(fieldnames(obj.CameraInfo)) && ...
                    isfield(obj.CameraInfo,'actual_exposure_ns')
                v = obj.CameraInfo.actual_exposure_ns;
            else
                v = 0;
            end
        end

        function v = get.actual_gain_value(obj)
            if ~isempty(fieldnames(obj.CameraInfo)) && ...
                    isfield(obj.CameraInfo,'actual_gain')
                v = obj.CameraInfo.actual_gain;
            else
                v = 0.0;
            end
        end

    end % GET

    % ═════════════════════════════════════════════════════════════════════════
    % SET  (cam.param = value → sends to server)
    methods

        function set.sensormode(obj, v)
            obj.p_sensormode = v;
            if obj.IsConnected, obj.setParams('sensormode', v); end
        end
        function set.exposure_ns(obj, v)
            obj.p_exposure_ns = v;
            if obj.IsConnected, obj.setParams('exposure_ns', v); end
        end
        function set.gain(obj, v)
            obj.p_gain = v;
            if obj.IsConnected, obj.setParams('gain', v); end
        end
        function set.fps(obj, v)
            obj.p_fps = v;
            if obj.IsConnected, obj.setParams('fps', v); end
        end
        function set.bit_depth(obj, v)
            if ~ismember(v, [8 10 12])
                error('ECamHDRClient:badBpp','bit_depth must be 8,10,12');
            end
            obj.p_bit_depth = v;
            if obj.IsConnected, obj.setParams('bit_depth', v); end
        end
        function set.hdr_exp_ratio(obj, v)
            obj.p_hdr_exp_ratio = v;
            if obj.IsConnected, obj.setParams('hdr_exp_ratio', v); end
        end
        function set.sat_threshold(obj, v)
            obj.p_sat_threshold = v;
            if obj.IsConnected, obj.setParams('sat_threshold', v); end
        end
        function set.lossless(obj, v)
            obj.p_lossless = logical(v);
            if obj.IsConnected, obj.setParams('lossless', logical(v)); end
        end
        function set.actual_exposure_ns(~,~), end  % read-only
        function set.actual_gain_value(~,~),  end  % read-only

    end % SET

    % ═════════════════════════════════════════════════════════════════════════
    % Private
    methods (Access = private)

        function requireConnected(obj)
            if ~obj.IsConnected
                error('ECamHDRClient:notConnected', ...
                    'Not connected. Call connect() first.');
            end
        end

        function killTimer(obj)
            try
                if ~isempty(obj.StreamTimer) && isvalid(obj.StreamTimer)
                    stop(obj.StreamTimer);
                    delete(obj.StreamTimer);
                end
            catch
            end
            obj.StreamTimer = [];
        end

        function syncFromServer(obj)
            info = obj.CameraInfo;
            if isfield(info,'exposure_ns_set')
                obj.p_exposure_ns = info.exposure_ns_set;
            elseif isfield(info,'exposure_ns')
                obj.p_exposure_ns = info.exposure_ns;
            end
            if isfield(info,'gain_set')
                obj.p_gain = info.gain_set;
            elseif isfield(info,'gain')
                obj.p_gain = info.gain;
            end
            map = {'sensormode','p_sensormode'; ...
                   'fps',       'p_fps'; ...
                   'bit_depth', 'p_bit_depth'; ...
                   'hdr_exp_ratio','p_hdr_exp_ratio'; ...
                   'sat_threshold','p_sat_threshold'; ...
                   'lossless',  'p_lossless'};
            for k = 1:size(map,1)
                if isfield(info, map{k,1})
                    obj.(map{k,2}) = info.(map{k,1});
                end
            end
        end

        function s = fmtParam(obj, f)
            switch f
                case 'exposure_ns'
                    v = obj.p_exposure_ns;
                    if v==0, s='0 (auto)';
                    else, s=sprintf('%d ns (%.2fms)',v,v/1e6); end
                case 'gain'
                    v = obj.p_gain;
                    if v==0, s='0 (auto)';
                    else, s=sprintf('%.4fx',v); end
                case 'bit_depth'
                    v = obj.p_bit_depth;
                    s = sprintf('%d-bit (0-%d)',v,2^v-1);
                case 'sensormode'
                    if obj.p_sensormode==1, t='plain';
                    elseif obj.p_sensormode==3, t='HDR';
                    else, t=''; end
                    s = sprintf('%d (%s)',obj.p_sensormode,t);
                case 'fps'
                    s = sprintf('%d',obj.p_fps);
                case 'hdr_exp_ratio'
                    s = sprintf('%d',obj.p_hdr_exp_ratio);
                case 'sat_threshold'
                    s = sprintf('%.2f',obj.p_sat_threshold);
                case 'lossless'
                    if obj.p_lossless, s = 'true (RAW10 packed)';
                    else, s = 'false (lossy allowed)'; end
                otherwise, s = '?';
            end
        end

        % ── Wire protocol ─────────────────────────────────────────────────────

        function sendCmd(obj, cmd, payload)
            if nargin < 3 || isempty(payload), payload = uint8([]); end
            payload = uint8(payload(:)');
            hdr     = typecast([uint32(cmd), uint32(numel(payload))], 'uint8');
            write(obj.Socket, [hdr, payload], 'uint8');
        end

        function data = recvResp(obj)
            hdr    = obj.rdBytes(5);
            status = hdr(1);
            len    = double(typecast(uint8(hdr(2:5)), 'uint32'));
            data   = uint8([]);
            if len > 0, data = obj.rdBytes(len); end
            if status == 0xFF
                error('ECamHDRClient:serverError', ...
                    'Server: %s', char(data(:)'));
            end
            if status ~= 0x00
                error('ECamHDRClient:badStatus', ...
                    'Status 0x%02X', status);
            end
        end

        function frame = recvFrame(obj)
            % Read status byte
            sb = obj.rdBytes(1);

            if sb(1) == 0xFF
                lb  = obj.rdBytes(4);
                len = double(typecast(uint8(lb),'uint32'));
                msg = uint8([]);
                if len > 0, msg = obj.rdBytes(len); end
                error('ECamHDRClient:captureError', ...
                    'Capture error:\n%s', char(msg(:)'));
            end
            if sb(1) ~= 0x00
                error('ECamHDRClient:badStatus', ...
                    'Frame status 0x%02X', sb(1));
            end

            % Frame header: H(4) W(4) dtype(1) = 9 bytes
            rest  = obj.rdBytes(9);
            H     = double(typecast(uint8(rest(1:4)), 'uint32'));
            W     = double(typecast(uint8(rest(5:8)), 'uint32'));
            dtype = rest(9);

            if H==0 || W==0
                error('ECamHDRClient:badDims','H=%d W=%d',H,W);
            end

            if dtype == 0x10
                % uint16 format: 2 bytes per pixel
                n_bytes = H * W * 2;
                t_rd    = tic;
                raw     = obj.rdBytes(n_bytes);
                t_rd    = toc(t_rd);
                t_up    = tic;
                pixels  = typecast(uint8(raw(:)'), 'uint16');
                frame   = reshape(pixels, [W, H])';
                t_up    = toc(t_up);
                obj.reportRecv(n_bytes, t_rd, t_up);

            elseif dtype == 0x11
                % RAW10 packed: server packs in groups of 4 pixels -> 5 bytes,
                % padding a partial final group. Must match pack_raw10 exactly:
                %   n_bytes = ceil(H*W/4) * 5   (NOT ceil(H*W*10/8), which
                %   differs when H*W is not a multiple of 4).
                n_groups = ceil(double(H) * double(W) / 4);
                n_packed = n_groups * 5;
                t_rd     = tic;
                raw      = obj.rdBytes(n_packed);
                t_rd     = toc(t_rd);
                t_up     = tic;
                frame    = obj.unpackRaw10(uint8(raw(:)'), H, W);
                t_up     = toc(t_up);
                obj.reportRecv(n_packed, t_rd, t_up);

            else
                error('ECamHDRClient:unknownDtype', ...
                    'Unknown frame dtype 0x%02X', dtype);
            end
        end

        function reportRecv(obj, n_bytes, t_rd, t_up)
            %REPORTRECV  Debug-only network-vs-CPU breakdown for one frame.
            %  Prints link throughput (MB/s) and unpack time so the
            %  bottleneck (slow link vs MATLAB-side decode) is unambiguous.
            if ~obj.Debug, return; end
            mb = n_bytes / 1e6;
            fprintf('[ECam]   recv %.1fMB in %.3fs (%.0f MB/s)  unpack %.3fs\n', ...
                mb, t_rd, mb / max(t_rd, 1e-6), t_up);
        end

        function frame = unpackRaw10(~, packed, h, w)
            %UNPACKRAW10  Decode RAW10 packed bytes → uint16 [H x W] 0-1023.
            %  4 pixels packed into 5 bytes:
            %    byte0=P0[9:2]  byte1=P1[9:2]
            %    byte2=P2[9:2]  byte3=P3[9:2]
            %    byte4=P3[1:0]|P2[1:0]<<2|P1[1:0]<<4|P0[1:0]<<6  (LE)
            n_px     = double(h) * double(w);
            n_groups = ceil(n_px / 4);
            pk       = reshape(packed(1:n_groups*5), 5, n_groups);

            lo = uint16(pk(5,:));
            p0 = bitor(bitshift(uint16(pk(1,:)),2), bitand(lo,          3));
            p1 = bitor(bitshift(uint16(pk(2,:)),2), bitand(bitshift(lo,-2), 3));
            p2 = bitor(bitshift(uint16(pk(3,:)),2), bitand(bitshift(lo,-4), 3));
            p3 = bitor(bitshift(uint16(pk(4,:)),2), bitand(bitshift(lo,-6), 3));

            all_px = [p0; p1; p2; p3];   % [4 x n_groups]
            frame  = reshape(all_px(1:n_px), [w, h])';
        end

        function data = rdBytes(obj, n)
            %RDBYTES  Read exactly n bytes from the tcpclient socket.
            %  A single blocking read() is correct and fast here: once bytes
            %  are present read() returns immediately (verified: sub-ms for
            %  headers and full frames). Earlier multi-second stalls were a
            %  server-side issue (a blocking v4l2-ctl subprocess), not this
            %  read path — that has been removed on the server.
            n = double(n);
            if n == 0, data = uint8([]); return; end

            deadline = tic;
            data = read(obj.Socket, n, 'uint8');

            % Top up in the rare case read() returns short.
            received = numel(data);
            while received < n
                if toc(deadline) > obj.RECV_TIMEOUT_S
                    error('ECamHDRClient:timeout', ...
                        'Got %d of %d bytes after %.0fs', ...
                        received, n, obj.RECV_TIMEOUT_S);
                end
                chunk = read(obj.Socket, n-received, 'uint8');
                if ~isempty(chunk)
                    data     = [data(:); chunk(:)];  %#ok
                    received = numel(data);
                else
                    pause(obj.POLL_INTERVAL);
                end
            end
            data = data(:)';
        end

        % ── Stream helpers ─────────────────────────────────────────────────────

        function streamPoll(obj)
            try
                if obj.Socket.NumBytesAvailable >= 10
                    frame = obj.recvFrame();
                    obj.StreamFrameCount = obj.StreamFrameCount + 1;
                    if ~isempty(obj.StreamCallback)
                        obj.StreamCallback(frame);
                    end
                end
            catch ME
                warning('ECamHDRClient:poll','%s',ME.message);
            end
        end

        function streamError(obj, err)
            warning('ECamHDRClient:streamErr','%s',err.message);
            obj.stopStream();
        end

    end % private
end