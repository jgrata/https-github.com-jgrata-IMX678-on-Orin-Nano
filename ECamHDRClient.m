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
        Verbose      (1,1) logical = true   % print per-capture status lines
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

        % ── setVerbose ────────────────────────────────────────────────────────
        function setVerbose(obj, tf)
            %SETVERBOSE  Toggle the per-capture status lines from capture()
            %  (the "[ECam] Capture/Frame/Actual" prints). cam.setVerbose(false)
            %  silences them; grab() is always silent. Debug stats are separate
            %  (see setDebug) and still require Debug=true.
            obj.Verbose = logical(tf);
        end

        % ── ping ──────────────────────────────────────────────────────────────
        function ping(obj)
            obj.requireConnected();
            obj.flushInput();
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
            obj.flushInput();                    % drop any stale/pending bytes
            obj.sendCmd(obj.CMD_GET_INFO);
            data = obj.recvResp();
            try
                info = jsondecode(char(data(:)'));
            catch
                % Desync: the response wasn't JSON (e.g. a stale 'OK'). Flush
                % and retry once so the control socket realigns.
                obj.flushInput();
                obj.sendCmd(obj.CMD_GET_INFO);
                data = obj.recvResp();
                info = jsondecode(char(data(:)'));
            end
        end

        % ── refresh ───────────────────────────────────────────────────────────
        function refresh(obj)
            %REFRESH  Re-query the server and update CameraInfo + synced params
            %  (including actual_exposure_ns / actual_gain). Use after changing
            %  exposure/gain and waiting ~1 frame, to read the settled actuals
            %  without a full capture().
            obj.requireConnected();
            obj.CameraInfo = obj.getInfo();
            obj.syncFromServer();
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

            obj.flushInput();                    % realign before the transaction
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

            if obj.Verbose
                if obj.p_exposure_ns == 0
                    exp_s = 'auto';
                else
                    exp_s = sprintf('%.0fms', obj.p_exposure_ns/1e6);
                end
                fprintf('[ECam] Capture  mode=%d  %d-bit  exp=%s\n', ...
                    obj.p_sensormode, obj.p_bit_depth, exp_s);
            end

            t_frame = tic;
            frame   = obj.grabFrame();   % flush + send + recv, resync on desync
            t_frame = toc(t_frame);

            t_info = tic;
            obj.CameraInfo = obj.getInfo();
            obj.syncFromServer();
            t_info = toc(t_info);

            if obj.Verbose
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
            frame = obj.grabFrame();
        end

        % ── autoExposure ────────────────────────────────────────────────────────
        function exp_ns = autoExposure(obj, target_frac, max_iter)
            %AUTOEXPOSURE  Set exposure so the brightest pixels sit just below
            %  saturation — no clipping, maximum usable dynamic range.
            %
            %  cam.autoExposure()               target 93% of full scale
            %  cam.autoExposure(frac)           custom target fraction (0-1)
            %  cam.autoExposure(frac, max_iter) cap the number of iterations
            %
            %  Iterates: grab a frame, measure the max, scale exposure toward
            %  the target. Exposure changes apply live (no pipeline restart),
            %  so this converges in a handful of fast steps. Returns the final
            %  exposure_ns. Full scale follows bit_depth (1023 @10-bit,
            %  4095 @12-bit), so run it after choosing the sensor mode.
            obj.requireConnected();
            if nargin < 2 || isempty(target_frac), target_frac = 0.93; end
            if nargin < 3 || isempty(max_iter),    max_iter    = 8;    end
            target_frac = max(0.30, min(0.99, target_frac));

            full   = 2^obj.p_bit_depth - 1;
            target = target_frac * full;

            for it = 1:max_iter
                pause(0.30);                 % let the last exposure change settle
                f  = obj.grab();
                mx = double(max(f(:)));
                fprintf('[ECam] AE %d: exp=%.2fms  max=%d (%.0f%% of %d)\n', ...
                    it, obj.p_exposure_ns/1e6, mx, 100*mx/full, full);

                if mx >= full
                    newexp = obj.p_exposure_ns * 0.5;      % clipping: halve
                else
                    newexp = obj.p_exposure_ns * (target / max(mx, 1));
                    if abs(newexp - obj.p_exposure_ns) < 0.03 * obj.p_exposure_ns
                        break;                              % within 3%: converged
                    end
                end

                newexp = max(450000, min(400000000, round(newexp)));
                if newexp == obj.p_exposure_ns, break; end  % hit a clamp
                obj.exposure_ns = newexp;                   % applies live
            end

            exp_ns = obj.p_exposure_ns;
            fprintf('[ECam] AE done: exposure=%.2fms  (%.0f%% target)\n', ...
                exp_ns/1e6, 100*target_frac);
        end

        % ── aeOnce ──────────────────────────────────────────────────────────────
        function exp_ns = aeOnce(obj, settle_s)
            %AEONCE  One-shot hardware auto-exposure: hand exposure to the
            %  sensor's AE, wait for it to converge, then PIN exposure to the
            %  value it chose — freezing AE so later frames run at full fps.
            %  Returns the pinned exposure_ns. Gain is left as-is (pin/auto it
            %  separately, or use aeagOnce to do both together).
            obj.requireConnected();
            if nargin < 2 || isempty(settle_s), settle_s = 3.0; end
            obj.exposure_ns = 0;                        % hand to sensor AE
            obj.waitActual(@() obj.actual_exposure_ns, 0.02, settle_s);
            pinned = obj.actual_exposure_ns;
            if pinned <= 0, pinned = obj.p_exposure_ns; end
            obj.exposure_ns = pinned;                   % pin -> AE frozen
            exp_ns = pinned;
            fprintf('[ECam] aeOnce: pinned exposure = %.2f ms\n', pinned/1e6);
        end

        % ── agOnce ──────────────────────────────────────────────────────────────
        function g = agOnce(obj, settle_s)
            %AGONCE  One-shot hardware auto-gain: hand gain to the sensor's
            %  AGC, wait for convergence, then PIN gain to the chosen value.
            %  Returns the pinned gain multiplier.
            obj.requireConnected();
            if nargin < 2 || isempty(settle_s), settle_s = 3.0; end
            obj.gain = 0;                               % hand to sensor AGC
            obj.waitActual(@() obj.actual_gain_value, 0.03, settle_s);
            pinned = obj.actual_gain_value;
            if pinned <= 0, pinned = obj.p_gain; end
            obj.gain = pinned;                          % pin -> AGC frozen
            g = pinned;
            fprintf('[ECam] agOnce: pinned gain = %.4fx\n', pinned);
        end

        % ── aeagOnce ────────────────────────────────────────────────────────────
        function [exp_ns, g] = aeagOnce(obj, settle_s)
            %AEAGONCE  One-shot auto exposure AND gain together: enable both,
            %  let them co-converge, then pin both. Best from unknown lighting,
            %  since AE and AGC interact (fixing one changes the other's target).
            obj.requireConnected();
            if nargin < 2 || isempty(settle_s), settle_s = 4.0; end
            obj.setParams('exposure_ns', 0, 'gain', 0);      % both auto
            obj.waitActual(@() obj.actual_exposure_ns, 0.02, settle_s);
            exp_ns = obj.actual_exposure_ns;
            g      = obj.actual_gain_value;
            if exp_ns <= 0, exp_ns = obj.p_exposure_ns; end
            if g      <= 0, g      = obj.p_gain;        end
            obj.setParams('exposure_ns', exp_ns, 'gain', g); % pin both
            fprintf('[ECam] aeagOnce: pinned exposure=%.2f ms  gain=%.4fx\n', ...
                exp_ns/1e6, g);
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

        % ── recordFramesFast ────────────────────────────────────────────────────
        function [frames, stats] = recordFramesFast(obj, nFrames, depth)
            %RECORDFRAMESFAST  Highest-throughput burst capture via a .NET
            %  background receiver (EcamReceiver.dll). A background thread
            %  receives frame N+1 while MATLAB unpacks/stores frame N, so the
            %  network transfer overlaps the CPU work and throughput approaches
            %  the gigabit ceiling (~10 fps at 4K 10-bit) instead of the
            %  serialized ~7 fps of recordFrames.
            %
            %  Requires EcamReceiver.dll next to this class file (build from
            %  EcamReceiver.cs:  csc /target:library /optimize+ EcamReceiver.cs).
            %  Opens a SECOND connection to the server for bulk frames; the
            %  control connection (this object) is used only for setup.
            obj.requireConnected();
            if nargin < 3 || isempty(depth), depth = 2; end

            persistent asmLoaded
            if isempty(asmLoaded)
                dll = fullfile(fileparts(which('ECamHDRClient')), 'EcamReceiver.dll');
                if ~isfile(dll)
                    error('ECamHDRClient:noDll', ...
                        'EcamReceiver.dll not found next to ECamHDRClient.m (build it from EcamReceiver.cs).');
                end
                NET.addAssembly(dll);
                asmLoaded = true;
            end

            H = double(obj.CameraInfo.height);
            W = double(obj.CameraInfo.width);
            frames = zeros(H, W, nFrames, 'uint16');

            rx = Ecam.Receiver(obj.Host, int32(obj.Port));
            guard = onCleanup(@() rx.Stop());   %#ok<NASGU>  stops thread + closes socket
            rx.Start(int32(depth));

            lastSeq = int32(0);
            t0 = tic;
            for k = 1:nFrames
                [payload, Hh, Ww, dtype, seq] = ...
                    rx.GetFrame(lastSeq, int32(10000));
                if isempty(payload)
                    error('ECamHDRClient:rxTimeout', ...
                        'Fast receiver timed out at frame %d/%d', k, nFrames);
                end
                lastSeq = seq;
                raw = uint8(payload);            % .NET byte[] -> MATLAB uint8
                if dtype == 17                    % 0x11 RAW10 packed
                    frames(:,:,k) = obj.unpackRaw10(raw(:)', double(Hh), double(Ww));
                elseif dtype == 18                % 0x12 RAW12 packed
                    frames(:,:,k) = obj.unpackRaw12(raw(:)', double(Hh), double(Ww));
                elseif dtype == 16                % 0x10 uint16
                    px = typecast(raw(:)', 'uint16');
                    frames(:,:,k) = reshape(px, [double(Ww), double(Hh)])';
                else
                    error('ECamHDRClient:dtype', 'Unknown dtype %d', dtype);
                end
            end
            elapsed = toc(t0);

            stats = struct('frames', nFrames, 'elapsed_s', elapsed, ...
                           'fps', nFrames / max(elapsed, eps));
            fprintf('[ECam] recordFramesFast: %d frames in %.2fs = %.1f fps\n', ...
                nFrames, elapsed, stats.fps);
        end

        % ── captureBracket ──────────────────────────────────────────────────────
        function [frames, meta] = captureBracket(obj, exposures_ns, gain)
            %CAPTUREBRACKET  Exposure-bracketed capture for HDR radiance work.
            %  [frames, meta] = cam.captureBracket([e1 e2 ... eN])
            %  [frames, meta] = cam.captureBracket([...], gainValue)
            %
            %  For each exposure (ns): set it, wait for the sensor to actually
            %  reach it (frame duration auto-extends for long exposures, so fps
            %  drops), then grab one full-bit frame and record the ACTUAL
            %  exposure/gain. Intended for STATIC scenes — each leg settles
            %  before capture. Returns:
            %    frames : uint16 [H x W x N]  (native bit depth: 10 or 12)
            %    meta   : struct array (1xN) with .exposure_ns .gain
            %             (actual values, for radiance reconstruction).
            %
            %  Gain is held fixed across the bracket (default: pin the current
            %  gain; pass gainValue to override). Exposures need not be sorted.
            %  Restores the prior exposure/gain when done.
            obj.requireConnected();
            exposures_ns = round(double(exposures_ns(:)'));
            N = numel(exposures_ns);
            if N < 1, error('ECamHDRClient:bracket','need >=1 exposure'); end

            % Pin gain so only exposure varies across the bracket.
            if nargin >= 3 && ~isempty(gain), g_fix = double(gain);
            else,  g_fix = obj.actual_gain_value; if g_fix<=0, g_fix = 1.0; end
            end
            orig_exp = obj.p_exposure_ns;
            orig_gain = obj.p_gain;
            obj.setParams('gain', g_fix);

            H = double(obj.CameraInfo.height);
            W = double(obj.CameraInfo.width);
            frames = zeros(H, W, N, 'uint16');
            meta = struct('exposure_ns', cell(1,N), 'gain', cell(1,N), ...
                          'requested_ns', cell(1,N));

            for k = 1:N
                obj.exposure_ns = exposures_ns(k);            % applies live
                obj.waitExposureSettle(exposures_ns(k));      % wait until reached
                obj.grab();                                   % flush one settled frame
                frames(:,:,k)       = obj.grab();             % keep the next
                meta(k).requested_ns = exposures_ns(k);
                meta(k).exposure_ns  = obj.actual_exposure_ns;
                meta(k).gain         = obj.actual_gain_value;
                fprintf('[ECam] bracket %d/%d: req=%.2fms actual=%.2fms gain=%.3fx  [%d..%d]\n', ...
                    k, N, exposures_ns(k)/1e6, meta(k).exposure_ns/1e6, ...
                    meta(k).gain, min(frames(:,:,k),[],'all'), max(frames(:,:,k),[],'all'));
            end

            obj.setParams('exposure_ns', orig_exp, 'gain', orig_gain);  % restore
        end

        % ── reconstructRadiance ─────────────────────────────────────────────────
        function [rad, wsum] = reconstructRadiance(obj, frames, meta, blacklevel, satfrac, flatfield)
            %RECONSTRUCTRADIANCE  Combine an exposure bracket into a linear HDR
            %  radiance map (Debevec-style weighted average; RAW sensor data is
            %  already linear, so no camera response curve is needed).
            %
            %  [rad, wsum] = cam.reconstructRadiance(frames, meta)
            %  [...] = cam.reconstructRadiance(frames, meta, blacklevel, satfrac, flatfield)
            %  flatfield: optional per-pixel response map from measureFlatField;
            %  the radiance is divided by it to remove PRNU/vignetting.
            %
            %  frames/meta come from captureBracket. For each exposure the
            %  per-pixel estimate is (DN - blacklevel) / (exposure_s * gain),
            %  weighted by a triangle that rejects saturated and noise-floor
            %  pixels, then averaged. Output `rad` is RELATIVE linear radiance
            %  (single, H x W); `wsum` is the summed weight (0 = no valid
            %  exposure covered that pixel — over/under-exposed everywhere).
            %
            %  blacklevel : sensor black level in DN (default 0 — MEASURE IT:
            %               a lens-capped frame's median gives a real value;
            %               it materially affects dark-region radiance).
            %  satfrac    : saturation cutoff as fraction of full scale (0.95).
            if nargin < 4 || isempty(blacklevel), blacklevel = 0;    end
            if nargin < 5 || isempty(satfrac),    satfrac    = 0.95; end
            [H, W, N] = size(frames);
            full = 2^obj.p_bit_depth - 1;
            sat  = satfrac * full;

            rad  = zeros(H, W, 'single');
            wsum = zeros(H, W, 'single');
            for k = 1:N
                f   = single(frames(:,:,k));
                eff = double(meta(k).exposure_ns)/1e9 * max(double(meta(k).gain), eps);
                w   = min(f - blacklevel, sat - f);   % triangle: peak mid-range
                w   = max(w, 0);
                w(f >= sat) = 0;                      % reject saturated
                rad  = rad  + w .* (max(f - blacklevel, 0) / eff);
                wsum = wsum + w;
            end
            rad = rad ./ max(wsum, eps('single'));    % relative linear radiance
            if nargin >= 6 && ~isempty(flatfield)     % flat-field correction
                rad = rad ./ single(flatfield);
            end
        end

        % ── measureBlackLevel ───────────────────────────────────────────────────
        function [darkframe, stats] = measureBlackLevel(obj, nframes, gain)
            %MEASUREBLACKLEVEL  Empirical black level / dark frame for radiance.
            %  ** CAP THE LENS (or use a fully dark scene) before calling. **
            %  Averages nframes at the shortest exposure into a per-pixel
            %  black-level image (removes the read pedestal AND fixed-pattern
            %  offset). Pass the result as `blacklevel` to reconstructRadiance.
            %
            %  cam.measureBlackLevel([nframes],[gain])
            %  IMPORTANT: the black level is GAIN-DEPENDENT on this sensor, so
            %  measure it at the SAME gain you use for the bracket (default 1.0
            %  — match captureBracket's gain).
            %
            %  Returns:
            %    darkframe : single [H x W]  — the dark reference to subtract
            %    stats     : per-Bayer-phase medians (.R .Gr .Gb .B) + .global
            %
            %  Measured at min exposure (exposure-independent pedestal); dark
            %  current at long bracket exposures is assumed negligible. RAW-based
            %  -> ISP-agnostic, ports across ISPs.
            obj.requireConnected();
            if nargin < 2 || isempty(nframes), nframes = 8;   end
            if nargin < 3 || isempty(gain),    gain    = 1.0; end
            oe = obj.p_exposure_ns; og = obj.p_gain; ov = obj.Verbose;
            obj.Verbose = false;
            obj.setParams('exposure_ns', 450000, 'gain', double(gain));
            pause(0.5);
            obj.grab();                                          % flush transitional
            acc = zeros(double(obj.CameraInfo.height), double(obj.CameraInfo.width));
            for k = 1:nframes, acc = acc + double(obj.grab()); end
            darkframe = single(acc / nframes);
            obj.setParams('exposure_ns', oe, 'gain', og);        % restore
            obj.Verbose = ov;

            R  = darkframe(1:2:end, 1:2:end);   Gr = darkframe(1:2:end, 2:2:end);
            Gb = darkframe(2:2:end, 1:2:end);   B  = darkframe(2:2:end, 2:2:end);
            stats = struct('R', median(R(:)),  'Gr', median(Gr(:)), ...
                           'Gb', median(Gb(:)),'B',  median(B(:)), ...
                           'global', median(darkframe(:)));
            fprintf('[ECam] blacklevel (DN): R=%.1f Gr=%.1f Gb=%.1f B=%.1f  global=%.1f\n', ...
                stats.R, stats.Gr, stats.Gb, stats.B, stats.global);
        end

        % ── measureFlatField ────────────────────────────────────────────────────
        function flatmap = measureFlatField(obj, dark, nframes, exposure_ns, gain, prnuOnly)
            %MEASUREFLATFIELD  Per-pixel flat-field (response non-uniformity) map.
            %  Point the camera at a UNIFORM illumination source, then:
            %    flat = cam.measureFlatField(dark)
            %  Captures nframes, dark-subtracts, averages, and normalizes PER
            %  BAYER PHASE to build a correction map (~1 mean). Pass it to
            %  reconstructRadiance (flatfield arg): corrected = radiance ./ map.
            %
            %  cam.measureFlatField(dark,[nframes],[exposure_ns],[gain],[prnuOnly])
            %
            %  prnuOnly=false (default): removes ALL spatial non-uniformity in
            %    the flat — sensor PRNU AND, if the lens is on, lens vignetting
            %    (flattens THIS lens).
            %  prnuOnly=true: high-passes each phase so only the high-frequency
            %    component (column/pixel PRNU — the banding) is corrected, and
            %    the smooth lens vignetting is left intact.
            %
            %  Aim for a mid flat level (~40-70% of full scale, unsaturated).
            %  Measure at the gain you'll use for the bracket.
            obj.requireConnected();
            if nargin<3||isempty(nframes),     nframes=8;                 end
            if nargin<4||isempty(exposure_ns), exposure_ns=obj.p_exposure_ns; end
            if nargin<5||isempty(gain),        gain=obj.p_gain; if gain<=0, gain=1; end; end
            if nargin<6||isempty(prnuOnly),    prnuOnly=false;            end

            oe=obj.p_exposure_ns; og=obj.p_gain; ov=obj.Verbose; obj.Verbose=false;
            obj.setParams('exposure_ns', round(exposure_ns), 'gain', double(gain));
            obj.waitExposureSettle(round(exposure_ns));
            obj.grab();
            acc = zeros(double(obj.CameraInfo.height), double(obj.CameraInfo.width));
            for k=1:nframes, acc = acc + double(obj.grab()); end
            obj.setParams('exposure_ns', oe, 'gain', og); obj.Verbose=ov;

            flat = single(acc/nframes) - single(dark);      % dark-subtracted signal
            fs   = 2^obj.p_bit_depth - 1;
            lvl  = median(flat(:));
            if lvl < 0.10*fs || lvl > 0.90*fs
                warning('ECamHDRClient:flatLevel', ...
                    'flat median %.0f DN is %.0f%% of full scale — aim ~40-70%%', ...
                    lvl, 100*lvl/fs);
            end

            flatmap = ones(size(flat), 'single');
            ph = {[1 1],[1 2],[2 1],[2 2]};                 % R Gr Gb B phases
            for p = 1:4
                r0=ph{p}(1); c0=ph{p}(2);
                sub = max(flat(r0:2:end, c0:2:end), eps('single'));
                if prnuOnly
                    m = sub ./ obj.boxlp(sub, 65);          % high-freq PRNU only
                else
                    m = sub ./ mean(sub(:));                % all spatial non-unif.
                end
                flatmap(r0:2:end, c0:2:end) = m;
            end
            fprintf('[ECam] flatfield: level %.0f DN (%.0f%% FS)  prnuOnly=%d\n', ...
                lvl, 100*lvl/fs, prnuOnly);
        end

        % ── measurePTC ──────────────────────────────────────────────────────────
        function ptc = measurePTC(obj, dark, gain, step)
            %MEASUREPTC  Photon-transfer curve on a UNIFORM source.
            %  Sweeps exposure (auto, from 450us up to saturation), grabs 2
            %  frames per level, and per Bayer phase computes mean signal and
            %  temporal variance via the 2-frame difference method (var(f1-f2)/2
            %  cancels fixed-pattern noise). Fits the shot-noise region to get:
            %  conversion gain K (e-/DN), read noise (DN & e-), full-well (e-),
            %  and dynamic range (dB). Plots variance-vs-mean per phase.
            %
            %  ptc = cam.measurePTC(dark, [gain], [step])
            %  dark : per-pixel dark from measureBlackLevel at the SAME gain
            %  gain : fixed analog gain for the sweep (default 1.0)
            %  step : exposure multiplier per level (default 1.4)
            %
            %  ** Point at a uniform source DIMMED so 450us reads ~5-10% FS, so
            %     the sweep spans dark->saturation. Assumes a temporally stable
            %     source (flicker inflates the variance). **
            obj.requireConnected();
            if nargin < 3 || isempty(gain), gain = 1.0; end
            if nargin < 4 || isempty(step), step = 1.4; end
            fs = 2^obj.p_bit_depth - 1;

            oe = obj.p_exposure_ns; og = obj.p_gain; ov = obj.Verbose;
            obj.Verbose = false;
            obj.setParams('gain', double(gain));

            ph = {'R',[1 1];'Gr',[1 2];'Gb',[2 1];'B',[2 2]};
            exps = []; mn = []; vr = [];        % mn/vr rows=level, cols=phase
            e = 450000; lvl = 0;
            while e <= 400000000 && lvl < 40
                obj.exposure_ns = round(e);
                obj.waitExposureSettle(round(e));
                obj.grab();                     % flush transitional frame
                f1 = single(obj.grab());
                f2 = single(obj.grab());
                lvl = lvl + 1;
                exps(lvl) = obj.actual_exposure_ns; %#ok<AGROW>
                for p = 1:4
                    r0 = ph{p,2}(1); c0 = ph{p,2}(2);
                    a1 = f1(r0:2:end, c0:2:end);
                    a2 = f2(r0:2:end, c0:2:end);
                    d  = dark(r0:2:end, c0:2:end);
                    mn(lvl,p) = mean((a1(:)+a2(:))/2 - d(:));  %#ok<AGROW> signal DN
                    df = a1(:) - a2(:);
                    vr(lvl,p) = var(df) / 2;                   %#ok<AGROW> temporal var
                end
                if median(f1(:)) >= 0.97*fs, break; end        % saturated
                e = e * step;
            end
            obj.setParams('exposure_ns', oe, 'gain', og);
            obj.Verbose = ov;

            ptc = struct('exposure_ns', exps, 'gain', gain, 'fullscale', fs);
            figure; hold on; co = lines(4);
            for p = 1:4
                m = mn(:,p); v = vr(:,p);
                sel = m > 0.05*fs & m < 0.70*fs;               % shot-noise region
                if nnz(sel) >= 2
                    c     = polyfit(m(sel), v(sel), 1);        % v = c1*m + c2
                    K     = 1/c(1);                            % e-/DN
                    rn_dn = sqrt(max(c(2),0));
                else
                    K = NaN; rn_dn = NaN;
                end
                rn_e  = rn_dn * K;
                fw_e  = K * max(m);                            % full well (e-)
                dr_db = 20*log10(fw_e / max(rn_e, eps));
                ptc.phase(p) = struct('name',ph{p,1}, 'mean',m(:)', 'var',v(:)', ...
                    'K',K, 'read_noise_DN',rn_dn, 'read_noise_e',rn_e, ...
                    'fullwell_e',fw_e, 'DR_dB',dr_db);
                loglog(m, v, 'o-', 'Color', co(p,:), 'DisplayName', ph{p,1});
                fprintf('[PTC] %-3s K=%.4f e-/DN  read=%.2f DN (%.1f e-)  FW=%.0f e-  DR=%.1f dB\n', ...
                    ph{p,1}, K, rn_dn, rn_e, fw_e, dr_db);
            end
            set(gca,'XScale','log','YScale','log'); grid on;
            xlabel('mean signal (DN)'); ylabel('temporal variance (DN^2)');
            title(sprintf('Photon-transfer curve  (gain %.2fx, %d-bit)', ...
                gain, obj.p_bit_depth));
            legend('Location','northwest');
            if numel(exps) < 5
                warning('ECamHDRClient:ptcShort', ...
                    'only %d usable levels — dim the source so 450us reads ~5-10%% FS', ...
                    numel(exps));
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
            obj.refreshInfo();   % live query so the value is current, not cached
            if ~isempty(fieldnames(obj.CameraInfo)) && ...
                    isfield(obj.CameraInfo,'actual_exposure_ns')
                v = obj.CameraInfo.actual_exposure_ns;
            else
                v = 0;
            end
        end

        function v = get.actual_gain_value(obj)
            obj.refreshInfo();   % live query so the value is current, not cached
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
            if v == 3
                error('ECamHDRClient:dolUnsupported', ...
                    ['sensormode 3 (native DOL HDR) is not functional: raw_capture ' ...
                     'produces no frames in DOL (EGL stream stays EMPTY) and it ' ...
                     'crash-loops/hangs the pipeline. Use software-bracket HDR on ' ...
                     'modes 0/1/2 via captureBracket. DOL needs IDolWdrSensorMode ' ...
                     'support in raw_capture (+ likely vendor driver support).']);
            end
            obj.p_sensormode = v;
            if obj.IsConnected, obj.setParams('sensormode', v); end
            % bit_depth tracks the sensor's native_bpp via syncFromServer once
            % the (async) pipeline restart completes and the next getInfo runs
            % (e.g. after your pause() + a refresh()/capture()). Reading it here
            % would race the restart and grab the OLD mode's value.
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

        function refreshInfo(obj)
            %REFRESHINFO  Pull a fresh CameraInfo snapshot from the server so
            %  actual_* reflect current sensor state. Guarded + no-throw so it
            %  is safe to call from property getters (incl. object display).
            %  NOTE: after changing a long exposure the sensor still needs ~1
            %  frame at the new setting before the actual value updates.
            if ~obj.IsConnected, return; end
            try
                obj.CameraInfo = obj.getInfo();
            catch
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
                   'hdr_exp_ratio','p_hdr_exp_ratio'; ...
                   'sat_threshold','p_sat_threshold'; ...
                   'lossless',  'p_lossless'};
            for k = 1:size(map,1)
                if isfield(info, map{k,1})
                    obj.(map{k,2}) = info.(map{k,1});
                end
            end
            % bit_depth reflects the ACTUAL data on the wire: the lossless
            % packed path carries the sensor's NATIVE bits (native_bpp), so
            % track that (mode 0 -> 12); only for lossless=false (scaled uint16)
            % does the server's bit_depth scaling knob apply.
            if isfield(info,'lossless') && info.lossless && ...
                    isfield(info,'native_bpp') && info.native_bpp > 0
                obj.p_bit_depth = info.native_bpp;
            elseif isfield(info,'bit_depth')
                obj.p_bit_depth = info.bit_depth;
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

        function flushInput(obj)
            %FLUSHINPUT  Discard any bytes sitting in the socket before a fresh
            %  one-shot request, so a prior interrupted/partial read can't
            %  desync the next response. Safe only for strict request-response
            %  (grab/capture) — NOT the pipelined streamPull, which keeps
            %  multiple frames in flight.
            try
                n = obj.Socket.NumBytesAvailable;
                if n > 0, read(obj.Socket, n, 'uint8'); end
            catch
            end
        end

        function frame = grabFrame(obj)
            %GRABFRAME  One CMD_CAPTURE round-trip with desync recovery: flush
            %  stale input, request a frame, read it; on a bad status byte
            %  (stream misalignment) resync and retry once.
            obj.flushInput();
            obj.sendCmd(obj.CMD_CAPTURE);
            try
                frame = obj.recvFrame();
            catch ME
                if strcmp(ME.identifier, 'ECamHDRClient:badStatus')
                    obj.flushInput();
                    obj.sendCmd(obj.CMD_CAPTURE);
                    frame = obj.recvFrame();   % retry once after resync
                else
                    rethrow(ME);
                end
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

            elseif dtype == 0x12
                % RAW12 packed: server packs in groups of 2 pixels -> 3 bytes,
                % padding a partial final group: n_bytes = ceil(H*W/2) * 3.
                n_groups = ceil(double(H) * double(W) / 2);
                n_packed = n_groups * 3;
                t_rd     = tic;
                raw      = obj.rdBytes(n_packed);
                t_rd     = toc(t_rd);
                t_up     = tic;
                frame    = obj.unpackRaw12(uint8(raw(:)'), H, W);
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

        function v = waitActual(obj, readFcn, tol, settle_s)
            %WAITACTUAL  Poll an actual-value getter (refreshing CameraInfo each
            %  step) until it stabilizes — two consecutive reads within `tol`
            %  fractional change — or settle_s elapses. Used by ae/agOnce to
            %  detect AE/AGC convergence before pinning. Returns last value.
            prev = -1; stable = 0; v = 0;
            t0 = tic;
            while toc(t0) < settle_s
                pause(0.25);
                obj.CameraInfo = obj.getInfo();     % refresh actuals from server
                v = readFcn();
                if v > 0 && abs(v - prev) <= tol * max(v, 1)
                    stable = stable + 1;
                    if stable >= 2, break; end       % converged
                else
                    stable = 0;
                end
                prev = v;
            end
        end

        function ok = waitExposureSettle(obj, target_ns, timeout_s)
            %WAITEXPOSURESETTLE  Block until the sensor's ACTUAL exposure reaches
            %  target_ns (within 5%), or a timeout (exposure genuinely clamped
            %  by the mode/range). Returns true if the target was reached.
            %
            %  A new exposure takes a few frames to appear; during that time the
            %  OLD value is briefly stable, so we do NOT use a stability
            %  shortcut (that caused a long leg to be captured at the previous
            %  exposure). Instead: unconditionally flush a few frame periods,
            %  then poll for the target. Poll/timeout scale with the exposure
            %  since long exposures lower fps (frame period ~= exposure).
            target = double(target_ns);
            fp     = max(0.03, target/1e9);              % new frame period
            if nargin < 3 || isempty(timeout_s)
                timeout_s = max(2.5, 10 * fp);
            end
            pause(3 * fp);                                % let the change flush in
            tol = 0.05; t0 = tic; ok = false;
            while toc(t0) < timeout_s
                a = double(obj.actual_exposure_ns);      % live query
                if a > 0 && abs(a - target) <= tol*max(target,1)
                    ok = true; return;                    % reached target
                end
                pause(fp);
            end
            warning('ECamHDRClient:expClamp', ...
                'exposure did not reach %.2fms (actual %.2fms) — clamped?', ...
                target/1e6, double(obj.actual_exposure_ns)/1e6);
        end

        function lp = boxlp(~, A, k)
            %BOXLP  Separable box low-pass with edge normalization (toolbox-free
            %  — uses conv2). Approximates the smooth (vignetting) component so
            %  measureFlatField(prnuOnly=true) can isolate high-frequency PRNU.
            h   = ones(1, k, 'single') / k;
            num = conv2(h', h, A, 'same');
            den = conv2(h', h, ones(size(A), 'single'), 'same');
            lp  = num ./ den;
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

        function frame = unpackRaw12(~, packed, h, w)
            %UNPACKRAW12  Decode RAW12 packed bytes → uint16 [H x W] 0-4095.
            %  2 pixels packed into 3 bytes (matches C++ pack_raw12):
            %    byte0=P0[11:4]  byte1=P1[11:4]
            %    byte2=P0[3:0] | P1[3:0]<<4   (low nibbles, LE)
            n_px     = double(h) * double(w);
            n_groups = ceil(n_px / 2);
            pk       = reshape(packed(1:n_groups*3), 3, n_groups);

            lo = uint16(pk(3,:));
            p0 = bitor(bitshift(uint16(pk(1,:)),4), bitand(lo,           15));
            p1 = bitor(bitshift(uint16(pk(2,:)),4), bitand(bitshift(lo,-4), 15));

            all_px = [p0; p1];           % [2 x n_groups]
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