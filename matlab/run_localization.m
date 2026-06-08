%% run_localization.m - UWB RTLS host: receive ranges, solve, filter, plot, log.
%
% Run from anywhere — paths are absolute.

clear; clc;

%% ---- Configuration -------------------------------------------------------
TRANSPORT   = "udp";       % "serial" or "udp"
SERIAL_PORT = "COM7";
SERIAL_BAUD = 115200;
UDP_PORT    = 5005;

SCRIPT_DIR   = fileparts(mfilename('fullpath'));
ANCHORS_JSON = fullfile(SCRIPT_DIR, 'config', 'anchors.json');
if isfile(ANCHORS_JSON)
    cfg = rtls.AnchorConfig.fromJson(ANCHORS_JSON);
else
    warning('config/anchors.json not found - using built-in 3-anchor example.');
    cfg = rtls.AnchorConfig();
end

%% ---- Reliability settings ------------------------------------------------
WATCHDOG_SEC    = 10;   % warn + auto-reconnect if no valid packet for this long
AGE_GATE_SEC    = 3.0;  % clear anchor range history if not seen for this long
EKF_JUMP_THRESH = 2.0;  % reset EKF if output is >2 m from multilateration result
%--------------------------------------------------------------------------

%% ---- Default tuning knobs ------------------------------------------------
RANGE_FILT_N = 8;      % sliding median window per anchor (1-16)
EKF_Q_ACCEL  = 0.005;  % EKF process noise  (lower = smoother, slower)
RANGE_SIGMA  = 0.20;   % multilateration measurement noise (metres)
GATE_K       = 2.5;    % NLOS gate (MAD multiplier)
EMA_ALPHA    = 0.15;   % output EMA alpha  (0=frozen, 1=raw EKF output)
RANGE_BIAS_M = 0.04;   % global multipath bias subtracted from all ranges (metres)
%--------------------------------------------------------------------------

%% ---- Live tuning panel ---------------------------------------------------
setappdata(0, 'uwbTune', struct( ...
    'rangeFiltN',  RANGE_FILT_N, ...
    'ekfQAccel',   EKF_Q_ACCEL,  ...
    'emaAlpha',    EMA_ALPHA,    ...
    'gateK',       GATE_K,       ...
    'rangeBiasM',  RANGE_BIAS_M));

% panelDefs: {label, field, min, max, defaultVal, format}
panelDefs = { ...
    'RANGE_FILT_N  (1-16, integer)',   'rangeFiltN',  1,     16,   RANGE_FILT_N, '%.0f'; ...
    'EKF_Q_ACCEL  (0.001-0.10)',       'ekfQAccel',   0.001, 0.10, EKF_Q_ACCEL,  '%.4f'; ...
    'EMA_ALPHA  (0.05-1.0)',           'emaAlpha',    0.05,  1.0,  EMA_ALPHA,    '%.2f'; ...
    'GATE_K  (1.5-5.0)',               'gateK',       1.5,   5.0,  GATE_K,       '%.1f'; ...
    'RANGE_BIAS_M  (0-0.10 m)',        'rangeBiasM',  0,     0.10, RANGE_BIAS_M, '%.3f'; ...
};
nKnobs = size(panelDefs, 1);
rowH   = 55;
topY   = 55 + (nKnobs - 1) * rowH;   % extra 30px at top for status label

tf = figure('Name','UWB Tuning','NumberTitle','off','MenuBar','none', ...
    'Position',[50 200 380 nKnobs*rowH+90],'Resize','off', ...
    'CloseRequestFcn', @(f,~) delete(f));

% Status label at the top of the panel.
statusLbl = uicontrol(tf, 'Style','text', 'Units','pixels', ...
    'Position',[10 nKnobs*rowH+58 360 20], ...
    'String','Waiting for first packet...', ...
    'HorizontalAlignment','left', 'FontSize',8, ...
    'ForegroundColor',[0.55 0.55 0.55]);

sliderH = gobjects(nKnobs, 1);
valLblH = gobjects(nKnobs, 1);
for i = 1:nKnobs
    y = topY - (i-1)*rowH;
    uicontrol(tf, 'Style','text', 'Units','pixels', ...
        'Position',[10 y+28 240 16], 'String',panelDefs{i,1}, ...
        'HorizontalAlignment','left', 'FontSize',9);
    valLblH(i) = uicontrol(tf, 'Style','text', 'Units','pixels', ...
        'Position',[255 y+28 115 16], ...
        'String', sprintf(panelDefs{i,6}, panelDefs{i,5}), ...
        'HorizontalAlignment','right', 'FontSize',9, 'FontWeight','bold');
    fld = panelDefs{i,2};  fmt = panelDefs{i,6};  vl = valLblH(i);
    sliderH(i) = uicontrol(tf, 'Style','slider', 'Units','pixels', ...
        'Position',[10 y 360 24], ...
        'Min',panelDefs{i,3}, 'Max',panelDefs{i,4}, 'Value',panelDefs{i,5}, ...
        'Callback', @(s,~) uwbUpdateTune(s, fld, fmt, vl));
end

% Preset buttons  [rangeFiltN, ekfQAccel, emaAlpha, gateK, rangeBiasM]
presets = { ...
    'Stationary', [10, 0.002, 0.08, 3.0, 0.05]; ...
    'Slow Walk',  [ 6, 0.015, 0.25, 2.5, 0.04]; ...
    'Fast Walk',  [ 3, 0.050, 0.50, 2.0, 0.03]; ...
};
btnW = 108;  gap = 9;
for p = 1:size(presets,1)
    pvals = presets{p,2};
    uicontrol(tf, 'Style','pushbutton', 'Units','pixels', ...
        'Position',[gap+(p-1)*(btnW+gap) 5 btnW 26], 'String',presets{p,1}, ...
        'Callback', @(~,~) uwbApplyPreset(pvals, panelDefs, sliderH, valLblH));
end
%--------------------------------------------------------------------------

%% ---- Set up pipeline -----------------------------------------------------
if TRANSPORT == "serial"
    rx = rtls.UwbReceiver('serial', 'Port', SERIAL_PORT, 'Baud', SERIAL_BAUD, 'Timeout', 1.0);
else
    rx = rtls.UwbReceiver('udp', 'LocalPort', UDP_PORT, 'Timeout', 1.0);
end

ml  = rtls.Multilaterator(cfg.dim);
ml.gateK      = GATE_K;
ml.rangeSigma = RANGE_SIGMA;

ekf = rtls.PositionEKF(cfg.dim);
ekf.qAccel = EKF_Q_ACCEL;

plotter = rtls.LivePlotter(cfg);

rangeHist      = struct();   % per-anchor sliding range buffers
anchorLastSeen = struct();   % per-anchor last-seen time (datenum)

logName = fullfile(SCRIPT_DIR, ...
    sprintf('rtls_log_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
fid = fopen(logName, 'w');
if fid < 0; error('Cannot open log file: %s', logName); end
if cfg.dim == 2
    fprintf(fid, 't_ms,x,y,nUsed,rms\n');
else
    fprintf(fid, 't_ms,x,y,z,nUsed,rms\n');
end

fprintf('Listening (%s).  Close the map figure or press Ctrl-C to stop.\n', TRANSPORT);
fprintf('Tuning panel open.  Watchdog: %.0f s.  Age-gate: %.1f s.\n', ...
    WATCHDOG_SEC, AGE_GATE_SEC);

prev_t    = NaN;
lastPos   = [];
emaPos    = [];
pktCount  = 0;
ekfResets = 0;
watchdogTic = tic;

%% ---- Main loop -----------------------------------------------------------
try
    while ishandle(plotter.fig)

        % ---- Pull latest tuning values ------------------------------------
        tp           = getappdata(0, 'uwbTune');
        RANGE_FILT_N = max(1, round(tp.rangeFiltN));
        ekf.qAccel   = tp.ekfQAccel;
        EMA_ALPHA    = tp.emaAlpha;
        ml.gateK     = tp.gateK;
        RANGE_BIAS_M = tp.rangeBiasM;

        % ---- Age-gate: evict anchor history not seen for AGE_GATE_SEC ----
        fnames = fieldnames(anchorLastSeen);
        for fi = 1:numel(fnames)
            age_s = toc(anchorLastSeen.(fnames{fi}));
            if age_s > AGE_GATE_SEC
                if isfield(rangeHist, fnames{fi})
                    rangeHist = rmfield(rangeHist, fnames{fi});
                end
                anchorLastSeen = rmfield(anchorLastSeen, fnames{fi});
            end
        end

        % ---- Watchdog: flag stale link, auto-reconnect UDP ---------------
        sinceGood = toc(watchdogTic);
        if sinceGood > WATCHDOG_SEC
            msg = sprintf('NO DATA %.0f s — check tag WiFi/USB', sinceGood);
            statusLbl.String        = msg;
            statusLbl.ForegroundColor = [0.85 0 0];
            fprintf('[%s] %s\n', char(datetime('now','Format','HH:mm:ss')), msg);
            if TRANSPORT == "udp"
                try
                    delete(rx);
                    rx = rtls.UwbReceiver('udp', 'LocalPort', UDP_PORT, 'Timeout', 1.0);
                    watchdogTic = tic;   % back-off: don't spam reconnects
                    fprintf('[%s] UDP receiver recreated.\n', char(datetime('now','Format','HH:mm:ss')));
                catch rxErr
                    fprintf('[%s] Reconnect failed: %s\n', ...
                        char(datetime('now','Format','HH:mm:ss')), rxErr.message);
                end
            end
        end

        % ---- Receive one packet ------------------------------------------
        s = rx.next();
        if ~s.valid; continue; end

        % Valid packet — reset watchdog, update status.
        watchdogTic = tic;
        pktCount    = pktCount + 1;
        statusLbl.String = sprintf('Pkt #%d  |  EKF resets: %d  |  bias: %.0f mm', ...
            pktCount, ekfResets, RANGE_BIAS_M * 1000);
        statusLbl.ForegroundColor = [0 0.50 0];

        % ---- Inner processing in its own try-catch -----------------------
        % A bad packet or transient NaN must not kill the whole session.
        try
            % Map anchor ids -> coordinates.
            [A, found] = cfg.coordsFor(s.ids);
            d = s.dist(found);
            A = A(found, :);
            if numel(d) < cfg.dim + 1; continue; end

            % Per-anchor median pre-filter + age-stamp.
            activeIds = s.ids(found);
            d_filt = d;
            for k = 1:numel(activeIds)
                fld = sprintf('a%d', activeIds(k));
                anchorLastSeen.(fld) = tic;
                if ~isfield(rangeHist, fld)
                    rangeHist.(fld) = d(k);
                else
                    buf = [rangeHist.(fld), d(k)];
                    if numel(buf) > RANGE_FILT_N
                        buf = buf(end-RANGE_FILT_N+1:end);
                    end
                    rangeHist.(fld) = buf;
                end
                d_filt(k) = median(rangeHist.(fld));
            end

            % Global multipath bias correction (keep ranges physically positive).
            d_filt = max(0.05, d_filt - RANGE_BIAS_M);

            % Multilateration.
            [pos, info] = ml.solve(A, d_filt, lastPos);
            if ~info.ok || ~all(isfinite(pos)); continue; end
            lastPos = pos;

            % Time step.
            if isnan(prev_t); dt = 0.1;
            else;             dt = max((s.t_ms - prev_t) / 1000, 1e-3);
            end
            prev_t = s.t_ms;

            % EKF.
            R = info.cov(1:cfg.dim, 1:cfg.dim);
            if any(~isfinite(R(:))); R = []; end
            [fpos, ~] = ekf.step(dt, pos, R);

            % EKF divergence guard: reset if filter drifts far from solver or
            % covariance trace blows up (covariance > 5m std-dev in any axis).
            ekfJump = norm(fpos - pos);
            if ekfJump > EKF_JUMP_THRESH || trace(ekf.P) > 25
                ekfResets = ekfResets + 1;
                fprintf('[%s] EKF reset #%d (jump=%.2f m, trP=%.1f)\n', ...
                    char(datetime('now','Format','HH:mm:ss')), ekfResets, ekfJump, trace(ekf.P));
                ekf.initialize(pos);
                fpos  = pos;
                emaPos = [];
            end

            % Output EMA.
            if isempty(emaPos); emaPos = fpos;
            else;               emaPos = EMA_ALPHA * fpos + (1 - EMA_ALPHA) * emaPos;
            end

            plotter.update(emaPos, info);

            % Log.
            if cfg.dim == 2
                fprintf(fid, '%d,%.4f,%.4f,%d,%.4f\n', ...
                    s.t_ms, emaPos(1), emaPos(2), sum(info.used), info.rms);
            else
                fprintf(fid, '%d,%.4f,%.4f,%.4f,%d,%.4f\n', ...
                    s.t_ms, emaPos(1), emaPos(2), emaPos(3), sum(info.used), info.rms);
            end

        catch ME_inner
            % Log the bad sweep but keep running.
            fprintf('[%s] Sweep skipped: %s\n', char(datetime('now','Format','HH:mm:ss')), ME_inner.message);
        end
    end % while

catch ME
    fprintf('Loop ended: %s\n', ME.message);
end

%% ---- Cleanup -------------------------------------------------------------
fclose(fid);
try
    delete(rx);
catch
end
fprintf('Stopped. %d packets received, %d EKF resets.\n', pktCount, ekfResets);
fprintf('Log saved to:\n  %s\n', logName);

%% ---- Local functions (tuning panel callbacks) ----------------------------
function uwbUpdateTune(src, field, fmt, lbl)
    tp = getappdata(0, 'uwbTune');
    tp.(field) = src.Value;
    setappdata(0, 'uwbTune', tp);
    lbl.String = sprintf(fmt, src.Value);
end

function uwbApplyPreset(vals, defs, sliders, lbls)
    tp = getappdata(0, 'uwbTune');
    for i = 1:numel(vals)
        v = max(defs{i,3}, min(defs{i,4}, vals(i)));
        tp.(defs{i,2}) = v;
        sliders(i).Value = v;
        lbls(i).String   = sprintf(defs{i,6}, v);
    end
    setappdata(0, 'uwbTune', tp);
end
