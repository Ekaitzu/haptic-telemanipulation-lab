%% exp2_kb_sweep_auto_validate.m
% Generic EXP2 K-B sweep with automatic representative-point validation.
%
% It can be used for:
%   - Stage 1: continuous-time nominal model
%   - Stage 2: sampled-data 1 kHz model
%
% Workflow:
%   1) Run K-B sweep
%   2) Automatically pick representative points:
%        - one decaying candidate
%        - one near-boundary candidate
%        - one divergent candidate
%   3) Re-run those points
%   4) Plot time responses
%
% IMPORTANT:
%   Set cfg.modelKind = 'stage1' or 'stage2' below.
%
% Stage 1 expected model:
%   modelName = 'exp2_bilateral_nodelay'
%   logged signals = x_M, x_S, optional F_c_M, F_c_S
%
% Stage 2 expected model:
%   modelName = 'exp2_bilateral_nodelay_stg2'
%   logged signals = x_M_k, x_S_k, optional F_c_M_k, F_c_S_k

clear; clc;

%% ============================================================
%  USER CONFIGURATION
%  ============================================================

cfg.modelKind = 'stage2';   % 'stage1' or 'stage2'

% Simulation horizon: 6 s ensures T_sim >= 2*W for the envelope ratio
% and allows the bilateral coupled response to reveal its decay or
% growth trend over many oscillation cycles (slowest period ~ 0.5 s
% at low K, low B).
cfg.stopTime  = '6';
cfg.useFastRestart = true;

% Automatic validation after sweep
cfg.autoValidate = true;
cfg.validationStopTime = '6';
cfg.saveValidationFigures = true;
cfg.validationFigureFolder = 'exp2_auto_validation_figures';

% Automatic storage of figure
cfg.saveMapFigure = true;
cfg.mapFigureFolder = 'saved_map_figures';
cfg.mapFigureResolution = 300;

% -------------------------
% Shared no-delay experiment
% -------------------------
cfg.Td = 0.0;

% K-B sweep ranges
cfg.K_vals = linspace(0, 150, 50);   % [N/m]
cfg.B_vals = linspace(0, 4, 50);   % [Ns/m]

% Initial-condition excitation
cfg.xM0 = 0.020;     % [m]
cfg.vM0 = 0.0;       % [m/s]
cfg.xS0 = 0.0;       % [m]
cfg.vS0 = 0.0;       % [m/s]

% Plant parameters
cfg.I_eq = 1.58e-6;
cfg.b_eq = 6.57e-6;
cfg.k_eq = 2.9e-4;
cfg.r_s  = 0.075;
cfg.r_h  = 0.090;
cfg.r_m  = 0.010;

% Derived handle-space parameters
cfg.m_lin = cfg.I_eq * (cfg.r_s / (cfg.r_h * cfg.r_m))^2;
cfg.b_lin = cfg.b_eq * (cfg.r_s / (cfg.r_h * cfg.r_m))^2;
cfg.k_lin = cfg.k_eq * (cfg.r_s / (cfg.r_h * cfg.r_m))^2;

% Stage 2 defaults (harmless if modelKind = 'stage1')
cfg.Ts    = 0.001;
cfg.a     = 0.92;

% -------- Classification thresholds --------
xRef = max(abs([cfg.xM0, cfg.xS0, 1e-6]));
cfg.classification.maxAbsX      = 20*xRef;
cfg.classification.maxAbsF      = 1e3;
cfg.classification.envGrowDiv   = 1.05;
cfg.classification.envGrowNear  = 0.30;

% Envelope window length: W = 2.0 s so that T_sim (6 s) >= 2*W (4 s),
% ensuring the envelope ratio uses its intended first/last windows
% rather than the quarter-based fallback.  Each 2 s window captures
% many oscillation cycles even at the lowest coupling stiffness values.
cfg.classification.minWindowSec = 2.0;

% Map styling
cfg.showCellGrid = true;
cfg.gridColor = [0.55 0.55 0.55];
cfg.gridAlpha = 0.7;
cfg.gridLineWidth = 0.6;

%% ============================================================
%  MODEL-SPECIFIC CONFIGURATION
%  ============================================================

switch lower(cfg.modelKind)
    case 'stage1'
        cfg.modelName = 'exp2_bilateral_nodelay_stg1';
        cfg.signalNames.xM  = 'x_M';
        cfg.signalNames.xS  = 'x_S';
        cfg.signalNames.fcM = 'F_c_M';   % optional
        cfg.signalNames.fcS = 'F_c_S';   % optional
        cfg.resultPrefix = 'stg1';
        cfg.plotTitle = 'EXP2 Stage 1';
    case 'stage2'
        cfg.modelName = 'exp2_bilateral_nodelay_stg2';
        cfg.signalNames.xM  = 'x_M_k';
        cfg.signalNames.xS  = 'x_S_k';
        cfg.signalNames.fcM = 'F_c_M_k'; % optional
        cfg.signalNames.fcS = 'F_c_S_k'; % optional
        cfg.resultPrefix = 'stg2';
        cfg.plotTitle = 'EXP2 Stage 2';
    otherwise
        error('cfg.modelKind must be ''stage1'' or ''stage2''.');
end

%% ============================================================
%  PREPARE SWEEP
%  ============================================================

nK = numel(cfg.K_vals);
nB = numel(cfg.B_vals);
totalRuns = nK * nB;
completedRuns = 0;
lastPrintedPercent = -1;
tStartSweep = tic;

resultMap = zeros(nB, nK);   % 0 decaying, 1 near-critical, 2 divergent
summaryRows = [];
rowCounter = 0;

fprintf('Starting %s sweep: %d total simulations (%d K values x %d B values)\n', ...
    cfg.plotTitle, totalRuns, nK, nB);

scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir);

if ~bdIsLoaded(cfg.modelName)
    load_system(cfg.modelName);
end

if cfg.useFastRestart
    set_param(cfg.modelName, 'FastRestart', 'on');
end

%% ============================================================
%  MAIN SWEEP
%  ============================================================

for iK = 1:nK
    for iB = 1:nB
        K_val = cfg.K_vals(iK);
        B_val = cfg.B_vals(iB);

        in = buildSimulationInput(cfg, cfg.modelName, cfg.stopTime, K_val, B_val);

        try
            simOut = sim(in);
            logs = simOut.logsout;

            % Required signals
            xM = getLogged(logs, cfg.signalNames.xM);
            xS = getLogged(logs, cfg.signalNames.xS);

            % Optional signals
            fcM = getLoggedSafe(logs, cfg.signalNames.fcM);
            fcS = getLoggedSafe(logs, cfg.signalNames.fcS);

            t = xM.Time(:);
            xM_data = squeeze(xM.Data(:));
            xS_data = squeeze(xS.Data(:));

            % Metrics
            metrics = struct();
            metrics.maxAbsX = max(abs([xM_data; xS_data]));
            xEnv = max(abs([xM_data, xS_data]), [], 2);
            metrics.envRatioX = envelopeRatio(t, xEnv, cfg.classification.minWindowSec);
            metrics.rmsErr = rms(xM_data - xS_data);
            metrics.maxAbsF = maxAbsOptional(fcM, fcS);

            classCode = classifyRun(metrics, cfg.classification);
            resultMap(iB, iK) = classCode;

            rowCounter = rowCounter + 1;
            summaryRows(rowCounter,:) = [K_val, B_val, metrics.maxAbsX, ...
                metrics.envRatioX, metrics.rmsErr, metrics.maxAbsF, classCode]; %#ok<SAGROW>

        catch ME
            warning('Simulation failed for K=%.3f, B=%.3f -> marked divergent.\n%s', ...
                K_val, B_val, ME.message);
            resultMap(iB, iK) = 2;
            rowCounter = rowCounter + 1;
            summaryRows(rowCounter,:) = [K_val, B_val, NaN, NaN, NaN, NaN, 2]; %#ok<SAGROW>
        end

        % Progress
        completedRuns = completedRuns + 1;
        currentPercent = floor(100 * completedRuns / totalRuns);
        if currentPercent > lastPrintedPercent
            elapsedTime = toc(tStartSweep);
            estimatedTotal = elapsedTime * totalRuns / completedRuns;
            remainingTime = estimatedTotal - elapsedTime;

            fprintf(['Progress: %3d%% (%d/%d) | K=%.2f | B=%.2f | ' ...
                     'Elapsed: %.1f min | Remaining: %.1f min\n'], ...
                     currentPercent, completedRuns, totalRuns, ...
                     K_val, B_val, elapsedTime/60, remainingTime/60);

            lastPrintedPercent = currentPercent;
        end
    end
end

if cfg.useFastRestart
    set_param(cfg.modelName, 'FastRestart', 'off');
end

%% ============================================================
%  RESULTS TABLE
%  ============================================================

summaryTable = array2table(summaryRows, 'VariableNames', ...
    {'K', 'B', 'maxAbsX', 'envRatioX', 'rmsErr', 'maxAbsF', 'classCode'});

classLabels = strings(height(summaryTable),1);
classLabels(summaryTable.classCode == 0) = "decaying";
classLabels(summaryTable.classCode == 1) = "near-critical";
classLabels(summaryTable.classCode == 2) = "divergent";
summaryTable.classLabel = classLabels;

assignin('base', ['resultMap_' cfg.resultPrefix], resultMap);
assignin('base', ['summaryTable_' cfg.resultPrefix], summaryTable);
assignin('base', ['cfgSweep_' cfg.resultPrefix], cfg);

%% ============================================================
%  PLOT MAP
%  ============================================================

figMap = figure('Name', [cfg.plotTitle ' parameter space'], 'Color', 'w');
imagesc(cfg.K_vals, cfg.B_vals, resultMap);
set(gca, 'YDir', 'normal', 'FontSize', 13);
caxis([0 2]);
xlabel('K [N/m]', 'FontSize', 14);
ylabel('B [Ns/m]', 'FontSize', 14);
title([cfg.plotTitle ': envelope ratio + force check'], 'FontSize', 14);
colormap(gca, [0.2 0.7 0.2; 0.95 0.75 0.2; 0.85 0.2 0.2]);
c = colorbar;
c.Ticks = [0, 1, 2];
c.TickLabels = {'decaying', 'near-critical', 'divergent'};
c.FontSize = 12;

if cfg.showCellGrid
    hold on;
    overlayCellGrid(cfg.K_vals, cfg.B_vals, cfg.gridColor, cfg.gridAlpha, cfg.gridLineWidth);
    hold off;
end

if cfg.saveMapFigure
    outDir = fullfile(scriptDir, cfg.mapFigureFolder);
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    fileBase = sprintf('%s_parameter_space', cfg.resultPrefix);

    exportgraphics(figMap, fullfile(outDir, [fileBase '.png']), ...
        'Resolution', cfg.mapFigureResolution);

    savefig(figMap, fullfile(outDir, [fileBase '.fig']));
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
fileBase = sprintf('%s_parameter_space_%s', cfg.resultPrefix, timestamp);

%% ============================================================
%  AUTOMATIC REPRESENTATIVE-POINT VALIDATION
%  ============================================================

if cfg.autoValidate
    fprintf('\nSelecting automatic validation candidates...\n');
    selectedPoints = selectValidationCandidates(summaryTable, cfg.classification);

    validationSummary = table();
    if ~isempty(selectedPoints)
        fprintf('Selected points:\n');
        disp(selectedPoints);

        if cfg.saveValidationFigures
            outDir = fullfile(scriptDir, [cfg.validationFigureFolder '_' cfg.resultPrefix]);
            if ~exist(outDir, 'dir')
                mkdir(outDir);
            end
        end

        validationRows = {};
        for p = 1:height(selectedPoints)
            ptLabel = selectedPoints.pointType{p};
            K_val   = selectedPoints.K(p);
            B_val   = selectedPoints.B(p);

            fprintf('Validating %s | K = %.3f | B = %.3f\n', ptLabel, K_val, B_val);

            inVal = buildSimulationInput(cfg, cfg.modelName, cfg.validationStopTime, K_val, B_val);
            simOutVal = sim(inVal);
            logsVal = simOutVal.logsout;

            xM = getLogged(logsVal, cfg.signalNames.xM);
            xS = getLogged(logsVal, cfg.signalNames.xS);
            fcM = getLoggedSafe(logsVal, cfg.signalNames.fcM);
            fcS = getLoggedSafe(logsVal, cfg.signalNames.fcS);

            t = xM.Time(:);
            xM_data = squeeze(xM.Data(:));
            xS_data = squeeze(xS.Data(:));
            e_data  = xM_data - xS_data;

            metrics.maxAbsX = max(abs([xM_data; xS_data]));
            xEnv = max(abs([xM_data, xS_data]), [], 2);
            metrics.envRatioX = envelopeRatio(t, xEnv, cfg.classification.minWindowSec);
            metrics.rmsErr = rms(e_data);
            metrics.maxAbsF = maxAbsOptional(fcM, fcS);
            metrics.classCode = classifyRun(metrics, cfg.classification);
            metrics.classLabel = classCodeToLabel(metrics.classCode);

            % Plot validation response
            fig = figure('Name', sprintf('%s | %s | K=%.2f | B=%.2f', ...
                cfg.plotTitle, ptLabel, K_val, B_val), 'Color', 'w');

            tiledlayout(1,3, 'Padding','compact', 'TileSpacing','compact');

            nexttile;
            plot(t, xM_data, 'LineWidth', 1.2); hold on;
            plot(t, xS_data, 'LineWidth', 1.2);
            grid on;
            xlabel('Time [s]');
            ylabel('Position [m]');
            title(sprintf('%s positions', ptLabel), 'Interpreter', 'none');
            legend('x_M','x_S','Location','best');

            nexttile;
            plot(t, e_data, 'LineWidth', 1.2);
            grid on;
            xlabel('Time [s]');
            ylabel('e = x_M - x_S [m]');
            title(sprintf('envRatio = %.3f | %s', metrics.envRatioX, metrics.classLabel));

            nexttile;
            if ~isempty(fcM) && ~isempty(fcS)
                plot(fcM.Time(:), squeeze(fcM.Data(:)), 'LineWidth', 1.2); hold on;
                plot(fcS.Time(:), squeeze(fcS.Data(:)), 'LineWidth', 1.2);
                grid on;
                xlabel('Time [s]');
                ylabel('Force [N]');
                title(sprintf('max|F| = %.3g', metrics.maxAbsF));
                legend('F_{c,M}','F_{c,S}','Location','best');
            else
                text(0.1,0.5,'No force signals logged', 'FontSize', 11);
                axis off;
            end

            sgtitle(sprintf('%s | %s | K = %.2f N/m | B = %.2f Ns/m', ...
                cfg.plotTitle, ptLabel, K_val, B_val), 'Interpreter', 'none');

            if cfg.saveValidationFigures
                exportgraphics(fig, fullfile(outDir, sprintf('%s_%s_K%.0f_B%.2f.png', ...
                    cfg.resultPrefix, ptLabel, K_val, B_val)), 'Resolution', 180);
            end

            validationRows(end+1,:) = {ptLabel, K_val, B_val, metrics.maxAbsX, ...
                metrics.envRatioX, metrics.rmsErr, metrics.maxAbsF, char(metrics.classLabel)}; %#ok<AGROW>
        end

        validationSummary = cell2table(validationRows, ...
            'VariableNames', {'pointType','K','B','maxAbsX','envRatioX','rmsErr','maxAbsF','classLabel'});

        assignin('base', ['selectedValidationPoints_' cfg.resultPrefix], selectedPoints);
        assignin('base', ['validationSummary_' cfg.resultPrefix], validationSummary);

        fprintf('\nAutomatic validation complete.\n');
        fprintf('Exported to workspace:\n');
        fprintf('  selectedValidationPoints_%s\n', cfg.resultPrefix);
        fprintf('  validationSummary_%s\n', cfg.resultPrefix);

        if cfg.saveValidationFigures
            fprintf('Saved validation figures in:\n%s\n', outDir);
        end

        disp(validationSummary);
    else
        fprintf('No representative candidates could be selected automatically.\n');
    end
end

totalElapsed = toc(tStartSweep);
fprintf('\nSweep complete. Variables exported to base workspace:\n');
fprintf('  resultMap_%s, summaryTable_%s, cfgSweep_%s\n', cfg.resultPrefix, cfg.resultPrefix, cfg.resultPrefix);
fprintf('Total sweep time: %.2f minutes (%.2f hours)\n', totalElapsed/60, totalElapsed/3600);

%% ============================================================
%  LOCAL FUNCTIONS
%  ============================================================

function in = buildSimulationInput(cfg, modelName, stopTimeValue, K_val, B_val)
    in = Simulink.SimulationInput(modelName);
    in = in.setModelParameter('StopTime', stopTimeValue);

    % Controller gains
    in = in.setVariable('Kc', K_val);
    in = in.setVariable('Bc', B_val);
    in = in.setVariable('Td', cfg.Td);

    % Initial conditions
    in = in.setVariable('xM0', cfg.xM0);
    in = in.setVariable('vM0', cfg.vM0);
    in = in.setVariable('xS0', cfg.xS0);
    in = in.setVariable('vS0', cfg.vS0);

    % Optional aliases
    in = in.setVariable('x_m',     cfg.xM0);
    in = in.setVariable('x_m_dot', cfg.vM0);
    in = in.setVariable('x_s',     cfg.xS0);
    in = in.setVariable('x_s_dot', cfg.vS0);

    % Plant parameters
    in = in.setVariable('I_eq', cfg.I_eq);
    in = in.setVariable('b_eq', cfg.b_eq);
    in = in.setVariable('k_eq', cfg.k_eq);
    in = in.setVariable('r_s',  cfg.r_s);
    in = in.setVariable('r_h',  cfg.r_h);
    in = in.setVariable('r_m',  cfg.r_m);
    in = in.setVariable('m_lin', cfg.m_lin);
    in = in.setVariable('b_lin', cfg.b_lin);
    in = in.setVariable('k_lin', cfg.k_lin);

    % Stage 2-only variables (harmless for stage1 if unused)
    in = in.setVariable('Ts',    cfg.Ts);
    in = in.setVariable('a',     cfg.a);
end

function selectedPoints = selectValidationCandidates(summaryTable, th)
    selectedPoints = table();

    % Keep only finite rows
    valid = isfinite(summaryTable.K) & isfinite(summaryTable.B) & ...
            isfinite(summaryTable.envRatioX) & isfinite(summaryTable.classCode);
    T = summaryTable(valid,:);

    if isempty(T)
        return;
    end

    chosen = {};

    % 1) Decaying candidate: most comfortably decaying -> minimum envRatioX among decaying
    Ts = T(T.classCode == 0,:);
    if ~isempty(Ts)
        [~, idx] = min(Ts.envRatioX);
        chosen(end+1,:) = {'decaying_candidate', Ts.K(idx), Ts.B(idx), Ts.classCode(idx)}; %#ok<AGROW>
    end

    % 2) Near-boundary candidate:
    %    Prefer near-critical point with envRatioX closest to envGrowNear.
    Tn = T(T.classCode == 1,:);
    if ~isempty(Tn)
        [~, idx] = min(abs(Tn.envRatioX - th.envGrowNear));
        chosen(end+1,:) = {'near_boundary_candidate', Tn.K(idx), Tn.B(idx), Tn.classCode(idx)}; %#ok<AGROW>
    else
        % fallback: decaying point closest below threshold OR divergent point closest above threshold
        below = Ts;
        above = T(T.classCode == 2,:);
        bestRow = [];
        bestType = '';

        if ~isempty(below)
            [d1, i1] = min(abs(below.envRatioX - th.envGrowNear));
            bestRow = below(i1,:);
            bestDist = d1;
            bestType = 'near_boundary_candidate';
        else
            bestDist = inf;
        end

        if ~isempty(above)
            [d2, i2] = min(abs(above.envRatioX - th.envGrowDiv));
            if d2 < bestDist
                bestRow = above(i2,:);
                bestType = 'near_boundary_candidate';
            end
        end

        if ~isempty(bestRow)
            chosen(end+1,:) = {bestType, bestRow.K(1), bestRow.B(1), bestRow.classCode(1)}; %#ok<AGROW>
        end
    end

    % 3) Divergent candidate: onset of divergence -> smallest envRatioX among divergent
    Tu = T(T.classCode == 2,:);
    if ~isempty(Tu)
        [~, idx] = min(Tu.envRatioX);
        chosen(end+1,:) = {'divergent_candidate', Tu.K(idx), Tu.B(idx), Tu.classCode(idx)}; %#ok<AGROW>
    end

    if isempty(chosen)
        return;
    end

    selectedPoints = cell2table(chosen, ...
        'VariableNames', {'pointType','K','B','classCode'});

    % Remove exact duplicates by K-B pair while preserving order
    [~, ia] = unique(selectedPoints(:,{'K','B'}), 'rows', 'stable');
    selectedPoints = selectedPoints(ia,:);
end

function sig = getLogged(logs, sigName)
    sig = logs.get(sigName);
    if isempty(sig)
        error('Signal "%s" not found in logsout. Enable Signal Logging with this exact name.', sigName);
    end
    sig = sig.Values;
end

function sig = getLoggedSafe(logs, sigName)
    try
        sig = getLogged(logs, sigName);
    catch
        sig = [];
    end
end

function m = maxAbsOptional(varargin)
    vals = [];
    for k = 1:nargin
        sig = varargin{k};
        if ~isempty(sig)
            vals = [vals; squeeze(sig.Data(:))]; %#ok<AGROW>
        end
    end
    if isempty(vals)
        m = NaN;
    else
        m = max(abs(vals));
    end
end

function ratio = envelopeRatio(t, y, minWindowSec)
    y = abs(y(:));
    t = t(:);
    if numel(t) < 10
        ratio = NaN;
        return;
    end

    T = t(end) - t(1);
    if T < 2*minWindowSec
        n = numel(y);
        idx1 = 1:max(5, floor(0.25*n));
        idx2 = max(1, floor(0.75*n)):n;
    else
        idx1 = t <= (t(1) + minWindowSec);
        idx2 = t >= (t(end) - minWindowSec);
    end

    e1 = max(y(idx1));
    e2 = max(y(idx2));

    if e1 < 1e-12
        ratio = e2;
    else
        ratio = e2 / e1;
    end
end

function classCode = classifyRun(metrics, th)
    % 0 decaying, 1 near-critical, 2 divergent
    if any(~isfinite([metrics.maxAbsX, metrics.envRatioX]))
        classCode = 2;
        return;
    end

    if metrics.maxAbsX > th.maxAbsX
        classCode = 2;
        return;
    end

    if ~isnan(metrics.maxAbsF) && metrics.maxAbsF > th.maxAbsF
        classCode = 2;
        return;
    end

    if metrics.envRatioX > th.envGrowDiv
        classCode = 2;
    elseif metrics.envRatioX > th.envGrowNear
        classCode = 1;
    else
        classCode = 0;
    end
end



function overlayCellGrid(xVals, yVals, gridColor, gridAlpha, gridLineWidth)
    xVals = xVals(:).';
    yVals = yVals(:).';

    if numel(xVals) < 2 || numel(yVals) < 2
        return;
    end

    xEdges = centersToEdges(xVals);
    yEdges = centersToEdges(yVals);

    xMin = xEdges(1);
    xMax = xEdges(end);
    yMin = yEdges(1);
    yMax = yEdges(end);

    for k = 1:numel(xEdges)
        plot([xEdges(k) xEdges(k)], [yMin yMax], '-', ...
            'Color', [gridColor gridAlpha], 'LineWidth', gridLineWidth, ...
            'HandleVisibility', 'off');
    end

    for k = 1:numel(yEdges)
        plot([xMin xMax], [yEdges(k) yEdges(k)], '-', ...
            'Color', [gridColor gridAlpha], 'LineWidth', gridLineWidth, ...
            'HandleVisibility', 'off');
    end
end

function edges = centersToEdges(vals)
    vals = vals(:).';

    if numel(vals) == 1
        d = 1.0;
        edges = [vals(1)-d/2, vals(1)+d/2];
        return;
    end

    mids = 0.5 * (vals(1:end-1) + vals(2:end));
    leftEdge = vals(1) - 0.5 * (vals(2) - vals(1));
    rightEdge = vals(end) + 0.5 * (vals(end) - vals(end-1));
    edges = [leftEdge, mids, rightEdge];
end

function label = classCodeToLabel(code)
    switch code
        case 0
            label = 'decaying';
        case 1
            label = 'near-critical';
        otherwise
            label = 'divergent';
    end
end