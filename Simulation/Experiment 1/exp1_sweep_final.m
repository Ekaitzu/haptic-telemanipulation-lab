%% exp1_kb_sweep_auto_validate_passivity_FIXED_grid.m
% EXP1 K-B sweep with automatic representative-point validation.
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
% EXP1 interpretation for THIS implementation:
%   - Sweep the LOCAL rendering gains Kc and Bc
%   - Classification uses EXP1-appropriate metrics:
%       * envelope ratio      -> main response discriminator
%       * settling time       -> near-critical discriminator
%       * energy generation   -> passivity-inspired sanity check
%
% IMPORTANT:
%   1) Set cfg.modelKind = 'stage1' or 'stage2'
%   2) Check cfg.modelName and cfg.signalNames below to match your model
%   3) This version assumes your local impedance controller uses Kc and Bc
%      in BOTH stages, as discussed.

clear; clc;

%% ============================================================
%  USER CONFIGURATION
%  ============================================================

cfg.modelKind = 'stage2';   % 'stage1' or 'stage2'

% Simulation horizon: 3 s ensures T_sim >= 2*W for the envelope ratio
% and captures several oscillation cycles even at the lowest natural
% frequencies in the swept range (slowest period ~ 0.5 s at K ~ 0).
cfg.stopTime  = '3';
cfg.useFastRestart = true;

% Automatic validation after sweep
cfg.autoValidate = true;
cfg.validationStopTime = '3';
cfg.saveValidationFigures = true;
cfg.validationFigureFolder = 'exp1_auto_validation_figures';

% Automatic figure saving
cfg.showCellGrid  = true;
cfg.gridColor     = [0.55 0.55 0.55];
cfg.gridAlpha     = 0.7;
cfg.gridLineWidth = 0.6;

cfg.saveMapFigure = true;
cfg.mapFigureFolder = 'saved_map_figures';
cfg.mapFigureResolution = 300;
% -------------------------
% EXP1 local-rendering sweep
% -------------------------
cfg.K_vals = linspace(0, 400, 150);     % [N/m]
cfg.B_vals = linspace(0, 20, 150);   % [Ns/m]

% Initial-condition excitation
cfg.x0 = 0.020;      % [m]
cfg.v0 = 0.0;        % [m/s]

% Local environment parameters
cfg.RestPos = 0.0;   % [m]

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
xRef = max(abs([cfg.x0 - cfg.RestPos, 1e-6]));
cfg.classification.maxAbsX = 20*xRef;

% Envelope ratio thresholds
cfg.classification.envGrowDiv  = 1.05;
cfg.classification.envGrowNear = 0.30;

% Settling time thresholds [s]
cfg.classification.settleBandFraction = 0.02;  % 2% of initial displacement
cfg.classification.minSettleBand = 1e-4;       % [m]
cfg.classification.settleDecay = 0.35 * str2double(cfg.stopTime);
cfg.classification.settleNear  = 0.85 * str2double(cfg.stopTime);

% Energy-generation threshold [J]
cfg.classification.energyTol = 1e-6;

% Envelope window length: W = 1.0 s so that T_sim (3 s) >= 2*W (2 s),
% ensuring the envelope ratio uses its intended first/last windows
% rather than the quarter-based fallback.  Each 1 s window captures
% at least two full cycles of the slowest expected oscillation.
cfg.classification.minWindowSec = 1.0;

% Optional map styling
cfg.showCellGrid = true;

%% ============================================================
%  MODEL-SPECIFIC CONFIGURATION
%  ============================================================

switch lower(cfg.modelKind)
    case 'stage1'
        % EDIT if your actual file name differs
        cfg.modelName = 'exp1_local_rendering_stg1';

        % Logged signals
        cfg.signalNames.x = 'x_M';
        cfg.signalNames.f = 'F_c_M';   % actual local rendered force

        cfg.resultPrefix = 'exp1_stg1';
        cfg.plotTitle = 'EXP1 Stage 1';

    case 'stage2'
        % EDIT if your actual file name differs
        cfg.modelName = 'exp1_local_rendering_stg2';

        % Logged signals
        cfg.signalNames.x = 'x_M_k';
        cfg.signalNames.f = 'F_c_M_k'; % actual local rendered force

        cfg.resultPrefix = 'exp1_stg2';
        cfg.plotTitle = 'EXP1 Stage 2';

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

            xSig = getLogged(logs, cfg.signalNames.x);
            fSig = getLoggedSafe(logs, cfg.signalNames.f);

            t = xSig.Time(:);
            x = squeeze(xSig.Data(:));
            xDev = x - cfg.RestPos;

            if isempty(t) || numel(t) < 10
                error('Too few samples returned from simulation.');
            end
            if any(~isfinite(t)) || any(~isfinite(x))
                error('Non-finite values in logged position signal.');
            end

            metrics = computeExp1Metrics(t, xDev, fSig, cfg.classification);
            classCode = classifyRunExp1(metrics, cfg.classification);
            resultMap(iB, iK) = classCode;

            rowCounter = rowCounter + 1;
            summaryRows(rowCounter,:) = [K_val, B_val, metrics.maxAbsX, ...
                metrics.envRatioX, metrics.settlingTime, metrics.energyMax, ...
                metrics.energyFinal, classCode]; %#ok<SAGROW>

        catch ME
            warning('Simulation failed for K=%.3f, B=%.3f -> marked divergent.\n%s', ...
                K_val, B_val, ME.message);
            resultMap(iB, iK) = 2;
            rowCounter = rowCounter + 1;
            summaryRows(rowCounter,:) = [K_val, B_val, NaN, NaN, NaN, NaN, NaN, 2]; %#ok<SAGROW>
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
    {'K', 'B', 'maxAbsX', 'envRatioX', 'settlingTime', 'energyMax', 'energyFinal', 'classCode'});

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
title([cfg.plotTitle ': envelope ratio + settling time + energy check'], 'FontSize', 14);
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

%% ============================================================
%  AUTOMATIC REPRESENTATIVE-POINT VALIDATION
%  ============================================================

if cfg.autoValidate
    fprintf('\nSelecting automatic validation candidates...\n');
    selectedPoints = selectValidationCandidatesExp1(summaryTable, cfg.classification);

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

            xSig = getLogged(logsVal, cfg.signalNames.x);
            fSig = getLoggedSafe(logsVal, cfg.signalNames.f);

            t = xSig.Time(:);
            x = squeeze(xSig.Data(:));
            xDev = x - cfg.RestPos;

            metrics = computeExp1Metrics(t, xDev, fSig, cfg.classification);
            metrics.classCode = classifyRunExp1(metrics, cfg.classification);
            metrics.classLabel = classCodeToLabel(metrics.classCode);

            energyTrace = [];
            if ~isempty(fSig)
                energyTrace = computeEnergyTrace(t, x, fSig);
            end

            fig = figure('Name', sprintf('%s | %s | K=%.2f | B=%.2f', ...
                cfg.plotTitle, ptLabel, K_val, B_val), 'Color', 'w');

            tiledlayout(1,4, 'Padding','compact', 'TileSpacing','compact');

            nexttile;
            plot(t, x, 'LineWidth', 1.2);
            grid on;
            xlabel('Time [s]');
            ylabel('Position [m]');
            title(sprintf('%s position', ptLabel), 'Interpreter', 'none');
            legend('x','Location','best');

            nexttile;
            plot(t, xDev, 'LineWidth', 1.2);
            yline(metrics.settleBand, '--');
            yline(-metrics.settleBand, '--');
            grid on;
            xlabel('Time [s]');
            ylabel('x - RestPos [m]');
            title(sprintf('env = %.3f | T_s = %.3f s', metrics.envRatioX, metrics.settlingTime));

            nexttile;
            if ~isempty(fSig)
                plot(fSig.Time(:), squeeze(fSig.Data(:)), 'LineWidth', 1.2);
                grid on;
                xlabel('Time [s]');
                ylabel('Force [N]');
                title('Rendered force');
                legend('F','Location','best');
            else
                text(0.1,0.5,'No force signal logged', 'FontSize', 11);
                axis off;
            end

            nexttile;
            if ~isempty(energyTrace)
                plot(t, energyTrace, 'LineWidth', 1.2);
                yline(cfg.classification.energyTol, '--');
                grid on;
                xlabel('Time [s]');
                ylabel('Energy [J]');
                title(sprintf('E_{max}=%.3g | E_f=%.3g', metrics.energyMax, metrics.energyFinal));
            else
                text(0.1,0.5,'No energy trace (force missing)', 'FontSize', 11);
                axis off;
            end

            sgtitle(sprintf('%s | %s | K = %.2f N/m | B = %.2f Ns/m | %s', ...
                cfg.plotTitle, ptLabel, K_val, B_val, metrics.classLabel), 'Interpreter', 'none');

            if cfg.saveValidationFigures
                exportgraphics(fig, fullfile(outDir, sprintf('%s_%s_K%.0f_B%.2f.png', ...
                    cfg.resultPrefix, ptLabel, K_val, B_val)), 'Resolution', 180);
            end

            validationRows(end+1,:) = {ptLabel, K_val, B_val, metrics.maxAbsX, ...
                metrics.envRatioX, metrics.settlingTime, metrics.energyMax, ...
                metrics.energyFinal, char(metrics.classLabel)}; %#ok<AGROW>
        end

        validationSummary = cell2table(validationRows, ...
            'VariableNames', {'pointType','K','B','maxAbsX','envRatioX','settlingTime','energyMax','energyFinal','classLabel'});

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

    % EXP1 local gains: in THIS implementation the controller uses Kc and Bc
    in = in.setVariable('Kc', K_val);
    in = in.setVariable('Bc', B_val);
    in = in.setVariable('RestPos', cfg.RestPos);

    % Initial conditions
    in = in.setVariable('xM0', cfg.x0);
    in = in.setVariable('vM0', cfg.v0);

    % Optional aliases used in some models
    in = in.setVariable('x_m',     cfg.x0);
    in = in.setVariable('x_m_dot', cfg.v0);
    in = in.setVariable('x0',      cfg.x0);
    in = in.setVariable('v0',      cfg.v0);

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

function metrics = computeExp1Metrics(t, xDev, fSig, th)
    metrics = struct();

    metrics.maxAbsX = max(abs(xDev));
    metrics.envRatioX = envelopeRatio(t, xDev, th.minWindowSec);

    x0mag = max(abs(xDev(1)), 1e-12);
    metrics.settleBand = max(th.minSettleBand, th.settleBandFraction * x0mag);
    metrics.settlingTime = settlingTimeWithinBand(t, xDev, metrics.settleBand);

    if ~isempty(fSig)
        energyTrace = computeEnergyTrace(t, xDev, fSig);
        metrics.energyMax = max(energyTrace);
        metrics.energyFinal = energyTrace(end);
    else
        metrics.energyMax = NaN;
        metrics.energyFinal = NaN;
    end
end

function E = computeEnergyTrace(t, x, fSig)
    % Passivity-inspired discrete work integral:
    %   E[k] = sum F[i] * (x[i]-x[i-1])
    % Force is interpolated onto x time if needed.
    fTime = fSig.Time(:);
    fData = squeeze(fSig.Data(:));

    if numel(fTime) ~= numel(t) || any(abs(fTime - t) > 1e-12)
        fInterp = interp1(fTime, fData, t, 'previous', 'extrap');
    else
        fInterp = fData;
    end

    dx = [0; diff(x(:))];
    E = cumsum(fInterp(:) .* dx);
end

function ts = settlingTimeWithinBand(t, y, band)
    ts = Inf;
    y = abs(y(:));
    t = t(:);

    idxLastOutside = find(y > band, 1, 'last');
    if isempty(idxLastOutside)
        ts = t(1);
        return;
    end
    if idxLastOutside < numel(t)
        ts = t(idxLastOutside + 1);
    end
end

function selectedPoints = selectValidationCandidatesExp1(summaryTable, th)
    selectedPoints = table();

    valid = isfinite(summaryTable.K) & isfinite(summaryTable.B) & ...
            isfinite(summaryTable.classCode) & isfinite(summaryTable.envRatioX);
    T = summaryTable(valid,:);
    if isempty(T)
        return;
    end

    chosen = {};

    % Decaying candidate: comfortably decaying = low env ratio + short settling
    Ts = T(T.classCode == 0,:);
    if ~isempty(Ts)
        scoreDecay = Ts.envRatioX + 0.05 * normalizeFinite(Ts.settlingTime);
        [~, idx] = min(scoreDecay);
        chosen(end+1,:) = {'decaying_candidate', Ts.K(idx), Ts.B(idx), Ts.classCode(idx)}; %#ok<AGROW>
    end

    % Near-boundary candidate: prefer near-critical with env ratio near threshold
    Tn = T(T.classCode == 1,:);
    if ~isempty(Tn)
        scoreNear = abs(Tn.envRatioX - th.envGrowNear) + 0.05 * normalizeFinite(Tn.settlingTime);
        [~, idx] = min(scoreNear);
        chosen(end+1,:) = {'near_boundary_candidate', Tn.K(idx), Tn.B(idx), Tn.classCode(idx)}; %#ok<AGROW>
    else
        below = Ts;
        above = T(T.classCode == 2,:);
        bestRow = [];
        bestDist = inf;

        if ~isempty(below)
            scoreBelow = abs(below.envRatioX - th.envGrowNear) + 0.05 * normalizeFinite(below.settlingTime);
            [d1, i1] = min(scoreBelow);
            bestRow = below(i1,:);
            bestDist = d1;
        end

        if ~isempty(above)
            scoreAbove = abs(above.envRatioX - th.envGrowDiv);
            [d2, i2] = min(scoreAbove);
            if d2 < bestDist
                bestRow = above(i2,:);
            end
        end

        if ~isempty(bestRow)
            chosen(end+1,:) = {'near_boundary_candidate', bestRow.K(1), bestRow.B(1), bestRow.classCode(1)}; %#ok<AGROW>
        end
    end

    % Divergent candidate: smallest divergent score above threshold
    Tu = T(T.classCode == 2,:);
    if ~isempty(Tu)
        scoreDiv = abs(Tu.envRatioX - th.envGrowDiv);
        [~, idx] = min(scoreDiv);
        chosen(end+1,:) = {'divergent_candidate', Tu.K(idx), Tu.B(idx), Tu.classCode(idx)}; %#ok<AGROW>
    end

    if isempty(chosen)
        return;
    end

    selectedPoints = cell2table(chosen, 'VariableNames', {'pointType','K','B','classCode'});
    [~, ia] = unique(selectedPoints(:,{'K','B'}), 'rows', 'stable');
    selectedPoints = selectedPoints(ia,:);
end

function z = normalizeFinite(v)
    v = v(:);
    if isempty(v) || all(~isfinite(v))
        z = zeros(size(v));
        return;
    end
    vf = v(isfinite(v));
    vmin = min(vf);
    vmax = max(vf);
    if abs(vmax - vmin) < 1e-12
        z = zeros(size(v));
    else
        z = (v - vmin) ./ (vmax - vmin);
    end
    z(~isfinite(z)) = 0;
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

function classCode = classifyRunExp1(metrics, th)
    % 0 decaying, 1 near-critical, 2 divergent

    if any(~isfinite([metrics.maxAbsX, metrics.envRatioX]))
        classCode = 2;
        return;
    end

    if metrics.maxAbsX > th.maxAbsX
        classCode = 2;
        return;
    end

    % Clearly divergent if the envelope grows
    if metrics.envRatioX > th.envGrowDiv
        classCode = 2;
        return;
    end

    % Passivity-inspired energy check:
    % only use it as an additional sanity check, never as the sole
    % discriminator of a decaying point
    if ~isnan(metrics.energyMax) && metrics.energyMax > th.energyTol && metrics.envRatioX >= 1.0
        classCode = 2;
        return;
    end

    % Decaying if clearly decaying and settles in reasonable time
    if metrics.envRatioX < th.envGrowNear && metrics.settlingTime <= th.settleDecay
        classCode = 0;
        return;
    end

    % Near-critical if finite, decays weakly / settles slowly
    if metrics.settlingTime <= th.settleNear || metrics.envRatioX <= th.envGrowDiv
        classCode = 1;
    else
        classCode = 2;
    end
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

function overlayCellGrid(xVals, yVals, gridColor, gridAlpha, gridLineWidth)
    %#ok<INUSD>  % gridAlpha kept for compatibility, not used directly

    xEdges = centersToEdges(xVals(:));
    yEdges = centersToEdges(yVals(:));

    xl = [xEdges(1), xEdges(end)];
    yl = [yEdges(1), yEdges(end)];

    for k = 1:numel(xEdges)
        plot([xEdges(k) xEdges(k)], yl, '-', ...
            'Color', gridColor, ...
            'LineWidth', gridLineWidth, ...
            'HandleVisibility', 'off');
    end

    for k = 1:numel(yEdges)
        plot(xl, [yEdges(k) yEdges(k)], '-', ...
            'Color', gridColor, ...
            'LineWidth', gridLineWidth, ...
            'HandleVisibility', 'off');
    end
end

function edges = centersToEdges(c)
    if numel(c) == 1
        dc = 0.5;
        edges = [c - dc; c + dc];
        return;
    end

    dc = diff(c);
    edges = zeros(numel(c)+1,1);
    edges(2:end-1) = 0.5*(c(1:end-1) + c(2:end));
    edges(1) = c(1) - 0.5*dc(1);
    edges(end) = c(end) + 0.5*dc(end);
end