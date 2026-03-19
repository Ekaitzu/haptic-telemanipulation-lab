%% sweep_exp3_KB_Td_stg2_clean.m
%
% K-B-Td Parameter Space Sweep for EXP3 Stage 2
%
% Structure and classification logic identical to exp2_sweep_final and
% sweep_exp3_KB_Td_stg1_clean:
%   - Classification based purely on maxAbsX, maxAbsF, and envRatioX
%     (rmsErr logged but not used in classification)
%   - Simulation horizon: 12 s (Tsim >= 2*W = 4 s -> proper envelope windows)
%   - minWindowSec: 2.0 s
%
% Speed strategy (same as stg1_clean):
%   1) Short pre-check with its own small timeout
%   2) Only points classified decaying in pre-check go to full simulation
%   3) Each delay slice uses ROLLING pruning: only tests points decaying at
%      the PREVIOUS delay slice (not just at Td=0). This avoids wasting
%      time on combinations already known to diverge at lower delays.
%   4) Validation disabled by default for heavy sweeps
%   5) Timeout / solver failures are safely classified as divergent

clear; clc;

%% =========================================================
%  USER CONFIGURATION
%% =========================================================

cfg.modelName = 'exp3_bilateral_delay_stg2';

% --- K grid: stiffness gain [N/m] — X axis ---
cfg.K_min  = 0;
cfg.K_max  = 150;
cfg.K_npts = 50;
cfg.KList  = linspace(cfg.K_min, cfg.K_max, cfg.K_npts);

% --- B grid: damping gain [Ns/m] — Y axis ---
cfg.B_min  = 0;
cfg.B_max  = 4.0;
cfg.B_npts = 50;
cfg.BList  = linspace(cfg.B_min, cfg.B_max, cfg.B_npts);

% --- Delay sweep [s] ---
cfg.delayList = [0, 0.005, 0.010, 0.020, 0.050];

% --- Simulation timing ---
% Full horizon 12 s: ensures Tsim (12 s) >= 2*W (4 s) so proper
% early/late envelope windows are always used (no quarter-sample fallback).
cfg.stopTimeFull      = '6';
cfg.stopTimePre       = '2.0';
cfg.simTimeoutSecFull = 120;   % wall-clock kill for full runs
cfg.simTimeoutSecPre  = 15;    % wall-clock kill for pre-check runs
cfg.usePreCheck       = true;

% --- Validation (disable for heavy sweeps; enable later for final runs) ---
cfg.autoValidate        = false;
cfg.validationStopTime  = '12';
cfg.saveValidationFigs  = false;
cfg.validationFolder    = 'exp3_validation_figures_stg2';

% --- Plant parameters ---
cfg.I_eq  = 1.58e-6; cfg.b_eq = 6.57e-6; cfg.k_eq = 2.9e-4;
cfg.r_s   = 0.075;   cfg.r_h  = 0.090;   cfg.r_m  = 0.010;
cfg.m_lin = cfg.I_eq * (cfg.r_s / (cfg.r_h * cfg.r_m))^2;
cfg.b_lin = cfg.b_eq * (cfg.r_s / (cfg.r_h * cfg.r_m))^2;
cfg.k_lin = cfg.k_eq * (cfg.r_s / (cfg.r_h * cfg.r_m))^2;

% --- Initial conditions ---
cfg.xM0 = 0.020; cfg.vM0 = 0.0;
cfg.xS0 = 0.0;   cfg.vS0 = 0.0;

% --- Stage 2 discrete params ---
cfg.Ts = 0.001;
cfg.a  = 0.92;

% --- Signal names (Stage 2) ---
cfg.signalNames.xM  = 'x_M_k';
cfg.signalNames.xS  = 'x_S_k';
cfg.signalNames.fcM = 'F_c_M_k';
cfg.signalNames.fcS = 'F_c_S_k';

% --- Classification thresholds (identical to exp2_sweep_final) ---
xRef = max(abs([cfg.xM0, cfg.xS0, 1e-6]));
cfg.classification.maxAbsX      = 20 * xRef;
cfg.classification.maxAbsF      = 1e3;
cfg.classification.envGrowDiv   = 1.05;
cfg.classification.envGrowNear  = 0.30;
% Envelope window: 2.0 s so that Tsim (12 s) >= 2*W (4 s)
cfg.classification.minWindowSec = 2.0;
cfg.classification.minSamples   = 20;

% --- Output ---
cfg.saveFigures     = true;
cfg.saveIncremental = true;
cfg.figureFolder    = 'exp3_paramspace_figures_stg2';
cfg.resultFile      = 'exp3_paramspace_results_stg2.mat';

% --- Map cell-grid overlay ---
cfg.showCellGrid  = true;
cfg.gridColor     = [0.55 0.55 0.55];
cfg.gridAlpha     = 0.7;
cfg.gridLineWidth = 0.6;

% --- Parallel pool ---
cfg.nWorkers = 3;   % adjust to your machine; 2 or 3 is usually reasonable

% --- Suppress known harmless Td=0 warning ---
warning('off', 'Simulink:blocks:TDelayTimeTooSmall');
warning('off', 'Simulink:Engine:TDelayTimeTooSmall');

%% =========================================================
%  SETUP
%% =========================================================

nK  = numel(cfg.KList);
nB  = numel(cfg.BList);
nTd = numel(cfg.delayList);

% resultMap(ki, bi, tdi): 0=decaying 1=near-critical 2=divergent 3=failed/timeout
resultMap = 3 * ones(nK, nB, nTd);

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir); scriptDir = pwd; end

if cfg.saveFigures
    outDir = fullfile(scriptDir, cfg.figureFolder);
    if ~exist(outDir, 'dir'); mkdir(outDir); end
end

fprintf('=== EXP3 Stage 2 K-B-Td Parameter Space Sweep ===\n');
fprintf('K: %d pts  [%.1f .. %.1f] N/m\n',  nK, cfg.K_min, cfg.K_max);
fprintf('B: %d pts  [%.2f .. %.2f] Ns/m\n', nB, cfg.B_min, cfg.B_max);
fprintf('Delays: %d values\n', nTd);
fprintf('Grid: %d x %d x %d = %d max runs\n', nK, nB, nTd, nK*nB*nTd);
fprintf('Pre-check stop time: %s s | timeout: %d s\n', cfg.stopTimePre, cfg.simTimeoutSecPre);
fprintf('Full-run  stop time: %s s | timeout: %d s\n\n', cfg.stopTimeFull, cfg.simTimeoutSecFull);

delete(gcp('nocreate'));
fprintf('Starting parallel pool (%d workers)...\n', cfg.nWorkers);
parpool('local', cfg.nWorkers);
fprintf('Pool ready.\n\n');

targetStopFull = str2double(cfg.stopTimeFull);
targetStopPre  = str2double(cfg.stopTimePre);

%% =========================================================
%  PHASE 1: K-B sweep at Td = 0
%% =========================================================

fprintf('--- PHASE 1: K-B sweep at Td = 0 ---\n');
tPhase1 = tic;

phase1TotalRuns         = nK * nB;
phase1CompletedRuns     = 0;
phase1LastPrintedPercent = -1;

loadOrReloadModel(cfg.modelName);

for ki = 1:nK
    for bi = 1:nB
        K_val = cfg.KList(ki);
        B_val = cfg.BList(bi);

        in  = buildSimInput(cfg, cfg.stopTimeFull, K_val, B_val, 0);
        res = runOneSafe(in, cfg, targetStopFull, cfg.simTimeoutSecFull);

        resultMap(ki, bi, 1) = res.classCode;

        phase1CompletedRuns = phase1CompletedRuns + 1;
        currentPercent = floor(100 * phase1CompletedRuns / phase1TotalRuns);

        if currentPercent > phase1LastPrintedPercent
            elapsedTime   = toc(tPhase1);
            estimatedTotal = elapsedTime * phase1TotalRuns / phase1CompletedRuns;
            remainingTime  = estimatedTotal - elapsedTime;

            fprintf(['Phase 1: %3d%% (%d/%d) | K=%.2f | B=%.3f | ' ...
                     'Elapsed: %.1f min | Remaining: %.1f min\n'], ...
                     currentPercent, phase1CompletedRuns, phase1TotalRuns, ...
                     K_val, B_val, elapsedTime/60, remainingTime/60);

            phase1LastPrintedPercent = currentPercent;
        end
    end
end

nDecayingBase = sum(resultMap(:,:,1) == 0, 'all');

fprintf('\nPhase 1 complete in %.1f min.\n', toc(tPhase1)/60);
fprintf('%d / %d pairs decaying at Td=0.\n', nDecayingBase, nK*nB);
fprintf('Phase 2 uses rolling pruning: each slice tests only pairs decaying at the previous slice.\n\n');

plotSlice(cfg, resultMap(:,:,1), 1, outDir);

if cfg.saveIncremental
    save(fullfile(scriptDir, cfg.resultFile), 'resultMap', 'cfg');
    fprintf('Phase 1 saved.\n\n');
end

%% =========================================================
%  OPTIONAL VALIDATION CANDIDATES (selected from Td=0 map)
%% =========================================================

valPts = table();
valDir = '';   % defined here so it is always in scope

if cfg.autoValidate
    fprintf('--- Selecting validation candidates from Td=0 map ---\n');

    rows = [];
    for ki = 1:nK
        for bi = 1:nB
            rows(end+1,:) = [cfg.KList(ki), cfg.BList(bi), resultMap(ki,bi,1)]; %#ok<AGROW>
        end
    end
    sweepTable = array2table(rows, 'VariableNames', {'K','B','classCode'});
    valPts = selectValidationCandidates(sweepTable);

    if ~isempty(valPts)
        fprintf('Validation points (reused at every delay):\n');
        disp(valPts);
    else
        fprintf('Not enough class diversity to pick candidates — skipping validation.\n');
    end

    if cfg.saveValidationFigs
        valDir = fullfile(scriptDir, cfg.validationFolder);
        if ~exist(valDir, 'dir'); mkdir(valDir); end
    end

    if ~isempty(valPts)
        runValidationSlice(cfg, valPts, 0, valDir);
    end
end

%% =========================================================
%  PHASE 2: Delay sweep — rolling pruning per slice
%% =========================================================

fprintf('--- PHASE 2: Delay sweep (rolling pruning) ---\n');
tPhase2 = tic;

% Upper-bound estimate for progress reporting (will be <= actual runs)
phase2TotalRuns          = nDecayingBase * (nTd - 1);
phase2CompletedRuns      = 0;
phase2LastPrintedPercent = -1;

for tdi = 2:nTd
    Td_val = cfg.delayList(tdi);
    fprintf('\n[Td = %.4f s]\n', Td_val);

    % Rolling: carry only pairs that were DECAYING at the previous slice
    decayingPrev = (resultMap(:,:,tdi-1) == 0);

    % Pre-mark everything not decaying previously as divergent for this slice
    for ki = 1:nK
        for bi = 1:nB
            if ~decayingPrev(ki, bi)
                resultMap(ki, bi, tdi) = 2;
            end
        end
    end

    % Collect candidates from previous decaying slice
    pairsToTest = [];
    for ki = 1:nK
        for bi = 1:nB
            if decayingPrev(ki, bi)
                pairsToTest(end+1,:) = [ki, bi]; %#ok<SAGROW>
            end
        end
    end
    nPairsBeforePre = size(pairsToTest, 1);

    if nPairsBeforePre == 0
        fprintf('  No decaying points carried from previous delay. Skipping slice.\n');
        plotSlice(cfg, resultMap(:,:,tdi), tdi, outDir);
        if cfg.saveIncremental
            save(fullfile(scriptDir, cfg.resultFile), 'resultMap', 'cfg');
            fprintf('  Saved.\n');
        end
        continue;
    end

    % -------- pre-check --------
    if cfg.usePreCheck
        passedPre = true(nPairsBeforePre, 1);

        for p = 1:nPairsBeforePre
            ki = pairsToTest(p,1);
            bi = pairsToTest(p,2);

            in  = buildSimInput(cfg, cfg.stopTimePre, cfg.KList(ki), cfg.BList(bi), Td_val);
            res = runOneSafe(in, cfg, targetStopPre, cfg.simTimeoutSecPre);

            % Only clearly decaying points survive to full simulation
            if res.classCode ~= 0
                passedPre(p) = false;
                resultMap(ki, bi, tdi) = res.classCode;
            end
        end

        pairsToTest = pairsToTest(passedPre, :);
    end

    nPairs = size(pairsToTest, 1);

    fprintf('--- Delay case %d/%d: Td = %.4f s ---\n', tdi, nTd, Td_val);
    fprintf('Candidates from previous decaying slice: %d\n', nPairsBeforePre);
    fprintf('Candidates passing pre-check:            %d\n', nPairs);

    if nPairs == 0
        fprintf('  No points survived pre-check.\n');
        plotSlice(cfg, resultMap(:,:,tdi), tdi, outDir);
        if cfg.saveIncremental
            save(fullfile(scriptDir, cfg.resultFile), 'resultMap', 'cfg');
            fprintf('  Saved.\n');
        end
        if cfg.autoValidate && ~isempty(valPts)
            runValidationSlice(cfg, valPts, Td_val, valDir);
        end
        continue;
    end

    % -------- full run --------
    fprintf('  Full sim (%s s) on %d pairs...\n', cfg.stopTimeFull, nPairs);
    tSlice = tic;

    for p = 1:nPairs
        ki    = pairsToTest(p,1);
        bi    = pairsToTest(p,2);
        K_val = cfg.KList(ki);
        B_val = cfg.BList(bi);

        in  = buildSimInput(cfg, cfg.stopTimeFull, K_val, B_val, Td_val);
        res = runOneSafe(in, cfg, targetStopFull, cfg.simTimeoutSecFull);

        resultMap(ki, bi, tdi) = res.classCode;

        phase2CompletedRuns = phase2CompletedRuns + 1;
        currentPercent = floor(100 * phase2CompletedRuns / max(1, phase2TotalRuns));

        if currentPercent > phase2LastPrintedPercent
            elapsedTime    = toc(tPhase2);
            estimatedTotal = elapsedTime * phase2TotalRuns / max(1, phase2CompletedRuns);
            remainingTime  = estimatedTotal - elapsedTime;

            fprintf(['Phase 2: %3d%% (%d/%d) | Td=%.4f | K=%.2f | B=%.3f | ' ...
                     'Elapsed: %.1f min | Remaining: %.1f min\n'], ...
                     currentPercent, phase2CompletedRuns, phase2TotalRuns, ...
                     Td_val, K_val, B_val, elapsedTime/60, remainingTime/60);

            phase2LastPrintedPercent = currentPercent;
        end
    end

    fprintf('  Slice done in %.1f min.\n', toc(tSlice)/60);

    plotSlice(cfg, resultMap(:,:,tdi), tdi, outDir);

    if cfg.saveIncremental
        save(fullfile(scriptDir, cfg.resultFile), 'resultMap', 'cfg');
        fprintf('  Saved.\n');
    end

    if cfg.autoValidate && ~isempty(valPts)
        runValidationSlice(cfg, valPts, Td_val, valDir);
    end
end

fprintf('\nPhase 2 complete in %.1f min.\n', toc(tPhase2)/60);

%% =========================================================
%  FINAL PLOTS AND SAVE
%% =========================================================

plotAllSlices(cfg, resultMap, outDir);
plotBoundaryEvolution(cfg, resultMap, outDir);

save(fullfile(scriptDir, cfg.resultFile), 'resultMap', 'cfg');
fprintf('\nDone.\nResults : %s\nFigures : %s\n', cfg.resultFile, outDir);

delete(gcp('nocreate'));

%% =========================================================
%  LOCAL FUNCTIONS
%% =========================================================

function runValidationSlice(cfg, valPts, Td_val, valDir)
    fprintf('\n  [Validation at Td = %.4f s]\n', Td_val);
    for p = 1:height(valPts)
        ptLabel    = valPts.pointType{p};
        K_val      = valPts.K(p);
        B_val      = valPts.B(p);
        sweepClass = classCodeToLabel(valPts.classCode(p));

        fprintf('    %s  K=%.2f  B=%.3f  -->  ', ptLabel, K_val, B_val);

        in  = buildSimInput(cfg, cfg.validationStopTime, K_val, B_val, Td_val);
        res = runOneSafe(in, cfg, str2double(cfg.validationStopTime), cfg.simTimeoutSecFull);

        if res.classCode == 3 || isempty(res.metrics)
            fprintf('sim failed (%s)\n', res.failReason);
            continue;
        end

        valClass = classCodeToLabel(res.classCode);
        match    = strcmp(valClass, sweepClass);
        fprintf('%s  [%s]\n', valClass, ternary(match, 'MATCH', 'MISMATCH'));

        % Re-run to collect full signal data for plotting
        in2 = buildSimInput(cfg, cfg.validationStopTime, K_val, B_val, Td_val);
        try
            f = parfeval(@runSimWorker, 1, in2);
            [~, simOut] = fetchNext(f, cfg.simTimeoutSecFull);
            if isempty(simOut) || ischar(simOut) || isstring(simOut)
                fprintf('    Could not retrieve signals for plot.\n');
                continue;
            end

            logs    = simOut.logsout;
            xMsig   = logs.get(cfg.signalNames.xM); xMsig = xMsig.Values;
            xSsig   = logs.get(cfg.signalNames.xS); xSsig = xSsig.Values;
            t       = xMsig.Time(:);
            xM_data = squeeze(xMsig.Data(:));
            xS_data = squeeze(xSsig.Data(:));
            e_data  = xM_data - xS_data;
            fcM     = getLoggedSafe(logs, cfg.signalNames.fcM);
            fcS     = getLoggedSafe(logs, cfg.signalNames.fcS);

            fig = figure('Visible','off','Color','w','Position',[0 0 1100 380]);
            tl  = tiledlayout(fig, 1, 3, 'Padding','compact','TileSpacing','compact');

            ax1 = nexttile(tl);
            plot(ax1, t, xM_data, 'LineWidth', 1.4); hold(ax1,'on');
            plot(ax1, t, xS_data, 'LineWidth', 1.4);
            grid(ax1,'on');
            xlabel(ax1,'t [s]'); ylabel(ax1,'x [m]');
            legend(ax1,'x_M','x_S','Location','best');
            title(ax1, sprintf('Positions | %s', valClass), 'Interpreter','none');

            ax2 = nexttile(tl);
            plot(ax2, t, e_data, 'Color',[0.6 0.1 0.7],'LineWidth',1.4);
            grid(ax2,'on');
            xlabel(ax2,'t [s]'); ylabel(ax2,'e = x_M - x_S [m]');
            title(ax2, sprintf('envRatio = %.4f  |  %s', res.metrics.envRatioX, valClass));

            ax3 = nexttile(tl);
            if ~isempty(fcM) && ~isempty(fcS)
                plot(ax3, fcM.Time(:), squeeze(fcM.Data(:)), 'LineWidth',1.4); hold(ax3,'on');
                plot(ax3, fcS.Time(:), squeeze(fcS.Data(:)), 'LineWidth',1.4);
                legend(ax3,'F_{c,M}','F_{c,S}','Location','best');
                ylabel(ax3,'F [N]');
                title(ax3, sprintf('max|F| = %.3g N', res.metrics.maxAbsF));
            else
                text(ax3, 0.1, 0.5, 'No force signals logged', 'FontSize', 11);
                axis(ax3,'off');
            end
            xlabel(ax3,'t [s]'); grid(ax3,'on');

            sgtitle(fig, sprintf('EXP3 Stage 2 | %s | K=%.2f N/m  B=%.3f Ns/m  T_d=%.4f s', ...
                ptLabel, K_val, B_val, Td_val), 'Interpreter','none','FontSize',11);

            if cfg.saveValidationFigs && ~isempty(valDir)
                fname = fullfile(valDir, sprintf('val_%s_K%.1f_B%.3f_Td%.4f.png', ...
                    ptLabel, K_val, B_val, Td_val));
                exportgraphics(fig, fname, 'Resolution', 180);
            end
            close(fig);

        catch ME2
            fprintf('    Plot failed: %s\n', ME2.message);
        end
    end
    fprintf('\n');
end

function res = runOneSafe(in, cfg, targetStopTime, thisTimeoutSec)
    res.classCode  = 3;
    res.failReason = '';
    res.metrics    = [];

    try
        f = parfeval(@runSimWorker, 1, in);
        [~, simOut] = fetchNext(f, thisTimeoutSec);

        if isempty(simOut)
            cancel(f);
            res.classCode  = 2;
            res.failReason = sprintf('Timeout >%ds', thisTimeoutSec);
            fprintf('[TIMEOUT] ');
            return;
        end

        if ischar(simOut) || isstring(simOut)
            res.classCode  = 2;
            res.failReason = char(simOut);
            return;
        end

        errMsg = '';
        try
            errMsg = simOut.ErrorMessage;
        catch
        end
        if ~isempty(errMsg)
            res.classCode  = 2;
            res.failReason = char(errMsg);
            return;
        end

        logs    = simOut.logsout;
        xMsig   = logs.get(cfg.signalNames.xM); xMsig = xMsig.Values;
        xSsig   = logs.get(cfg.signalNames.xS); xSsig = xSsig.Values;
        t       = xMsig.Time(:);
        xM_data = squeeze(xMsig.Data(:));
        xS_data = squeeze(xSsig.Data(:));

        if numel(t) < cfg.classification.minSamples || t(end) < (targetStopTime - 1e-3)
            res.classCode  = 2;
            res.failReason = 'Early stop / too few samples';
            return;
        end

        if any(~isfinite(xM_data)) || any(~isfinite(xS_data))
            res.classCode  = 2;
            res.failReason = 'Non-finite values';
            return;
        end

        xEnv              = max(abs([xM_data, xS_data]), [], 2);
        metrics.maxAbsX   = max(xEnv);
        metrics.envRatioX = envelopeRatio(t, xEnv, cfg.classification.minWindowSec);
        metrics.rmsErr    = rms(xM_data - xS_data);   % logged only, not used for classification
        metrics.maxAbsF   = NaN;

        try
            fcM = logs.get(cfg.signalNames.fcM); fcM = fcM.Values;
            fcS = logs.get(cfg.signalNames.fcS); fcS = fcS.Values;
            fVals = [squeeze(fcM.Data(:)); squeeze(fcS.Data(:))];
            metrics.maxAbsF = max(abs(fVals));
        catch
        end

        res.metrics   = metrics;
        res.classCode = classifyRun(metrics, cfg.classification);

    catch ME
        res.classCode  = 2;
        res.failReason = ME.message;
    end
end

function simOut = runSimWorker(in)
    try
        warning('off','Simulink:blocks:TDelayTimeTooSmall');
        warning('off','Simulink:Engine:TDelayTimeTooSmall');
        simOut = sim(in);
    catch ME
        simOut = ME.message;
    end
end

function loadOrReloadModel(modelName)
    try
        if bdIsLoaded(modelName)
            try
                set_param(modelName,'FastRestart','off');
            catch
            end
            bdclose(modelName);
        end
    catch
    end
    load_system(modelName);
end

function in = buildSimInput(cfg, stopTime, K_val, B_val, Td_val)
    in = Simulink.SimulationInput(cfg.modelName);
    in = in.setModelParameter('StopTime', stopTime);

    % Controller gains
    in = in.setVariable('Kc',      K_val);
    in = in.setVariable('Bc',      B_val);
    in = in.setVariable('Td',      Td_val);

    % Initial conditions
    in = in.setVariable('xM0',     cfg.xM0);
    in = in.setVariable('vM0',     cfg.vM0);
    in = in.setVariable('xS0',     cfg.xS0);
    in = in.setVariable('vS0',     cfg.vS0);

    % Optional aliases
    in = in.setVariable('x_m',     cfg.xM0);
    in = in.setVariable('x_m_dot', cfg.vM0);
    in = in.setVariable('x_s',     cfg.xS0);
    in = in.setVariable('x_s_dot', cfg.vS0);

    % Plant parameters
    in = in.setVariable('I_eq',    cfg.I_eq);
    in = in.setVariable('b_eq',    cfg.b_eq);
    in = in.setVariable('k_eq',    cfg.k_eq);
    in = in.setVariable('r_s',     cfg.r_s);
    in = in.setVariable('r_h',     cfg.r_h);
    in = in.setVariable('r_m',     cfg.r_m);
    in = in.setVariable('m_lin',   cfg.m_lin);
    in = in.setVariable('b_lin',   cfg.b_lin);
    in = in.setVariable('k_lin',   cfg.k_lin);

    % Stage 2-only variables
    in = in.setVariable('Ts',      cfg.Ts);
    in = in.setVariable('a',       cfg.a);
end

function classCode = classifyRun(metrics, th)
    % Identical logic to exp2_sweep_final classifyRun:
    % 1. Non-finite guard
    if ~isfinite(metrics.maxAbsX) || ~isfinite(metrics.envRatioX)
        classCode = 2;
        return;
    end

    % 2. Displacement bound
    if metrics.maxAbsX > th.maxAbsX
        classCode = 2;
        return;
    end

    % 3. Force bound (skipped if force not available)
    if ~isnan(metrics.maxAbsF) && metrics.maxAbsF > th.maxAbsF
        classCode = 2;
        return;
    end

    % 4. Envelope ratio — divergent
    if metrics.envRatioX > th.envGrowDiv
        classCode = 2;
        return;
    end

    % 5. Envelope ratio — near-critical
    if metrics.envRatioX > th.envGrowNear
        classCode = 1;
        return;
    end

    % 6. Decaying
    classCode = 0;
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
        n    = numel(y);
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

function plotSlice(cfg, mapSlice, tdi, outDir)
    if ~cfg.saveFigures; return; end

    Td_val   = cfg.delayList(tdi);
    plotData = mapSlice';   % [nB x nK] — K on X axis, B on Y axis

    fig = figure('Visible','off','Color','w','Position',[0 0 640 480]);
    ax  = axes(fig);

    imagesc(ax, cfg.KList, cfg.BList, plotData);
    colormap(ax, [0 0.55 0.27; 1 0.75 0; 0.80 0.07 0.07; 0.65 0.65 0.65]);
    clim(ax, [0 3]);
    set(ax, 'YDir', 'normal');   % small B at bottom, large B at top

    if cfg.showCellGrid
        hold(ax, 'on');
        overlayCellGrid(ax, cfg.KList, cfg.BList, cfg.gridColor, cfg.gridAlpha, cfg.gridLineWidth);
        hold(ax, 'off');
    end

    cb            = colorbar(ax);
    cb.Ticks      = [0.375, 1.125, 1.875, 2.625];
    cb.TickLabels = {'decaying','near-critical','divergent','failed/timeout'};

    xlabel(ax, 'K [N/m]');
    ylabel(ax, 'B [Ns/m]');
    title(ax, sprintf('EXP3 Stage 2 — Response Map  |  T_d = %.4f s', Td_val), 'FontSize', 12);

    exportgraphics(fig, fullfile(outDir, sprintf('response_map_Td%.4f.png', Td_val)), ...
        'Resolution', 150);
    close(fig);
end

function plotAllSlices(cfg, resultMap, outDir)
    if ~cfg.saveFigures; return; end

    nTd   = numel(cfg.delayList);
    nCols = ceil(sqrt(nTd));
    nRows = ceil(nTd / nCols);

    fig = figure('Visible','off','Color','w','Position',[0 0 430*nCols 370*nRows]);

    for tdi = 1:nTd
        ax       = subplot(nRows, nCols, tdi);
        plotData = resultMap(:,:,tdi)';   % [nB x nK]

        imagesc(ax, cfg.KList, cfg.BList, plotData);
        colormap(ax, [0 0.55 0.27; 1 0.75 0; 0.80 0.07 0.07; 0.65 0.65 0.65]);
        clim(ax, [0 3]);
        set(ax, 'YDir', 'normal');

        if cfg.showCellGrid
            hold(ax, 'on');
            overlayCellGrid(ax, cfg.KList, cfg.BList, cfg.gridColor, cfg.gridAlpha, cfg.gridLineWidth);
            hold(ax, 'off');
        end

        xlabel(ax, 'K [N/m]');
        ylabel(ax, 'B [Ns/m]');
        title(ax, sprintf('T_d = %.4f s', cfg.delayList(tdi)));
    end

    sgtitle('EXP3 Stage 2 — Response Maps across Delays', 'FontSize', 13);
    exportgraphics(fig, fullfile(outDir, 'response_all_slices.png'), 'Resolution', 150);
    close(fig);
end

function plotBoundaryEvolution(cfg, resultMap, outDir)
    if ~cfg.saveFigures; return; end
    nTd    = numel(cfg.delayList);
    colors = parula(nTd);

    fig = figure('Visible','off','Color','w','Position',[0 0 640 430]);
    ax  = axes(fig);
    hold(ax,'on');
    for tdi = 1:nTd
        decayingCount = sum(resultMap(:,:,tdi) == 0, 2);
        plot(ax, cfg.KList, decayingCount / numel(cfg.BList), '-o', ...
            'Color', colors(tdi,:), 'LineWidth', 1.8, 'MarkerSize', 5, ...
            'DisplayName', sprintf('T_d = %.3f s', cfg.delayList(tdi)));
    end
    legend(ax,'Location','northeast','FontSize',8);
    xlabel(ax,'K [N/m]');
    ylabel(ax,'Fraction of B values decaying');
    title(ax,'Decaying region shrinkage as T_d increases');
    grid(ax,'on'); ylim(ax,[0 1.05]);
    exportgraphics(fig, fullfile(outDir, 'boundary_evolution.png'), 'Resolution', 150);
    close(fig);
end

function pts = selectValidationCandidates(sweepTable)
    pts = table();
    T   = sweepTable(isfinite(sweepTable.K) & isfinite(sweepTable.B) & ...
                     isfinite(sweepTable.classCode), :);
    if isempty(T); return; end

    chosen = {};

    Ts = T(T.classCode == 0, :);
    if ~isempty(Ts)
        medK = median(Ts.K); medB = median(Ts.B);
        dist = (Ts.K - medK).^2 + (Ts.B - medB).^2;
        [~, idx] = min(dist);
        chosen(end+1,:) = {'decaying', Ts.K(idx), Ts.B(idx), 0};
    end

    Tn = T(T.classCode == 1, :);
    if ~isempty(Tn)
        chosen(end+1,:) = {'near_critical', Tn.K(1), Tn.B(1), 1};
    else
        if ~isempty(Ts)
            [~, idx] = max(Ts.K);
            chosen(end+1,:) = {'near_boundary', Ts.K(idx), Ts.B(idx), 0};
        end
    end

    Tu = T(T.classCode == 2, :);
    if ~isempty(Tu)
        [~, idx] = min(Tu.K);
        chosen(end+1,:) = {'divergent', Tu.K(idx), Tu.B(idx), 2};
    end

    if isempty(chosen); return; end

    pts = cell2table(chosen, 'VariableNames', {'pointType','K','B','classCode'});
    [~, ia] = unique(pts(:,{'K','B'}), 'rows', 'stable');
    pts = pts(ia,:);
end

function sig = getLoggedSafe(logs, sigName)
    try
        sig = logs.get(sigName);
        if isempty(sig); sig = []; return; end
        sig = sig.Values;
    catch
        sig = [];
    end
end

function label = classCodeToLabel(code)
    switch code
        case 0;    label = 'decaying';
        case 1;    label = 'near-critical';
        case 2;    label = 'divergent';
        otherwise; label = 'failed/timeout';
    end
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function overlayCellGrid(ax, xVals, yVals, gridColor, gridAlpha, gridLineWidth)
    xEdges = centersToEdges(xVals(:));
    yEdges = centersToEdges(yVals(:));

    xl = [xEdges(1), xEdges(end)];
    yl = [yEdges(1), yEdges(end)];

    lineColor = 1 - gridAlpha * (1 - gridColor);

    for k = 1:numel(xEdges)
        plot(ax, [xEdges(k) xEdges(k)], yl, '-', ...
            'Color', lineColor, ...
            'LineWidth', gridLineWidth, ...
            'HandleVisibility', 'off');
    end

    for k = 1:numel(yEdges)
        plot(ax, xl, [yEdges(k) yEdges(k)], '-', ...
            'Color', lineColor, ...
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
    edges(2:end-1) = 0.5 * (c(1:end-1) + c(2:end));
    edges(1) = c(1) - 0.5 * dc(1);
    edges(end) = c(end) + 0.5 * dc(end);
end
