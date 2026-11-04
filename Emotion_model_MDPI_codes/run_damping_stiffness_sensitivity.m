%% run_damping_stiffness_sensitivity.m
% Joint damping-stiffness sensitivity study for the nonlinear emotion model.
%
% All damping coefficients are scaled together by c/c0, and all stiffness
% coefficients are scaled together by k/k0. The nominal LQR gain remains
% fixed so that the study evaluates model sensitivity and controller
% robustness to restoring-coefficient mismatch.

clear; clc; close all;

thisFolder = fileparts(mfilename('fullpath'));
if isempty(thisFolder)
    thisFolder = pwd;
end
addpath(thisFolder);

if exist('lqr', 'file') ~= 2
    error(['The function lqr was not found. Install or enable the ', ...
        'MATLAB Control System Toolbox before running this script.']);
end

outputFolder = fullfile(thisFolder, 'validation_results');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% 1. Nominal model and study settings
scenarioId = 2;
timeStep = 0.01;
stateBound = 1.0;

pNominal = emotion_model_parameters(scenarioId);
[~, ~, pNominal.Klqr, ~] = design_lqr_controller(pNominal);

% Multipliers for the nominal damping and stiffness vectors.
dampingScales = [0.75, 0.875, 1.00, 1.125, 1.25];
stiffnessScales = [0.75, 0.875, 1.00, 1.125, 1.25];

numberOfDampingValues = numel(dampingScales);
numberOfStiffnessValues = numel(stiffnessScales);

adverseIndices = [2, 3, 4, 6];

%% 2. Preallocate results
openLoopPeak = zeros(numberOfStiffnessValues, numberOfDampingValues);
lqrPeak = zeros(numberOfStiffnessValues, numberOfDampingValues);

openLoopIAE = zeros(numberOfStiffnessValues, numberOfDampingValues);
lqrIAE = zeros(numberOfStiffnessValues, numberOfDampingValues);

peakReductionPercent = zeros(numberOfStiffnessValues, ...
    numberOfDampingValues);

iaeReductionPercent = zeros(numberOfStiffnessValues, ...
    numberOfDampingValues);

lqrMaximumState = zeros(numberOfStiffnessValues, ...
    numberOfDampingValues);

openLoopBounded = false(numberOfStiffnessValues, ...
    numberOfDampingValues);

lqrBounded = false(numberOfStiffnessValues, ...
    numberOfDampingValues);

%% 3. Joint damping-stiffness simulations
for stiffnessIndex = 1:numberOfStiffnessValues

    for dampingIndex = 1:numberOfDampingValues

        pTrial = pNominal;

        pTrial.c = dampingScales(dampingIndex) * pNominal.c;
        pTrial.k = stiffnessScales(stiffnessIndex) * pNominal.k;

        % Retain the nominal gain to test robustness to coefficient changes.
        pTrial.Klqr = pNominal.Klqr;

        openLoopResult = simulate_restoring_coefficient_case( ...
            pTrial, scenarioId, timeStep, false);

        lqrResult = simulate_restoring_coefficient_case( ...
            pTrial, scenarioId, timeStep, true);

        openLoopPeak(stiffnessIndex, dampingIndex) = ...
            max(max(openLoopResult.x(:, adverseIndices)));

        lqrPeak(stiffnessIndex, dampingIndex) = ...
            max(max(lqrResult.x(:, adverseIndices)));

        openLoopIAE(stiffnessIndex, dampingIndex) = ...
            sum(trapz(openLoopResult.t, ...
            abs(openLoopResult.x(:, adverseIndices))));

        lqrIAE(stiffnessIndex, dampingIndex) = ...
            sum(trapz(lqrResult.t, ...
            abs(lqrResult.x(:, adverseIndices))));

        peakReductionPercent(stiffnessIndex, dampingIndex) = ...
            100 * (openLoopPeak(stiffnessIndex, dampingIndex) - ...
            lqrPeak(stiffnessIndex, dampingIndex)) / ...
            max(openLoopPeak(stiffnessIndex, dampingIndex), eps);

        iaeReductionPercent(stiffnessIndex, dampingIndex) = ...
            100 * (openLoopIAE(stiffnessIndex, dampingIndex) - ...
            lqrIAE(stiffnessIndex, dampingIndex)) / ...
            max(openLoopIAE(stiffnessIndex, dampingIndex), eps);

        lqrMaximumState(stiffnessIndex, dampingIndex) = ...
            max(abs(lqrResult.x(:)));

        openLoopBounded(stiffnessIndex, dampingIndex) = ...
            all(isfinite(openLoopResult.x(:))) && ...
            max(abs(openLoopResult.x(:))) <= stateBound;

        lqrBounded(stiffnessIndex, dampingIndex) = ...
            all(isfinite(lqrResult.x(:))) && ...
            lqrMaximumState(stiffnessIndex, dampingIndex) <= stateBound;
    end
end

%% 4. Export full numerical grid
[dampingGrid, stiffnessGrid] = meshgrid( ...
    dampingScales, stiffnessScales);

sensitivityTable = table(dampingGrid(:), stiffnessGrid(:), ...
    openLoopPeak(:), lqrPeak(:), peakReductionPercent(:), ...
    openLoopIAE(:), lqrIAE(:), iaeReductionPercent(:), ...
    lqrMaximumState(:), openLoopBounded(:), lqrBounded(:), ...
    'VariableNames', ...
    {'DampingScale', 'StiffnessScale', ...
     'OpenLoopPeakAdverseEmotion', 'LQRPeakAdverseEmotion', ...
     'PeakReductionPercent', 'OpenLoopAdverseEmotionIAE', ...
     'LQRAdverseEmotionIAE', 'IAEReductionPercent', ...
     'LQRMaximumStateMagnitude', 'OpenLoopBounded', 'LQRBounded'});

writetable(sensitivityTable, fullfile(outputFolder, ...
    'damping_stiffness_sensitivity_summary.csv'));

%% 5. Generate one compact sensitivity figure
combinedPeakValues = [openLoopPeak(:); lqrPeak(:)];
peakColourLimits = [min(combinedPeakValues), max(combinedPeakValues)];

figure('Color', 'w', 'Position', [100 100 1250 400]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
imagesc(dampingScales, stiffnessScales, openLoopPeak);
set(gca, 'YDir', 'normal');
caxis(peakColourLimits);
colorbar;
xlabel('Damping multiplier, c/c_0');
ylabel('Stiffness multiplier, k/k_0');
title('Open-loop peak adverse emotion');

nexttile;
imagesc(dampingScales, stiffnessScales, lqrPeak);
set(gca, 'YDir', 'normal');
caxis(peakColourLimits);
colorbar;
xlabel('Damping multiplier, c/c_0');
ylabel('Stiffness multiplier, k/k_0');
title('Saturated-LQR peak adverse emotion');

nexttile;
imagesc(dampingScales, stiffnessScales, iaeReductionPercent);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('Damping multiplier, c/c_0');
ylabel('Stiffness multiplier, k/k_0');
title('LQR IAE reduction relative to open loop (%)');

exportgraphics(gcf, fullfile(outputFolder, ...
    'damping_stiffness_sensitivity.png'), ...
    'Resolution', 300);

%% 6. Command-window summary
fprintf('\nJoint damping-stiffness sensitivity study\n');
fprintf('Scenario: %d\n', scenarioId);
fprintf('Damping range: %.3f to %.3f of nominal\n', ...
    min(dampingScales), max(dampingScales));
fprintf('Stiffness range: %.3f to %.3f of nominal\n\n', ...
    min(stiffnessScales), max(stiffnessScales));

fprintf('All open-loop cases bounded: %d of %d\n', ...
    sum(openLoopBounded(:)), numel(openLoopBounded));

fprintf('All saturated-LQR cases bounded: %d of %d\n', ...
    sum(lqrBounded(:)), numel(lqrBounded));

fprintf('LQR peak-reduction range: %.2f%% to %.2f%%\n', ...
    min(peakReductionPercent(:)), max(peakReductionPercent(:)));

fprintf('LQR IAE-reduction range: %.2f%% to %.2f%%\n', ...
    min(iaeReductionPercent(:)), max(iaeReductionPercent(:)));

fprintf('Maximum saturated-LQR state magnitude: %.6f\n', ...
    max(lqrMaximumState(:)));

fprintf('\nFiles exported to:\n%s\n', outputFolder);

%% Local functions

function result = simulate_restoring_coefficient_case( ...
    p, scenarioId, timeStep, useLqr)

    t = (0:timeStep:p.simulationEnd).';
    numberOfSamples = numel(t);

    x = zeros(numberOfSamples, 12);

    for sampleIndex = 1:numberOfSamples - 1

        currentTime = t(sampleIndex);
        currentState = x(sampleIndex, :).';

        if useLqr
            uRaw = p.u0 - p.Klqr * currentState;
            u = min(max(uRaw, 0), 1);
            uTilde = u - p.u0;
        else
            uTilde = zeros(4, 1);
        end

        k1 = restoring_coefficient_dynamics( ...
            currentTime, currentState, p, scenarioId, uTilde);

        k2 = restoring_coefficient_dynamics( ...
            currentTime + timeStep/2, ...
            currentState + timeStep*k1/2, p, scenarioId, uTilde);

        k3 = restoring_coefficient_dynamics( ...
            currentTime + timeStep/2, ...
            currentState + timeStep*k2/2, p, scenarioId, uTilde);

        k4 = restoring_coefficient_dynamics( ...
            currentTime + timeStep, ...
            currentState + timeStep*k3, p, scenarioId, uTilde);

        x(sampleIndex + 1, :) = ...
            (currentState + timeStep*(k1 + 2*k2 + 2*k3 + k4)/6).';
    end

    result = struct('t', t, 'x', x);
end

function dx = restoring_coefficient_dynamics( ...
    t, x, p, scenarioId, uTilde)

    e = x(1:6);
    de = x(7:12);

    z = scenario_two_restoring_inputs(t, scenarioId);

    Fh = p.b.h(1)*z.P + p.b.h(2)*z.Ch + p.b.h(3)*z.Sh ...
       + p.gamma.hs*e(2) + p.gamma.ha*e(3) + p.gamma.hf*e(4) ...
       + p.gamma.hq*z.nu*e(6) + p.R(1,:)*uTilde;

    Fs = p.b.s(1)*z.Ls + p.b.s(2)*z.Cs + p.b.s(3)*z.Ms ...
       - p.b.s(4)*z.Rs + p.gamma.sh*e(1) + p.R(2,:)*uTilde;

    Fa = p.b.a(1)*z.Ga + p.b.a(2)*z.Ba + p.b.a(3)*z.Ua ...
       - p.b.a(4)*z.Ra + p.gamma.ah*e(1) + p.R(3,:)*uTilde;

    Ff = p.b.f(1)*z.Tf + p.b.f(2)*z.Cf + p.b.f(3)*z.Mf ...
       - p.b.f(4)*z.Sf + p.gamma.fh*e(1) + p.R(4,:)*uTilde;

    Fd = p.b.d(1)*z.Vd + p.b.d(2)*z.Md + p.b.d(3)*z.Nd ...
       - p.b.d(4)*z.Hd + p.R(5,:)*uTilde;

    Fq = p.b.q(1)*z.Iq + p.b.q(2)*z.Uq - p.b.q(3)*z.Eq ...
       + p.R(6,:)*uTilde;

    dde = -p.c .* de - p.k .* e + [Fh; Fs; Fa; Ff; Fd; Fq];
    dde(1) = dde(1) - p.beta_h * e(1)^3;

    dx = [de; dde];
end

function z = scenario_two_restoring_inputs(t, scenarioId)

    if scenarioId ~= 2
        error('This script is configured for Scenario 2 only.');
    end

    z = struct('P',0,'Ch',0,'Sh',0, ...
               'Ls',0,'Cs',0,'Ms',0,'Rs',0, ...
               'Ga',0,'Ba',0,'Ua',0,'Ra',0, ...
               'Tf',0,'Cf',0,'Mf',0,'Sf',0, ...
               'Vd',0,'Md',0,'Nd',0,'Hd',0, ...
               'Iq',0,'Uq',0,'Eq',0,'nu',0);

    eventOne = double(t >= 0 && t < 2);
    eventTwo = double(t >= 8 && t < 10);

    z.Iq = 0.8*eventOne + 0.9*eventTwo;
    z.Uq = 0.6*eventOne + 0.7*eventTwo;
    z.Tf = 0.4*eventOne + 0.55*eventTwo;
    z.Ga = 0.3*eventOne + 0.6*eventTwo;
    z.Cs = 0.2*eventOne + 0.35*eventTwo;
    z.Ms = 0.15*eventTwo;
    z.Mf = 0.20*eventTwo;
    z.nu = -0.5*eventOne - 0.7*eventTwo;
end