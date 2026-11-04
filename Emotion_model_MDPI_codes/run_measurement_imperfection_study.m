%% run_measurement_imperfection_study.m
% Imperfect-feedback assessment using the common nominal model.
% The no-error/no-delay case is taken directly from simulate_emotion_case,
% so its values exactly match the nominal Scenario 2 simulation.

clear; clc; close all;
thisFolder = fileparts(mfilename('fullpath'));
if isempty(thisFolder), thisFolder = pwd; end
addpath(thisFolder);

if exist('lqr', 'file') ~= 2
    error('The MATLAB Control System Toolbox is required for LQR design.');
end

outputFolder = fullfile(thisFolder, 'validation_results');
if ~exist(outputFolder, 'dir'), mkdir(outputFolder); end

scenarioId = 2;
timeStep = 0.01;
stateBound = 1.0;
saturationTolerance = 1e-8;
errorAmplitude = 0.01;
feedbackDelayMinutes = 0.25;
adverseIndices = [2, 3, 4, 6];

p = emotion_model_parameters(scenarioId);
[~, ~, p.Klqr, ~] = design_lqr_controller(p);

conditionNames = {'Nominal feedback'; 'State error only'; ...
    'Delay only'; 'State error and delay'};
conditionErrorAmplitude = [0; errorAmplitude; 0; errorAmplitude];
conditionDelayMinutes = [0; 0; feedbackDelayMinutes; feedbackDelayMinutes];
numberOfConditions = numel(conditionNames);

% Canonical nominal simulation: used for the open-loop reference and the
% no-error/no-delay closed-loop condition.
canonical = simulate_emotion_case(p);
openLoopResult = canonical_open_loop_result(canonical, p);
nominalLqrResult = canonical_lqr_result(canonical, p);

openLoopPeak = max(openLoopResult.x(:, adverseIndices), [], 'all');
openLoopIAE = sum(trapz(openLoopResult.t, ...
    abs(openLoopResult.x(:, adverseIndices))));

peakAdverse = zeros(numberOfConditions, 1);
adverseIAE = zeros(numberOfConditions, 1);
peakReductionPercent = zeros(numberOfConditions, 1);
iaeReductionPercent = zeros(numberOfConditions, 1);
minimumHappiness = zeros(numberOfConditions, 1);
controlEffort = zeros(numberOfConditions, 1);
saturationPercent = zeros(numberOfConditions, 1);
maximumStateMagnitude = zeros(numberOfConditions, 1);
boundedResponse = false(numberOfConditions, 1);

for conditionIndex = 1:numberOfConditions
    if conditionErrorAmplitude(conditionIndex) == 0 && ...
            conditionDelayMinutes(conditionIndex) == 0
        result = nominalLqrResult;
    else
        result = simulate_imperfect_feedback_case(p, timeStep, ...
            conditionErrorAmplitude(conditionIndex), ...
            conditionDelayMinutes(conditionIndex));
    end

    peakAdverse(conditionIndex) = max(result.x(:, adverseIndices), [], 'all');
    adverseIAE(conditionIndex) = sum(trapz(result.t, ...
        abs(result.x(:, adverseIndices))));
    peakReductionPercent(conditionIndex) = 100 * ...
        (openLoopPeak - peakAdverse(conditionIndex)) / max(openLoopPeak, eps);
    iaeReductionPercent(conditionIndex) = 100 * ...
        (openLoopIAE - adverseIAE(conditionIndex)) / max(openLoopIAE, eps);
    minimumHappiness(conditionIndex) = min(result.x(:, 1));
    controlEffort(conditionIndex) = trapz(result.t, sum(result.uTilde.^2, 2));

    commandAtBoundary = any(result.u <= saturationTolerance | ...
        result.u >= 1 - saturationTolerance, 2);
    saturationPercent(conditionIndex) = 100 * mean(commandAtBoundary);
    maximumStateMagnitude(conditionIndex) = max(abs(result.x(:)));
    boundedResponse(conditionIndex) = all(isfinite(result.x(:))) && ...
        maximumStateMagnitude(conditionIndex) <= stateBound;
end

imperfectionTable = table(conditionNames, conditionErrorAmplitude, ...
    conditionDelayMinutes, peakAdverse, adverseIAE, peakReductionPercent, ...
    iaeReductionPercent, minimumHappiness, controlEffort, ...
    saturationPercent, maximumStateMagnitude, boundedResponse, ...
    'VariableNames', {'FeedbackCondition','EmotionStateErrorAmplitude', ...
    'FeedbackDelayMinutes','PeakAdverseEmotion','AdverseEmotionIAE', ...
    'PeakReductionPercent','IAEReductionPercent','MinimumHappiness', ...
    'ControlEffort','SaturationPercent','MaximumStateMagnitude', ...
    'BoundedResponse'});
writetable(imperfectionTable, fullfile(outputFolder, ...
    'measurement_imperfection_summary.csv'));

figure('Color','w','Position',[100 100 1150 760]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
allLabels = [{'Open loop'}; conditionNames];

nexttile; bar([openLoopPeak; peakAdverse], 'FaceColor', [0.20 0.45 0.75]); grid on; box on;
xticks(1:numel(allLabels)); xticklabels(allLabels); xtickangle(20);
ylabel('Peak emotion deviation'); title('Maximum adverse-emotion response');

nexttile; bar([openLoopIAE; adverseIAE], 'FaceColor', [0.85 0.35 0.15]); grid on; box on;
xticks(1:numel(allLabels)); xticklabels(allLabels); xtickangle(20);
ylabel('Aggregate emotion IAE'); title('Integrated adverse-emotion response');

nexttile; bar([0; controlEffort], 'FaceColor', [0.30 0.65 0.35]); grid on; box on;
xticks(1:numel(allLabels)); xticklabels(allLabels); xtickangle(20);
ylabel('Control effort'); title('Control effort under imperfect feedback');

nexttile; bar([peakReductionPercent, iaeReductionPercent]); grid on; box on;
xticks(1:numberOfConditions); xticklabels(conditionNames); xtickangle(20);
ylabel('Open loop reduction (%)'); title('Retained regulation benefit');
legend({'Peak reduction','IAE reduction'},'Location','southwest');

exportgraphics(gcf, fullfile(outputFolder, ...
    'measurement_imperfection_study.png'), 'Resolution', 300);

fprintf('\nMeasurement-imperfection study\n');
disp(imperfectionTable);
fprintf('Nominal consistency: peak reduction = %.2f%%, IAE reduction = %.2f%%, effort = %.5f\n', ...
    peakReductionPercent(1), iaeReductionPercent(1), controlEffort(1));

function result = simulate_imperfect_feedback_case(p, timeStep, errorAmplitude, delayMinutes)
t = (0:timeStep:p.simulationEnd).';
x = zeros(numel(t), 12);
delaySteps = round(delayMinutes/timeStep);

for k = 1:numel(t)-1
    xk = x(k,:).';
    uTilde = delayed_control(k, t(k), x, p, delaySteps, errorAmplitude);
    k1 = emotion_dynamics_with_input(t(k), xk, p, uTilde);
    k2 = emotion_dynamics_with_input(t(k)+timeStep/2, xk+timeStep*k1/2, p, uTilde);
    k3 = emotion_dynamics_with_input(t(k)+timeStep/2, xk+timeStep*k2/2, p, uTilde);
    k4 = emotion_dynamics_with_input(t(k)+timeStep, xk+timeStep*k3, p, uTilde);
    x(k+1,:) = (xk + timeStep*(k1+2*k2+2*k3+k4)/6).';
end

[u, uRaw, uTilde] = delayed_control_history(t, x, p, delaySteps, errorAmplitude);
result = struct('t',t,'x',x,'u',u,'uRaw',uRaw,'uTilde',uTilde);
end

function uTilde = delayed_control(k, t, x, p, delaySteps, errorAmplitude)
delayedIndex = max(1, k-delaySteps);
xHat = x(delayedIndex,:).' + bounded_state_error(t, errorAmplitude);
u = min(max(p.u0-p.Klqr*xHat,0),1);
uTilde = u-p.u0;
end

function [u, uRaw, uTilde] = delayed_control_history(t, x, p, delaySteps, errorAmplitude)
n = numel(t); u = zeros(n,4); uRaw = zeros(n,4); uTilde = zeros(n,4);
for k = 1:n
    delayedIndex = max(1,k-delaySteps);
    xHat = x(delayedIndex,:).' + bounded_state_error(t(k), errorAmplitude);
    raw = p.u0-p.Klqr*xHat;
    command = min(max(raw,0),1);
    u(k,:) = command.'; uRaw(k,:) = raw.'; uTilde(k,:) = (command-p.u0).';
end
end

function e = bounded_state_error(t, a)
ef = [0.11;0.13;0.17;0.19;0.23;0.29]; rf = [0.31;0.37;0.41;0.43;0.47;0.53];
ep = [0;0.4;0.8;1.2;1.6;2.0]; rp = [0.2;0.6;1.0;1.4;1.8;2.2];
emotionError = a*(0.6*sin(2*pi*ef*t+ep)+0.4*cos(2*pi*ef*t+ep));
rateError = 0.2*a*(0.6*sin(2*pi*rf*t+rp)+0.4*cos(2*pi*rf*t+rp));
e = [emotionError;rateError];
end

function result = canonical_open_loop_result(canonical, p)
n = numel(canonical.tFree); u = repmat(p.u0.',n,1);
result = struct('t',canonical.tFree,'x',canonical.xFree,'u',u, ...
    'uRaw',u,'uTilde',zeros(n,4));
end

function result = canonical_lqr_result(canonical, p)
x = canonical.xLqr; t = canonical.tLqr; n = numel(t);
uRaw = repmat(p.u0.',n,1)-x*p.Klqr.';
u = min(max(uRaw,0),1);
result = struct('t',t,'x',x,'u',u,'uRaw',uRaw,'uTilde',u-repmat(p.u0.',n,1));
end
