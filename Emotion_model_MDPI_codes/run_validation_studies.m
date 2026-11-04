%% run_validation_studies.m
% Robustness and LQR-weighting studies for the nonlinear emotion model.
% Scenario 2, the repeated unclear medication reminder, is used because it
% is the more demanding of the two established scenarios.

clear; clc; close all;
thisFolder = fileparts(mfilename('fullpath'));
if isempty(thisFolder), thisFolder = pwd; end
addpath(thisFolder);
if exist('lqr', 'file') ~= 2
    error(['The function lqr was not found. Install or enable the ', ...
        'MATLAB Control System Toolbox before running this script.']);
end

outputFolder = fullfile(thisFolder, 'validation_results');
if ~exist(outputFolder, 'dir'), mkdir(outputFolder); end
scenarioId = 2;
pNominal = emotion_model_parameters(scenarioId);
[~, ~, pNominal.Klqr, ~] = design_lqr_controller(pNominal);

%% Part A: LQR weighting study
designNames = {'Balanced', 'Protective', 'Low effort'};
designCount = numel(designNames);

% Balanced: nominal design from the original simulations.
pBalanced = rmfield(pNominal, 'Klqr');

% Protective: larger penalties on sadness, anger, fear, and surprise.
pProtective = pNominal;
pProtective.Qlqr = diag([5, 25, 30, 35, 8, 10, 1, 1, 1, 1, 1, 1]);
pProtective.Wlqr = 0.1 * eye(4);
pProtective = rmfield(pProtective, 'Klqr');

% Low effort: nominal state penalties and a larger effort penalty.
pLowEffort = pNominal;
pLowEffort.Qlqr = pNominal.Qlqr;
pLowEffort.Wlqr = 0.5 * eye(4);
pLowEffort = rmfield(pLowEffort, 'Klqr');

designParameters = {pBalanced, pProtective, pLowEffort};
designResults = cell(designCount,1);
for designIndex = 1:designCount
    designResults{designIndex} = simulate_emotion_case( ...
        designParameters{designIndex});
end

positiveIndices = [2, 3, 4, 6];
peakValues = zeros(designCount, numel(positiveIndices));
happinessMinimum = zeros(designCount,1);
undesirableIae = zeros(designCount,1);
controlEffort = zeros(designCount,1);
maximumCommand = zeros(designCount,1);
for designIndex = 1:designCount
    result = designResults{designIndex};
    peakValues(designIndex,:) = max(result.xLqr(:,positiveIndices), [], 1);
    happinessMinimum(designIndex) = min(result.xLqr(:,1));
    undesirableIae(designIndex) = sum(result.metrics.iaeComparison.IAELQR( ...
        ismember(result.metrics.iaeComparison.Emotion, ...
        {'Sadness','Anger','Fear','Surprise'})));
    controlEffort(designIndex) = result.metrics.controlEffort;
    maximumCommand(designIndex) = max(result.uLqr(:));
end

weightingTable = table(designNames.', peakValues(:,1), peakValues(:,2), ...
    peakValues(:,3), peakValues(:,4), happinessMinimum, undesirableIae, ...
    controlEffort, maximumCommand, 'VariableNames', ...
    {'Design','SadnessPeak','AngerPeak','FearPeak','SurprisePeak', ...
     'MinimumHappiness','UndesirableEmotionIAE','ControlEffort', ...
     'MaximumCommand'});
writetable(weightingTable, fullfile(outputFolder, 'lqr_weighting_study.csv'));

figWeighting = figure('Color','w','Position',[100,100,1050,620]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
peakTitles = {'Sadness peak','Anger peak','Fear peak'};
for index = 1:3
    nexttile; bar(peakValues(:,index));
    set(gca, 'XTick', 1:designCount, 'XTickLabel', designNames);
    ylabel('\Delta emotion'); grid on; title(peakTitles{index});
end
nexttile; bar([undesirableIae, controlEffort]);
set(gca, 'XTick', 1:designCount, 'XTickLabel', designNames);
ylabel('Value'); grid on;
legend('Undesirable-emotion IAE','Control effort','Location','best');
title('Performance-effort trade-off');
print(figWeighting, fullfile(outputFolder, 'lqr_weighting_study.png'), ...
    '-dpng', '-r300');

%% Part B: Robustness study with a fixed nominal LQR gain
numberOfTrials = 100;
uncertaintyLevel = 0.20;
rng(2026, 'twister');
openLoopPeak = zeros(numberOfTrials, numel(positiveIndices));
lqrPeak = zeros(numberOfTrials, numel(positiveIndices));
openLoopIae = zeros(numberOfTrials,1);
lqrIae = zeros(numberOfTrials,1);
openLoopHappinessMinimum = zeros(numberOfTrials,1);
lqrHappinessMinimum = zeros(numberOfTrials,1);
boundedResponse = false(numberOfTrials,1);

for trial = 1:numberOfTrials
    pTrial = perturb_emotion_parameters(pNominal, uncertaintyLevel);
    % The nominal gain is retained: this tests robustness rather than retuning.
    pTrial.Klqr = pNominal.Klqr;
    trialResult = simulate_emotion_case(pTrial);
    openLoopPeak(trial,:) = max(trialResult.xFree(:,positiveIndices), [], 1);
    lqrPeak(trial,:) = max(trialResult.xLqr(:,positiveIndices), [], 1);
    openLoopIae(trial) = sum(trapz(trialResult.tFree, ...
        abs(trialResult.xFree(:,positiveIndices))));
    lqrIae(trial) = sum(trapz(trialResult.tLqr, ...
        abs(trialResult.xLqr(:,positiveIndices))));
    openLoopHappinessMinimum(trial) = min(trialResult.xFree(:,1));
    lqrHappinessMinimum(trial) = min(trialResult.xLqr(:,1));
    boundedResponse(trial) = all(isfinite(trialResult.xFree(:))) && ...
        all(isfinite(trialResult.xLqr(:))) && ...
        max(abs([trialResult.xFree(:); trialResult.xLqr(:)])) <= 1;
end

emotionNames = pNominal.emotionNames(positiveIndices);
meanOpenLoopPeak = mean(openLoopPeak,1).';
meanLqrPeak = mean(lqrPeak,1).';
meanPeakReduction = 100 * (meanOpenLoopPeak - meanLqrPeak) ./ ...
    max(meanOpenLoopPeak, eps);
peakReductionSuccessRate = 100 * mean(lqrPeak < openLoopPeak,1).';
robustnessTable = table(emotionNames.', meanOpenLoopPeak, meanLqrPeak, ...
    meanPeakReduction, peakReductionSuccessRate, 'VariableNames', ...
    {'Emotion','MeanOpenLoopPeak','MeanLQRPeak', ...
     'MeanPeakReductionPercent','PeakReductionSuccessRatePercent'});
writetable(robustnessTable, fullfile(outputFolder, 'robustness_peak_summary.csv'));

robustnessGlobalTable = table(mean(openLoopIae), mean(lqrIae), ...
    100*(mean(openLoopIae)-mean(lqrIae))/max(mean(openLoopIae),eps), ...
    mean(openLoopHappinessMinimum), mean(lqrHappinessMinimum), ...
    100*mean(boundedResponse), 'VariableNames', ...
    {'MeanUndesirableIAEOpenLoop','MeanUndesirableIAELQR', ...
     'MeanIAEReductionPercent','MeanMinimumHappinessOpenLoop', ...
     'MeanMinimumHappinessLQR','BoundedResponseRatePercent'});
writetable(robustnessGlobalTable, ...
    fullfile(outputFolder, 'robustness_global_summary.csv'));

figRobustness = figure('Color','w','Position',[100,100,1000,500]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; bar(meanPeakReduction);
set(gca, 'XTick', 1:numel(emotionNames), 'XTickLabel', emotionNames);
ylabel('Mean peak reduction (%)'); grid on;
title('Robustness study: adverse-emotion peaks');
nexttile; bar([mean(openLoopIae), mean(lqrIae)]);
set(gca, 'XTick', 1:2, 'XTickLabel', {'Open loop','Saturated LQR'});
ylabel('Mean undesirable-emotion IAE'); grid on;
title('Robustness study: aggregate response error');
print(figRobustness, fullfile(outputFolder, 'robustness_study.png'), ...
    '-dpng', '-r300');

fprintf('\nLQR-weighting study:\n'); disp(weightingTable);
fprintf('\nRobustness study: peak summary (%d trials, +/- %.0f%% uncertainty):\n', ...
    numberOfTrials, 100*uncertaintyLevel); disp(robustnessTable);
fprintf('\nRobustness study: aggregate summary:\n'); disp(robustnessGlobalTable);
fprintf('\nFiles exported to:\n%s\n', outputFolder);
