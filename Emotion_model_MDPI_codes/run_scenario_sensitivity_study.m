%% run_scenario_sensitivity_study.m
% One-factor-at-a-time scenario sensitivity study for Scenario 2.
% The nominal LQR gain is retained in every case.

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

% Nominal Scenario 2 controller.
pNominal = emotion_model_parameters(2);
[~, ~, pNominal.Klqr, ~] = design_lqr_controller(pNominal);

% Each study changes one scenario characteristic while all other settings,
% including the nominal LQR gain, remain unchanged.
studyNames = {'Amplitude', 'Duration', 'Reminder interval'};
studyValues = {
    [0.75, 1.00, 1.25], ...   % Common event-amplitude multiplier
    [0.75, 1.00, 1.25], ...   % Common duration multiplier
    [6.00, 8.00, 12.00] ...   % Start time of second reminder, in minutes
    };

studyXLabels = {
    'Event-amplitude multiplier', ...
    'Event-duration multiplier', ...
    'Second-reminder start time (min)'
    };

numberOfStudies = numel(studyNames);
numberOfCases = sum(cellfun(@numel, studyValues));

studyLabel = cell(numberOfCases, 1);
settingValue = zeros(numberOfCases, 1);
eventAmplitudeScale = zeros(numberOfCases, 1);
eventDurationMin = zeros(numberOfCases, 1);
secondReminderStartMin = zeros(numberOfCases, 1);
openLoopPeakRisk = zeros(numberOfCases, 1);
lqrPeakRisk = zeros(numberOfCases, 1);
peakRiskReductionPercent = zeros(numberOfCases, 1);
openLoopRiskIAE = zeros(numberOfCases, 1);
lqrRiskIAE = zeros(numberOfCases, 1);
riskIAEReductionPercent = zeros(numberOfCases, 1);
minimumHappinessLQR = zeros(numberOfCases, 1);
controlEffort = zeros(numberOfCases, 1);
saturationPercent = zeros(numberOfCases, 1);
boundedResponse = false(numberOfCases, 1);

positiveIndices = [2, 3, 4, 6];
row = 0;

for studyIndex = 1:numberOfStudies
    currentValues = studyValues{studyIndex};

    for valueIndex = 1:numel(currentValues)
        row = row + 1;
        p = pNominal;

        % Nominal Scenario 2 settings.
        p.eventAmplitudeScale = 1.0;
        p.eventDurationMin = 2.0;
        p.secondReminderStartMin = 8.0;

        switch studyNames{studyIndex}
            case 'Amplitude'
                p.eventAmplitudeScale = currentValues(valueIndex);

            case 'Duration'
                p.eventDurationMin = 2.0 * currentValues(valueIndex);

            case 'Reminder interval'
                p.secondReminderStartMin = currentValues(valueIndex);
        end

        % Update the final-event time used for settling-time calculations.
        p.eventEndTimes = [p.eventDurationMin, ...
            p.secondReminderStartMin + p.eventDurationMin];
        p.settlingReferenceTime = p.eventEndTimes(end);

        % Retain the nominal controller. No controller retuning is performed.
        p.Klqr = pNominal.Klqr;
        result = simulate_emotion_case(p);

        openLoopPeakRisk(row) = max(result.xFree(:, positiveIndices), [], 'all');
        lqrPeakRisk(row) = max(result.xLqr(:, positiveIndices), [], 'all');

        peakRiskReductionPercent(row) = 100 * ...
            (openLoopPeakRisk(row) - lqrPeakRisk(row)) / ...
            max(openLoopPeakRisk(row), eps);

        openLoopRiskIAE(row) = sum(trapz(result.tFree, ...
            abs(result.xFree(:, positiveIndices))));

        lqrRiskIAE(row) = sum(trapz(result.tLqr, ...
            abs(result.xLqr(:, positiveIndices))));

        riskIAEReductionPercent(row) = 100 * ...
            (openLoopRiskIAE(row) - lqrRiskIAE(row)) / ...
            max(openLoopRiskIAE(row), eps);

        minimumHappinessLQR(row) = min(result.xLqr(:, 1));
        controlEffort(row) = result.metrics.controlEffort;

        isSaturated = any(result.uLqr <= 1e-10 | ...
            result.uLqr >= 1 - 1e-10, 2);
        saturationPercent(row) = 100 * mean(isSaturated);

        boundedResponse(row) = all(isfinite(result.xFree(:))) && ...
            all(isfinite(result.xLqr(:))) && ...
            max(abs([result.xFree(:); result.xLqr(:)])) <= 1;

        studyLabel{row} = studyNames{studyIndex};
        settingValue(row) = currentValues(valueIndex);
        eventAmplitudeScale(row) = p.eventAmplitudeScale;
        eventDurationMin(row) = p.eventDurationMin;
        secondReminderStartMin(row) = p.secondReminderStartMin;
    end
end

summaryTable = table(studyLabel, settingValue, eventAmplitudeScale, ...
    eventDurationMin, secondReminderStartMin, openLoopPeakRisk, ...
    lqrPeakRisk, peakRiskReductionPercent, openLoopRiskIAE, ...
    lqrRiskIAE, riskIAEReductionPercent, minimumHappinessLQR, ...
    controlEffort, saturationPercent, boundedResponse, ...
    'VariableNames', {'Study', 'SettingValue', ...
    'EventAmplitudeScale', 'EventDurationMin', ...
    'SecondReminderStartMin', 'OpenLoopPeakRisk', 'LQRPeakRisk', ...
    'PeakRiskReductionPercent', 'OpenLoopRiskIAE', 'LQRRiskIAE', ...
    'RiskIAEReductionPercent', 'MinimumHappinessLQR', ...
    'ControlEffort', 'SaturationPercent', 'BoundedResponse'});

writetable(summaryTable, fullfile(outputFolder, ...
    'scenario_sensitivity_summary.csv'));

% Plot percentage risk reduction across the three one-factor studies.
figure('Color', 'w', 'Position', [120, 120, 1200, 360]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for studyIndex = 1:numberOfStudies
    nexttile;
    currentRows = strcmp(studyLabel, studyNames{studyIndex});

    plot(settingValue(currentRows), ...
        peakRiskReductionPercent(currentRows), '-o', ...
        'LineWidth', 1.8, 'MarkerSize', 6);
    hold on;
    plot(settingValue(currentRows), ...
        riskIAEReductionPercent(currentRows), '-s', ...
        'LineWidth', 1.8, 'MarkerSize', 6);

    grid on;
    box on;
    xlabel(studyXLabels{studyIndex});
    ylabel('Reduction relative to open loop (%)');
    title(studyNames{studyIndex});
    % legend('Peak risk', 'Risk IAE', 'Location', 'best');
    legend('Peak deviation', 'Aggregate IAE', 'Location', 'best');
end

exportgraphics(gcf, fullfile(outputFolder, ...
    'scenario_sensitivity_reductions.png'), 'Resolution', 300);

fprintf('\nScenario sensitivity study summary:\n');
disp(summaryTable);

fprintf('\nAll cases bounded: %d of %d\n', ...
    sum(boundedResponse), numberOfCases);

fprintf('Files exported to:\n%s\n', outputFolder);