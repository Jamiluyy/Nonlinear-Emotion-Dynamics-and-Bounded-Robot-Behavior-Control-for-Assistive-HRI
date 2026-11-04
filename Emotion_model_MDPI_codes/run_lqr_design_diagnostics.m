%% run_lqr_design_diagnostics.m
% Numerical LQR-design diagnostics for the nominal emotion model.
% This script reports the exact weighting matrices, feedback gain,
% closed-loop poles, and PBH stabilizability/detectability checks.

clear; clc;

thisFolder = fileparts(mfilename('fullpath'));
if isempty(thisFolder)
    thisFolder = pwd;
end
addpath(thisFolder);

if exist('lqr', 'file') ~= 2
    error(['The function lqr was not found. Install or enable the ', ...
        'MATLAB Control System Toolbox before running this script.']);
end

%% 1. Nominal model and LQR design
scenarioId = 2;
p = emotion_model_parameters(scenarioId);

[A, B, K, closedLoopPoles] = design_lqr_controller(p);

nStates = size(A, 1);
tolerance = 1e-9;

%% 2. Controllability rank
controllabilityMatrix = ctrb(A, B);
controllabilityRank = rank(controllabilityMatrix, tolerance);

%% 3. PBH stabilizability test
% Only eigenvalues with non-negative real part need to be checked.
openLoopPoles = eig(A);
criticalOpenLoopPoles = openLoopPoles(real(openLoopPoles) >= -tolerance);

if isempty(criticalOpenLoopPoles)
    stabilizable = true;
    minimumPBHStabilizabilityRank = nStates;
else
    pbhStabilizabilityRanks = zeros(numel(criticalOpenLoopPoles), 1);

    for index = 1:numel(criticalOpenLoopPoles)
        lambda = criticalOpenLoopPoles(index);
        pbhMatrix = [lambda * eye(nStates) - A, B];
        pbhStabilizabilityRanks(index) = rank(pbhMatrix, tolerance);
    end

    minimumPBHStabilizabilityRank = min(pbhStabilizabilityRanks);
    stabilizable = all(pbhStabilizabilityRanks == nStates);
end

%% 4. PBH detectability test
% Q^(1/2) is used as the output matrix associated with the LQR state cost.
Cq = sqrtm(p.Qlqr);

if isempty(criticalOpenLoopPoles)
    detectable = true;
    minimumPBHDetectabilityRank = nStates;
else
    pbhDetectabilityRanks = zeros(numel(criticalOpenLoopPoles), 1);

    for index = 1:numel(criticalOpenLoopPoles)
        lambda = criticalOpenLoopPoles(index);
        pbhMatrix = [lambda * eye(nStates) - A; Cq];
        pbhDetectabilityRanks(index) = rank(pbhMatrix, tolerance);
    end

    minimumPBHDetectabilityRank = min(pbhDetectabilityRanks);
    detectable = all(pbhDetectabilityRanks == nStates);
end

%% 5. Closed-loop stability check
closedLoopSpectralAbscissa = max(real(closedLoopPoles));
closedLoopStable = closedLoopSpectralAbscissa < -tolerance;

%% 6. Export tables
outputFolder = fullfile(thisFolder, 'validation_results');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

gainRowNames = {'Support', 'Predictability', 'Adaptation', 'Repair'};
stateColumnNames = {'Happiness', 'Sadness', 'Anger', 'Fear', ...
    'Disgust', 'Surprise', 'HappinessRate', 'SadnessRate', ...
    'AngerRate', 'FearRate', 'DisgustRate', 'SurpriseRate'};

gainTable = array2table(K, ...
    'RowNames', gainRowNames, ...
    'VariableNames', stateColumnNames);

writetable(gainTable, fullfile(outputFolder, 'nominal_lqr_gain.csv'), ...
    'WriteRowNames', true);

diagnosticNames = { ...
    'Number of states'; ...
    'Number of inputs'; ...
    'Controllability-matrix rank'; ...
    'Minimum PBH stabilizability rank'; ...
    'Minimum PBH detectability rank'; ...
    'Closed-loop spectral abscissa'; ...
    'Stabilizable'; ...
    'Detectable'; ...
    'Closed-loop asymptotically stable'};

diagnosticValues = { ...
    nStates; ...
    size(B, 2); ...
    controllabilityRank; ...
    minimumPBHStabilizabilityRank; ...
    minimumPBHDetectabilityRank; ...
    closedLoopSpectralAbscissa; ...
    stabilizable; ...
    detectable; ...
    closedLoopStable};

diagnosticTable = table(diagnosticNames, diagnosticValues, ...
    'VariableNames', {'Diagnostic', 'Value'});

writetable(diagnosticTable, fullfile(outputFolder, ...
    'nominal_lqr_design_diagnostics.csv'));

%% 7. Command-window output
fprintf('\nNominal LQR design diagnostics\n');
fprintf('Scenario: %d\n\n', scenarioId);

fprintf('State-weighting matrix Q:\n');
disp(p.Qlqr);

fprintf('Control-weighting matrix W:\n');
disp(p.Wlqr);

fprintf('Nominal LQR gain K:\n');
disp(gainTable);

fprintf('Open-loop eigenvalues:\n');
disp(openLoopPoles);

fprintf('Closed-loop eigenvalues:\n');
disp(closedLoopPoles);

fprintf('\nLQR diagnostic summary:\n');
disp(diagnosticTable);

fprintf('\nInterpretation:\n');
fprintf('Stabilizable: %d\n', stabilizable);
fprintf('Detectable: %d\n', detectable);
fprintf('Closed-loop asymptotically stable: %d\n', closedLoopStable);
fprintf('Closed-loop spectral abscissa: %.8f\n', ...
    closedLoopSpectralAbscissa);

fprintf('\nFiles exported to:\n%s\n', outputFolder);