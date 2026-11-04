%% run_emotion_simulation.m
% Main entry point for nonlinear emotion simulations in medication assistance.
% Change scenarioId only; all model specifications are stored in function files.

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

scenarioId = 2;  % 1: single unclear reminder; 2: repeated unclear reminder
p = emotion_model_parameters(scenarioId);
[A, B, p.Klqr, closedLoopPoles] = design_lqr_controller(p);

fprintf('Open-loop poles:\n');
disp(eig(A));
fprintf('Closed-loop LQR poles:\n');
disp(closedLoopPoles);
fprintf('LQR gain K (4 x 12):\n');
disp(p.Klqr);

tspan = linspace(0, p.simulationEnd, p.numberOfSamples);
x0 = zeros(12,1);
solverOptions = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

[tFree, xFree] = ode45(@(t,x) emotion_dynamics(t, x, p, false), ...
                         tspan, x0, solverOptions);
[tLqr, xLqr] = ode45(@(t,x) emotion_dynamics(t, x, p, true), ...
                       tspan, x0, solverOptions);

[uLqr, uTildeLqr] = robot_control_history(tLqr, xLqr, p);
metrics = compute_performance_indices(tFree, xFree, tLqr, xLqr, ...
    uLqr, uTildeLqr, p);

plot_and_export_results(tFree, xFree, tLqr, xLqr, uLqr, p, thisFolder);

fprintf('\nPositive-peak comparison (sadness, anger, fear, surprise):\n');
disp(metrics.peakComparison);
fprintf('\nHappiness protection measure:\n');
disp(metrics.happinessComparison);
fprintf('\nIntegral-of-absolute-error comparison:\n');
disp(metrics.iaeComparison);
fprintf('\nPost-event settling-time comparison (minutes):\n');
disp(metrics.settlingComparison);
fprintf('\nRobot-control summary:\n');
disp(metrics.robotControlSummary);
fprintf('Total LQR control effort: %.6e\n', metrics.controlEffort);

emotionDeviationLqr = abs(xLqr(:,1:6));
emotionDeviationFree = abs(xFree(:,1:6));
if any(emotionDeviationLqr(:) > 1) || any(emotionDeviationFree(:) > 1)
    warning(['At least one normalised emotion deviation exceeded one. ', ...
             'Review the scenario amplitudes and parameter values.']);
end
