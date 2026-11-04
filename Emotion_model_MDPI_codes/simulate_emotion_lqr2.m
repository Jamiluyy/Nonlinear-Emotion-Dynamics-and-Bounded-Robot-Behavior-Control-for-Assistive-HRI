%% simulate_emotion_lqr.m
% Nonlinear six-emotion simulation for medication assistance.
% Compares uncontrolled dynamics against a saturated LQR controller.
%
% Requirements: Control System Toolbox (for lqr).
% Time is measured in minutes.  State order is
% [h s a f d q dh ds da df dd dq]^T, where each emotion is a deviation
% from the user's individual baseline.

clear; clc; close all;

outputFolder = fileparts(mfilename('fullpath'));
if isempty(outputFolder)
    outputFolder = pwd;
end

if exist('lqr', 'file') ~= 2
    error(['The function lqr was not found. Install or enable the ', ...
           'MATLAB Control System Toolbox before running this script.']);
end

%% 1. Baseline model parameters
p.c = 0.8 * ones(6,1);                 % damping coefficients [min^-1]
p.k = 0.16 * ones(6,1);                % restoring coefficients [min^-2]
p.beta_h = 0.008;                      % happiness cubic coefficient [min^-2]

% Coupling coefficients [min^-2].  The surprise-to-happiness effect is
% handled separately because it is multiplied by the time-varying valence.
p.gamma.hs = -0.006;
p.gamma.sh = -0.006;
p.gamma.ha = -0.006;
p.gamma.ah = -0.005;
p.gamma.hf = -0.008;
p.gamma.fh = -0.007;
p.gamma.dh = -0.004;
p.gamma.hq =  0.004;

% Robot-input matrix.  Columns are: supportive communication,
% predictability/explanation, task adaptation, and repair-oriented action.
p.R = [ 0.008,  0.006,  0.007,  0.005;   % happiness
       -0.005, -0.004, -0.007, -0.006;   % sadness
       -0.006, -0.007, -0.005, -0.008;   % anger
       -0.007, -0.008, -0.006, -0.004;   % fear
        0.000,  0.000,  0.000,  0.000;   % disgust: no direct actuation
        0.000, -0.008,  0.000, -0.005];  % surprise

% Appraisal gains [min^-2].  The order in each vector follows the notation
% used in the written model.
p.b.h = [0.030, 0.020, 0.025];          % P, C, S
p.b.s = [0.030, 0.025, 0.020, 0.030];   % L, C_s, M_s, R_s
p.b.a = [0.030, 0.025, 0.025, 0.030];   % G_a, B_a, U_a, R_a
p.b.f = [0.040, 0.030, 0.020, 0.040];   % T_f, C_f, M_f, S_f
p.b.d = [0.030, 0.020, 0.020, 0.030];   % V_d, M_d, N_d, H_d
p.b.q = [0.040, 0.035, 0.040];          % I_q, U_q, E_q

% Nominal physical robot behaviour.  The controller changes this vector,
% and each physical input is then saturated to the interval [0, 1].
p.u0 = [0.50; 0.50; 0.50; 0.25];

%% 2. Linearised model and LQR controller
Gamma0 = [0,           p.gamma.hs, p.gamma.ha, p.gamma.hf, 0, 0;
          p.gamma.sh,  0,          0,          0,          0, 0;
          p.gamma.ah,  0,          0,          0,          0, 0;
          p.gamma.fh,  0,          0,          0,          0, 0;
          p.gamma.dh,  0,          0,          0,          0, 0;
          0,           0,          0,          0,          0, 0];

A = [zeros(6), eye(6);
     -diag(p.k) + Gamma0, -diag(p.c)];
B = [zeros(6,4); p.R];

Q = diag([3, 10, 12, 15, 8, 5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
% A lower input penalty permits a clearer, but still bounded, LQR response.
W = 0.1 * eye(4);
[p.Klqr, P, closedLoopPoles] = lqr(A, B, Q, W);

fprintf('Open-loop poles:\n');
disp(eig(A));
fprintf('Closed-loop LQR poles:\n');
disp(closedLoopPoles);
fprintf('LQR gain K (4 x 12):\n');
disp(p.Klqr);

%% 3. Simulate the nonlinear emotion model
tspan = linspace(0, 40, 2001);          % 20-minute simulation
x0 = zeros(12,1);                       % baseline equilibrium
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

[tFree, xFree] = ode45(@(t,x) emotionDynamics(t, x, p, false), ...
                         tspan, x0, opts);
[tLqr, xLqr] = ode45(@(t,x) emotionDynamics(t, x, p, true), ...
                       tspan, x0, opts);

% Recover physical robot commands and control deviations for plotting.
uLqr = zeros(numel(tLqr), 4);
uTildeLqr = zeros(numel(tLqr), 4);
for n = 1:numel(tLqr)
    [uNow, uTildeNow] = robotControl(xLqr(n,:).', p, true);
    uLqr(n,:) = uNow.';
    uTildeLqr(n,:) = uTildeNow.';
end

%% 4. Plot emotion deviations
emotionNames = {'Happiness', 'Sadness', 'Anger', 'Fear', 'Disgust', 'Surprise'};
figComparison = figure('Color', 'w', 'Name', 'Emotion deviations', ...
    'Position', [100, 100, 1100, 700]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:6
    nexttile;
    plot(tFree, xFree(:,i), 'k--', 'LineWidth', 1.5); hold on;
    plot(tLqr, xLqr(:,i), 'b-', 'LineWidth', 1.8);
    xline(2, ':', 'Event ends', 'LabelVerticalAlignment', 'bottom');
    grid on;
    xlabel('Time (min)');
    ylabel('\Delta emotion');
    title(emotionNames{i});
    legend('Uncontrolled', 'Saturated LQR', 'Location', 'best');
end

% print(figComparison, fullfile(outputFolder, 'open_loop_lqr_comparison.png'), '-dpng', '-r300');
% print(figComparison, fullfile(outputFolder, 'open_loop_lqr_comparison.pdf'), '-dpdf', '-r300');

%% 5. Export open-loop-only figure
% Open-loop-only figure for direct inclusion in the simulation-results section.
figOpenLoop = figure('Color', 'w', 'Name', 'Open-loop emotion deviations', ...
    'Position', [100, 100, 1100, 700]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:6
    nexttile;
    plot(tFree, xFree(:,i), 'k-', 'LineWidth', 1.8); hold on;
    xline(2, ':', 'Event ends', 'LabelVerticalAlignment', 'bottom');
    yline(0, ':', 'Color', [0.45, 0.45, 0.45]);
    grid on;
    xlabel('Time (min)');
    ylabel('\Delta emotion');
    title(emotionNames{i});
end

print(figOpenLoop, fullfile(outputFolder, 'open_loop_emotions.png'), '-dpng', '-r300');
% print(figOpenLoop, fullfile(outputFolder, 'open_loop_emotions.pdf'), '-dpdf', '-r300');

%% 6. Plot robot commands
inputNames = {'Support', 'Predictability', 'Adaptation', 'Repair'};
figRobot = figure('Color', 'w', 'Name', 'Robot behaviour commands', ...
    'Position', [100, 100, 1000, 650]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:4
    nexttile;
    plot(tLqr, uLqr(:,j), 'r-', 'LineWidth', 1.8); hold on;
    yline(p.u0(j), 'k--', 'Nominal');
    grid on;
    ylim([-0.05, 1.05]);
    xlabel('Time (min)');
    ylabel('Command level');
    title(inputNames{j});
end

% print(figRobot, fullfile(outputFolder, 'lqr_robot_commands.png'), '-dpng', '-r300');
% print(figRobot, fullfile(outputFolder, 'lqr_robot_commands.pdf'), '-dpdf', '-r300');

%% 7. Compute and print performance indices
eventEnd = 2;                  % Medication-reminder event ends at 2 min
settlingFraction = 0.02;       % Two-percent settling band
positiveEmotionIndices = [2, 3, 4, 6];  % sadness, anger, fear, surprise
positiveEmotionNames = emotionNames(positiveEmotionIndices);

% Positive peak comparison for the emotions directly excited by the scenario.
positivePeakFree = max(xFree(:,positiveEmotionIndices), [], 1).';
positivePeakLqr = max(xLqr(:,positiveEmotionIndices), [], 1).';
positivePeakReduction = 100 * (positivePeakFree - positivePeakLqr) ./ positivePeakFree;
peakComparison = table(positiveEmotionNames.', positivePeakFree, positivePeakLqr, ...
    positivePeakReduction, 'VariableNames', ...
    {'Emotion', 'OpenLoopPeak', 'LQRPeak', 'PeakReductionPercent'});

% Happiness is assessed by its minimum deviation, not by absolute peak error.
minimumHappinessOpenLoop = min(xFree(:,1));
minimumHappinessLqr = min(xLqr(:,1));
happinessImprovement = minimumHappinessLqr - minimumHappinessOpenLoop;
happinessComparison = table(minimumHappinessOpenLoop, minimumHappinessLqr, ...
    happinessImprovement, 'VariableNames', ...
    {'MinimumOpenLoop', 'MinimumLQR', 'Improvement'});

% Integral of absolute error for all emotion deviations.
iaeOpenLoop = trapz(tFree, abs(xFree(:,1:6))).';
iaeLqr = trapz(tLqr, abs(xLqr(:,1:6))).';
iaeReduction = 100 * (iaeOpenLoop - iaeLqr) ./ max(iaeOpenLoop, eps);
iaeComparison = table(emotionNames.', iaeOpenLoop, iaeLqr, iaeReduction, ...
    'VariableNames', {'Emotion', 'IAEOpenLoop', 'IAELQR', 'IAEReductionPercent'});

% Settling time is measured from the end of the two-minute event.
settlingTimeOpenLoop = zeros(6,1);
settlingTimeLqr = zeros(6,1);
for i = 1:6
    settlingTimeOpenLoop(i) = settlingTimeAfterEvent( ...
        tFree, xFree(:,i), eventEnd, settlingFraction);
    settlingTimeLqr(i) = settlingTimeAfterEvent( ...
        tLqr, xLqr(:,i), eventEnd, settlingFraction);
end
settlingComparison = table(emotionNames.', settlingTimeOpenLoop, settlingTimeLqr, ...
    'VariableNames', {'Emotion', 'OpenLoopSettlingTimeMin', 'LQRSettlingTimeMin'});

% Total controller effort and maximum physical robot-command levels.
controlEffort = trapz(tLqr, sum(uTildeLqr.^2, 2));
maximumRobotCommand = max(uLqr, [], 1).';
robotControlSummary = table(inputNames.', p.u0, maximumRobotCommand, ...
    'VariableNames', {'RobotInput', 'NominalLevel', 'MaximumLevel'});

fprintf('\nPositive-peak comparison (sadness, anger, fear, surprise):\n');
disp(peakComparison);
fprintf('\nHappiness protection measure:\n');
disp(happinessComparison);
fprintf('\nIntegral-of-absolute-error comparison:\n');
disp(iaeComparison);
fprintf('\nPost-event settling-time comparison (minutes):\n');
disp(settlingComparison);
fprintf('\nRobot-control summary:\n');
disp(robotControlSummary);
fprintf('Total LQR control effort: %.6e\n', controlEffort);

lqrEmotionDeviation = abs(xLqr(:,1:6));
freeEmotionDeviation = abs(xFree(:,1:6));
if any(lqrEmotionDeviation(:) > 1) || any(freeEmotionDeviation(:) > 1)
    warning(['At least one normalised emotion deviation exceeded one. ', ...
             'Review the scenario amplitudes and parameter values.']);
end

%% Local functions
function dx = emotionDynamics(t, x, p, useLqr)
    e = x(1:6);
    de = x(7:12);
    z = scenarioInputs(t);
    [~, uTilde] = robotControl(x, p, useLqr);

    % Happiness forcing term
    Fh = p.b.h(1)*z.P + p.b.h(2)*z.Ch + p.b.h(3)*z.Sh ...
       + p.gamma.hs*e(2) + p.gamma.ha*e(3) + p.gamma.hf*e(4) ...
       + p.gamma.hq*z.nu*e(6) + p.R(1,:)*uTilde;

    % Sadness forcing term
    Fs = p.b.s(1)*z.Ls + p.b.s(2)*z.Cs + p.b.s(3)*z.Ms ...
       - p.b.s(4)*z.Rs + p.gamma.sh*e(1) + p.R(2,:)*uTilde;

    % Anger forcing term
    Fa = p.b.a(1)*z.Ga + p.b.a(2)*z.Ba + p.b.a(3)*z.Ua ...
       - p.b.a(4)*z.Ra + p.gamma.ah*e(1) + p.R(3,:)*uTilde;

    % Fear forcing term
    Ff = p.b.f(1)*z.Tf + p.b.f(2)*z.Cf + p.b.f(3)*z.Mf ...
       - p.b.f(4)*z.Sf + p.gamma.fh*e(1) + p.R(4,:)*uTilde;

    % Disgust forcing term
    Fd = p.b.d(1)*z.Vd + p.b.d(2)*z.Md + p.b.d(3)*z.Nd ...
       - p.b.d(4)*z.Hd + p.gamma.dh*e(1) + p.R(5,:)*uTilde;

    % Surprise forcing term
    Fq = p.b.q(1)*z.Iq + p.b.q(2)*z.Uq - p.b.q(3)*z.Eq ...
       + p.R(6,:)*uTilde;

    F = [Fh; Fs; Fa; Ff; Fd; Fq];
    dde = -p.c .* de - p.k .* e + F;
    dde(1) = dde(1) - p.beta_h*e(1)^3;

    dx = [de; dde];
end

function [u, uTilde] = robotControl(x, p, useLqr)
    if useLqr
        uUnsaturated = p.u0 - p.Klqr*x;
        u = min(max(uUnsaturated, 0), 1);   % Element-wise saturation
        uTilde = u - p.u0;
    else
        u = p.u0;
        uTilde = zeros(4,1);
    end
end

function z = scenarioInputs(t)
    % Smooth two-minute pulse for an unexpected and unclear reminder.
    if t >= 0 && t <= 2
        pulse = sin(pi*t/2)^2;
    else
        pulse = 0;
    end

    % Initialise all centred appraisal inputs to zero.
    z = struct('P',0,'Ch',0,'Sh',0, ...
               'Ls',0,'Cs',0,'Ms',0,'Rs',0, ...
               'Ga',0,'Ba',0,'Ua',0,'Ra',0, ...
               'Tf',0,'Cf',0,'Mf',0,'Sf',0, ...
               'Vd',0,'Md',0,'Nd',0,'Hd',0, ...
               'Iq',0,'Uq',0,'Eq',0,'nu',0);

    % Active scenario inputs.
    z.Iq = 0.8*pulse;     % unexpectedness / Bayesian-surprise signal
    z.Uq = 0.6*pulse;     % uncertainty about the reminder
    z.Tf = 0.4*pulse;     % perceived threat or concern
    z.Ga = 0.3*pulse;     % perceived goal obstruction
    z.Cs = 0.2*pulse;     % low perceived control
    z.nu = -0.5*pulse;    % unpleasant surprise valence
end

function recoveryTime = settlingTimeAfterEvent(t, y, eventEnd, fraction)
    peakMagnitude = max(abs(y));
    if peakMagnitude < 1e-12
        recoveryTime = 0;
        return;
    end

    threshold = fraction * peakMagnitude;
    startIndex = find(t >= eventEnd, 1, 'first');
    recoveryTime = NaN;
    for k = startIndex:numel(t)
        if all(abs(y(k:end)) <= threshold)
            recoveryTime = t(k) - eventEnd;
            return;
        end
    end
end
