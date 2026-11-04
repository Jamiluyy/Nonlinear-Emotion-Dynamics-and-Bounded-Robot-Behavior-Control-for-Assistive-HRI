function result = simulate_emotion_case(p)
%SIMULATE_EMOTION_CASE Simulates open-loop and saturated-LQR responses.
% If p.Klqr is supplied, that fixed gain is used. Otherwise, an LQR gain
% is designed from the Qlqr and Wlqr matrices contained in p.

if ~isfield(p, 'Klqr') || isempty(p.Klqr)
    [A, B, p.Klqr, closedLoopPoles] = design_lqr_controller(p);
else
    [A, B, ~, closedLoopPoles] = design_lqr_controller(p);
end

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

result = struct('p', p, 'A', A, 'B', B, ...
    'closedLoopPoles', closedLoopPoles, ...
    'tFree', tFree, 'xFree', xFree, 'tLqr', tLqr, 'xLqr', xLqr, ...
    'uLqr', uLqr, 'uTildeLqr', uTildeLqr, 'metrics', metrics);
end
