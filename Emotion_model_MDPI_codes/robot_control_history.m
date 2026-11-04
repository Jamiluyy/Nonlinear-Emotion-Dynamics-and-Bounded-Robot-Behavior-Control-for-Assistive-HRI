function [uHistory, uTildeHistory] = robot_control_history(t, x, p)
%ROBOT_CONTROL_HISTORY Evaluates the controller over a simulated trajectory.

uHistory = zeros(numel(t), 4);
uTildeHistory = zeros(numel(t), 4);
for n = 1:numel(t)
    [u, uTilde] = robot_control(x(n,:).', p, true);
    uHistory(n,:) = u.';
    uTildeHistory(n,:) = uTilde.';
end
end
