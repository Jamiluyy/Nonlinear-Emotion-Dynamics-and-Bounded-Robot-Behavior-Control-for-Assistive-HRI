function dx = emotion_dynamics(t, x, p, useLqr)
%EMOTION_DYNAMICS Full nonlinear six-emotion model in grouped state order.

[~, uTilde] = robot_control(x, p, useLqr);
dx = emotion_dynamics_with_input(t, x, p, uTilde);
end
