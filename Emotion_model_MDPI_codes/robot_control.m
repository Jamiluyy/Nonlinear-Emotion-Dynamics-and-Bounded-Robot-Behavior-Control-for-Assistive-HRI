function [u, uTilde] = robot_control(x, p, useLqr)
%ROBOT_CONTROL Computes physical bounded commands and centred input values.

if useLqr
    u = min(max(p.u0 - p.Klqr*x, 0), 1);
    uTilde = u - p.u0;
else
    u = p.u0;
    uTilde = zeros(4,1);
end
end
