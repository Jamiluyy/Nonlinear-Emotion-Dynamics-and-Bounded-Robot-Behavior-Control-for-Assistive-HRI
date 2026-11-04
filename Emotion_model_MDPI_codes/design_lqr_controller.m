function [A, B, Klqr, closedLoopPoles] = design_lqr_controller(p)
%DESIGN_LQR_CONTROLLER Constructs the baseline linear model and LQR gain.

Gamma0 = [0,           p.gamma.hs, p.gamma.ha, p.gamma.hf, 0, 0;
          p.gamma.sh,  0,          0,          0,          0, 0;
          p.gamma.ah,  0,          0,          0,          0, 0;
          p.gamma.fh,  0,          0,          0,          0, 0;
          p.gamma.dh,  0,          0,          0,          0, 0;
          0,           0,          0,          0,          0, 0];

A = [zeros(6), eye(6); -diag(p.k) + Gamma0, -diag(p.c)];
B = [zeros(6,4); p.R];
[Klqr, ~, closedLoopPoles] = lqr(A, B, p.Qlqr, p.Wlqr);
end
