function dx = emotion_dynamics_with_input(t, x, p, uTilde)
%EMOTION_DYNAMICS_WITH_INPUT Full nonlinear model with an explicit input.
% This shared helper prevents the delayed-feedback and saturation studies
% from redefining a separate version of the emotion dynamics.

e = x(1:6);
de = x(7:12);
z = scenario_inputs(t, p);

Fh = p.b.h(1)*z.P + p.b.h(2)*z.Ch + p.b.h(3)*z.Sh ...
   + p.gamma.hs*e(2) + p.gamma.ha*e(3) + p.gamma.hf*e(4) ...
   + p.gamma.hq*z.nu*e(6) + p.R(1,:)*uTilde;
Fs = p.b.s(1)*z.Ls + p.b.s(2)*z.Cs + p.b.s(3)*z.Ms ...
   - p.b.s(4)*z.Rs + p.gamma.sh*e(1) + p.R(2,:)*uTilde;
Fa = p.b.a(1)*z.Ga + p.b.a(2)*z.Ba + p.b.a(3)*z.Ua ...
   - p.b.a(4)*z.Ra + p.gamma.ah*e(1) + p.R(3,:)*uTilde;
Ff = p.b.f(1)*z.Tf + p.b.f(2)*z.Cf + p.b.f(3)*z.Mf ...
   - p.b.f(4)*z.Sf + p.gamma.fh*e(1) + p.R(4,:)*uTilde;
Fd = p.b.d(1)*z.Vd + p.b.d(2)*z.Md + p.b.d(3)*z.Nd ...
   - p.b.d(4)*z.Hd + p.gamma.dh*e(1) + p.R(5,:)*uTilde;
Fq = p.b.q(1)*z.Iq + p.b.q(2)*z.Uq - p.b.q(3)*z.Eq ...
   + p.R(6,:)*uTilde;

dde = -p.c .* de - p.k .* e + [Fh; Fs; Fa; Ff; Fd; Fq];
dde(1) = dde(1) - p.beta_h*e(1)^3;
dx = [de; dde];
end
