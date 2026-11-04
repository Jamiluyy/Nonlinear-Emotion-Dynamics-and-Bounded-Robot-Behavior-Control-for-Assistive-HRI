function eq = find_emotion_equilibria(p)
%FIND_EMOTION_EQUILIBRIA Equilibrium points of the autonomous emotion model.
%
% The analysis assumes:
%   - zero appraisal and disturbance inputs,
%   - nominal robot behaviour, so utilde_r = 0,
%   - nu_q = 0,
%   - beta_h > 0.
%
% Output:
%   eq.mu                 Effective happiness restoring coefficient.
%   eq.numberOfEquilibria Number of isolated equilibrium points.
%   eq.classification     Text description of the equilibrium regime.
%   eq.states             6-by-N matrix; each column is one equilibrium.
%   eq.labels             Labels corresponding to the columns of eq.states.
%   eq.residualNorms      Equilibrium-equation residual norm for each point.

% Emotion order: 1 = happiness, 2 = sadness, 3 = anger, 4 = fear, 5 = disgust, 6 = surprise.

k = p.k(:);

if numel(k) ~= 6
    error('p.k must contain six restoring coefficients.');
end

if any(k <= 0)
    error('All restoring coefficients k_i must be strictly positive.');
end

if p.beta_h <= 0
    error('This equilibrium analysis assumes beta_h > 0.');
end

% Extract the retained emotion-coupling coefficients.
ghs = p.gamma.hs;
gsh = p.gamma.sh;

gha = p.gamma.ha;
gah = p.gamma.ah;

ghf = p.gamma.hf;
gfh = p.gamma.fh;

gdh = p.gamma.dh;

% Effective happiness restoring coefficient:
% mu = k_h - (gamma_hs*gamma_sh)/k_s
%            - (gamma_ha*gamma_ah)/k_a
%            - (gamma_hf*gamma_fh)/k_f.
eq.mu = k(1) ...
      - (ghs * gsh) / k(2) ...
      - (gha * gah) / k(3) ...
      - (ghf * gfh) / k(4);

% These coefficients reconstruct the other equilibrium states
% once an equilibrium value of happiness has been found.
equilibriumDirection = [1;
                        gsh / k(2);
                        gah / k(3);
                        gfh / k(4);
                        gdh / k(5);
                        0];

tolerance = 1e-12;

% The origin is always an equilibrium.
origin = zeros(6,1);

if eq.mu > tolerance

    % mu > 0 and beta_h > 0:
    % only the baseline equilibrium exists.
    eq.states = origin;
    eq.labels = {'Baseline'};
    eq.numberOfEquilibria = 1;
    eq.classification = ...
        'Unique baseline equilibrium: mu > 0.';

elseif abs(eq.mu) <= tolerance

    % mu = 0:
    % the origin remains the only equilibrium, but it is degenerate.
    eq.states = origin;
    eq.labels = {'Baseline'};
    eq.numberOfEquilibria = 1;
    eq.classification = ...
        'Degenerate baseline equilibrium: mu is approximately zero.';

else

    % mu < 0 and beta_h > 0:
    % two nonzero equilibrium points appear in addition to the origin.
    hMagnitude = sqrt(-eq.mu / p.beta_h);

    positiveEquilibrium =  hMagnitude * equilibriumDirection;
    negativeEquilibrium = -hMagnitude * equilibriumDirection;

    eq.states = [origin, positiveEquilibrium, negativeEquilibrium];
    eq.labels = {'Baseline', 'PositiveNonzero', 'NegativeNonzero'};
    eq.numberOfEquilibria = 3;
    eq.classification = ...
        'Three equilibria: baseline plus two nonzero equilibria.';
end

% Verify every returned equilibrium by evaluating the autonomous
% nonlinear equilibrium equations.
numberOfPoints = size(eq.states, 2);
eq.residualNorms = zeros(1, numberOfPoints);

for pointIndex = 1:numberOfPoints
    residual = autonomous_equilibrium_residual(eq.states(:, pointIndex), p);
    eq.residualNorms(pointIndex) = norm(residual, 2);
end

eq.stateNames = {'Happiness', 'Sadness', 'Anger', ...
                 'Fear', 'Disgust', 'Surprise'};

end


function residual = autonomous_equilibrium_residual(e, p)
%AUTONOMOUS_EQUILIBRIUM_RESIDUAL Residual of the zero-input equilibrium equations.

h = e(1); s = e(2); a = e(3); f = e(4); d = e(5); q = e(6);

residual = zeros(6,1);

% Happiness equation.
residual(1) = p.k(1) * h + p.beta_h * h^3 ...
            - p.gamma.hs * s ...
            - p.gamma.ha * a ...
            - p.gamma.hf * f;

% Sadness, anger, fear, disgust, and surprise equations.
residual(2) = p.k(2) * s - p.gamma.sh * h;
residual(3) = p.k(3) * a - p.gamma.ah * h;
residual(4) = p.k(4) * f - p.gamma.fh * h;
residual(5) = p.k(5) * d - p.gamma.dh * h;
residual(6) = p.k(6) * q;

end