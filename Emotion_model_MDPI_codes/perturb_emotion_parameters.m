function pPerturbed = perturb_emotion_parameters(pNominal, uncertaintyLevel)
%PERTURB_EMOTION_PARAMETERS Applies independent bounded parameter errors.
% Every nonzero tunable coefficient is multiplied by 1 + delta, where
% delta is uniformly sampled from [-uncertaintyLevel, uncertaintyLevel].
% Zero entries are retained as zero, including the medication-scenario
% disgust row of the robot-input matrix.

if uncertaintyLevel < 0 || uncertaintyLevel >= 1
    error('uncertaintyLevel must be in the interval [0, 1).');
end

pPerturbed = pNominal;
pPerturbed.c = perturb_nonzero(pNominal.c, uncertaintyLevel);
pPerturbed.k = perturb_nonzero(pNominal.k, uncertaintyLevel);
pPerturbed.beta_h = perturb_nonzero(pNominal.beta_h, uncertaintyLevel);
pPerturbed.R = perturb_nonzero(pNominal.R, uncertaintyLevel);

pPerturbed.gamma.hs = perturb_nonzero(pNominal.gamma.hs, uncertaintyLevel);
pPerturbed.gamma.sh = perturb_nonzero(pNominal.gamma.sh, uncertaintyLevel);
pPerturbed.gamma.ha = perturb_nonzero(pNominal.gamma.ha, uncertaintyLevel);
pPerturbed.gamma.ah = perturb_nonzero(pNominal.gamma.ah, uncertaintyLevel);
pPerturbed.gamma.hf = perturb_nonzero(pNominal.gamma.hf, uncertaintyLevel);
pPerturbed.gamma.fh = perturb_nonzero(pNominal.gamma.fh, uncertaintyLevel);
pPerturbed.gamma.dh = perturb_nonzero(pNominal.gamma.dh, uncertaintyLevel);
pPerturbed.gamma.hq = perturb_nonzero(pNominal.gamma.hq, uncertaintyLevel);

pPerturbed.b.h = perturb_nonzero(pNominal.b.h, uncertaintyLevel);
pPerturbed.b.s = perturb_nonzero(pNominal.b.s, uncertaintyLevel);
pPerturbed.b.a = perturb_nonzero(pNominal.b.a, uncertaintyLevel);
pPerturbed.b.f = perturb_nonzero(pNominal.b.f, uncertaintyLevel);
pPerturbed.b.d = perturb_nonzero(pNominal.b.d, uncertaintyLevel);
pPerturbed.b.q = perturb_nonzero(pNominal.b.q, uncertaintyLevel);
end


function value = perturb_nonzero(valueNominal, uncertaintyLevel)
mask = valueNominal ~= 0;
value = valueNominal;
randomScale = 1 + uncertaintyLevel * (2 * rand(size(valueNominal)) - 1);
value(mask) = valueNominal(mask) .* randomScale(mask);
end

% function value = perturb_nonzero(valueNominal, uncertaintyLevel)
% mask = valueNominal ~= 0;
% value = valueNominal;
% value(mask) = valueNominal(mask) .* ...
%     (1 + uncertaintyLevel * (2 * rand(nnz(mask),1) - 1));
% end
