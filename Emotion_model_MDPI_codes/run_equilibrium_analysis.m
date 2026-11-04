%% Stage 1: Equilibrium-point analysis
clear; clc;

%% Nominal parameter set
pNominal = emotion_model_parameters(1);

eqNominal = find_emotion_equilibria(pNominal);

fprintf('\n Nominal equilibrium analysis \n');
fprintf('mu = %.10f\n', eqNominal.mu);
fprintf('Number of equilibria = %d\n', eqNominal.numberOfEquilibria);
fprintf('%s\n\n', eqNominal.classification);

nominalEquilibriaTable = array2table(eqNominal.states, ...
    'VariableNames', eqNominal.labels, ...
    'RowNames', eqNominal.stateNames);

disp(nominalEquilibriaTable);

fprintf('Residual norms:\n');
for pointIndex = 1:eqNominal.numberOfEquilibria
    fprintf('  %s: %.3e\n', ...
        eqNominal.labels{pointIndex}, ...
        eqNominal.residualNorms(pointIndex));
end


%% Mathematical test case: create a three-equilibrium regime
% This is only a verification test for the equilibrium function.
% It is not yet a literature-supported parameter set.
pTest = pNominal;
pTest.k(1) = 5e-4;

eqTest = find_emotion_equilibria(pTest);

fprintf('\n For the case with reduced happiness restoring coefficient \n');
fprintf('mu = %.10f\n', eqTest.mu);
fprintf('Number of equilibria = %d\n', eqTest.numberOfEquilibria);
fprintf('%s\n\n', eqTest.classification);

testEquilibriaTable = array2table(eqTest.states, ...
    'VariableNames', eqTest.labels, ...
    'RowNames', eqTest.stateNames);

disp(testEquilibriaTable);

fprintf('Residual norms:\n');
for pointIndex = 1:eqTest.numberOfEquilibria
    fprintf('  %s: %.3e\n', ...
        eqTest.labels{pointIndex}, ...
        eqTest.residualNorms(pointIndex));
end