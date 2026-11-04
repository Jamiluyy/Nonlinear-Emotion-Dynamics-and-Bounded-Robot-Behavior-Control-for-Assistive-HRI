%% run_saturation_ablation_fixed.m
% Saturation-ablation study using the shared emotion model and inputs.
% The event-scale-one row is the canonical nominal Scenario 2 simulation.

clear; clc; close all;
thisFolder = fileparts(mfilename('fullpath'));
if isempty(thisFolder), thisFolder = pwd; end
addpath(thisFolder);
if exist('lqr','file') ~= 2, error('The MATLAB Control System Toolbox is required.'); end

outputFolder = fullfile(thisFolder,'validation_results');
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end

scenarioId = 2; eventScales = [1.0,2.0,3.0]; timeStep = 0.01;
stateBound = 1.0; saturationTolerance = 1e-8;
controllerNames = {'Open loop','Unbounded LQR','Saturated LQR'};
adverseIndices = [2,3,4,6];

pNominal = emotion_model_parameters(scenarioId);
[~,~,pNominal.Klqr,~] = design_lqr_controller(pNominal);
nScale = numel(eventScales); nControl = numel(controllerNames);
peakAdverse = zeros(nScale,nControl); adverseIAE = zeros(nScale,nControl);
controlEffort = zeros(nScale,nControl); minimumHappiness = zeros(nScale,nControl);
maximumState = zeros(nScale,nControl); boundedResponse = false(nScale,nControl);
rawViolationPercent = zeros(nScale,nControl); boundaryPercent = zeros(nScale,nControl);
trajectoryDifference = zeros(nScale,1); nominalConsistencyPassed = false(nScale,1);

for s = 1:nScale
    scale = eventScales(s);
    if scale == 1
        canonical = simulate_emotion_case(pNominal);
        openLoopResult = canonical_open_loop_result(canonical,pNominal);
        saturatedResult = canonical_lqr_result(canonical,pNominal,'saturated');
        unboundedResult = canonical_lqr_result(canonical,pNominal,'unbounded');
    else
        p = pNominal; p.eventAmplitudeScale = scale;
        openLoopResult = simulate_fixed_step_case(p,'open',timeStep);
        unboundedResult = simulate_fixed_step_case(p,'unbounded',timeStep);
        saturatedResult = simulate_fixed_step_case(p,'saturated',timeStep);
    end
    resultSet = {openLoopResult,unboundedResult,saturatedResult};
    for c = 1:nControl
        r = resultSet{c};
        peakAdverse(s,c) = max(r.x(:,adverseIndices),[],'all');
        adverseIAE(s,c) = sum(trapz(r.t,abs(r.x(:,adverseIndices))));
        controlEffort(s,c) = trapz(r.t,sum(r.uTilde.^2,2));
        minimumHappiness(s,c) = min(r.x(:,1));
        maximumState(s,c) = max(abs(r.x(:)));
        boundedResponse(s,c) = all(isfinite(r.x(:))) && maximumState(s,c) <= stateBound;
        rawViolationPercent(s,c) = 100*mean(any(r.uRaw<0 | r.uRaw>1,2));
        boundaryPercent(s,c) = 100*mean(any(r.u<=saturationTolerance | r.u>=1-saturationTolerance,2));
    end
    trajectoryDifference(s) = max(abs(unboundedResult.x(:)-saturatedResult.x(:)));
    inactive = rawViolationPercent(s,2)==0 && boundaryPercent(s,3)==0;
    nominalConsistencyPassed(s) = ~inactive || trajectoryDifference(s)<1e-10;
end

eventScaleColumn = repelem(eventScales(:),nControl);
controllerColumn = repmat(controllerNames(:),nScale,1);
resultTable = table(eventScaleColumn,controllerColumn,reshape(peakAdverse.',[],1), ...
    reshape(adverseIAE.',[],1),reshape(minimumHappiness.',[],1), ...
    reshape(controlEffort.',[],1),reshape(rawViolationPercent.',[],1), ...
    reshape(boundaryPercent.',[],1),reshape(maximumState.',[],1), ...
    reshape(boundedResponse.',[],1),'VariableNames',{'EventScale','Controller', ...
    'PeakAdverseEmotion','AdverseEmotionIAE','MinimumHappiness','ControlEffort', ...
    'RawCommandViolationPercent','CommandBoundaryPercent','MaximumStateMagnitude','BoundedResponse'});
writetable(resultTable,fullfile(outputFolder,'saturation_ablation_fixed_step_results.csv'));

consistencyTable = table(eventScales(:),trajectoryDifference,rawViolationPercent(:,2), ...
    boundaryPercent(:,3),nominalConsistencyPassed,'VariableNames',{'EventScale', ...
    'UnboundedVsSaturatedStateDifference','UnboundedCommandViolationPercent', ...
    'SaturatedCommandBoundaryPercent','ConsistencyCheckPassed'});
writetable(consistencyTable,fullfile(outputFolder,'saturation_ablation_consistency_check.csv'));

labels = arrayfun(@(v)sprintf('Scale %.1f',v),eventScales,'UniformOutput',false);
figure('Color','w','Position',[100 100 1150 760]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; bar(peakAdverse); grid on; box on; xticklabels(labels); ylabel('Peak emotion deviation'); title('Maximum adverse-emotion response'); legend(controllerNames,'Location','northwest');
nexttile; bar(adverseIAE); grid on; box on; xticklabels(labels); ylabel('Aggregate emotion IAE'); title('Integrated adverse-emotion response'); legend(controllerNames,'Location','northwest');
nexttile; bar(controlEffort(:,2:3)); grid on; box on; xticklabels(labels); ylabel('Control effort'); title('Control effort under LQR feedback'); legend({'Unbounded LQR','Saturated LQR'},'Location','northwest');
nexttile; bar([rawViolationPercent(:,2),boundaryPercent(:,3)]); grid on; box on; xticklabels(labels); ylabel('Time samples (%)'); title('Effect of physical command limits'); legend({'Unbounded command outside [0,1]','Saturated command at boundary'},'Location','northwest');
exportgraphics(gcf,fullfile(outputFolder,'saturation_ablation_fixed_step.png'),'Resolution',300);

fprintf('\nSaturation-ablation study\n'); disp(resultTable); fprintf('\nConsistency check\n'); disp(consistencyTable);

function result = simulate_fixed_step_case(p,mode,dt)
t = (0:dt:p.simulationEnd).'; x = zeros(numel(t),12);
for k = 1:numel(t)-1
    xk = x(k,:).';
    k1 = dynamics(t(k),xk,p,mode); k2 = dynamics(t(k)+dt/2,xk+dt*k1/2,p,mode);
    k3 = dynamics(t(k)+dt/2,xk+dt*k2/2,p,mode); k4 = dynamics(t(k)+dt,xk+dt*k3,p,mode);
    x(k+1,:) = (xk+dt*(k1+2*k2+2*k3+k4)/6).';
end
[u,uRaw,uTilde] = control_history(x,p,mode);
result = struct('t',t,'x',x,'u',u,'uRaw',uRaw,'uTilde',uTilde);
end

function dx = dynamics(t,x,p,mode)
[~,~,uTilde] = command(x,p,mode);
dx = emotion_dynamics_with_input(t,x,p,uTilde);
end

function [u,uRaw,uTilde] = command(x,p,mode)
switch mode
    case 'open', uRaw = p.u0; u = p.u0;
    case 'unbounded', uRaw = p.u0-p.Klqr*x; u = uRaw;
    case 'saturated', uRaw = p.u0-p.Klqr*x; u = min(max(uRaw,0),1);
end
uTilde = u-p.u0;
end

function [u,uRaw,uTilde] = control_history(x,p,mode)
n = size(x,1); u = zeros(n,4); uRaw = zeros(n,4); uTilde = zeros(n,4);
for k = 1:n
    [uk,rawk,tildek] = command(x(k,:).',p,mode);
    u(k,:) = uk.'; uRaw(k,:) = rawk.'; uTilde(k,:) = tildek.';
end
end

function result = canonical_open_loop_result(canonical,p)
n = numel(canonical.tFree); u = repmat(p.u0.',n,1);
result = struct('t',canonical.tFree,'x',canonical.xFree,'u',u,'uRaw',u,'uTilde',zeros(n,4));
end

function result = canonical_lqr_result(canonical,p,mode)
x = canonical.xLqr; t = canonical.tLqr; n = numel(t);
raw = repmat(p.u0.',n,1)-x*p.Klqr.';
if strcmp(mode,'unbounded'), u = raw; else, u = min(max(raw,0),1); end
result = struct('t',t,'x',x,'u',u,'uRaw',raw,'uTilde',u-repmat(p.u0.',n,1));
end
