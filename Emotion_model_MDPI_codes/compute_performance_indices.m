function metrics = compute_performance_indices(tFree, xFree, tLqr, xLqr, uLqr, uTildeLqr, p)
%COMPUTE_PERFORMANCE_INDICES Calculates response and control metrics.

positiveIndices = [2, 3, 4, 6];
positiveNames = p.emotionNames(positiveIndices);
peakFree = max(xFree(:,positiveIndices), [], 1).';
peakLqr = max(xLqr(:,positiveIndices), [], 1).';
peakReduction = 100 * (peakFree - peakLqr) ./ peakFree;
metrics.peakComparison = table(positiveNames.', peakFree, peakLqr, peakReduction, ...
    'VariableNames', {'Emotion', 'OpenLoopPeak', 'LQRPeak', 'PeakReductionPercent'});

minOpen = min(xFree(:,1));
minLqr = min(xLqr(:,1));
metrics.happinessComparison = table(minOpen, minLqr, minLqr-minOpen, ...
    'VariableNames', {'MinimumOpenLoop', 'MinimumLQR', 'Improvement'});

iaeOpen = trapz(tFree, abs(xFree(:,1:6))).';
iaeLqr = trapz(tLqr, abs(xLqr(:,1:6))).';
metrics.iaeComparison = table(p.emotionNames.', iaeOpen, iaeLqr, ...
    100*(iaeOpen-iaeLqr)./max(iaeOpen,eps), 'VariableNames', ...
    {'Emotion', 'IAEOpenLoop', 'IAELQR', 'IAEReductionPercent'});

settleOpen = zeros(6,1); settleLqr = zeros(6,1);
for i = 1:6
    settleOpen(i) = settling_time_after_event(tFree, xFree(:,i), ...
        p.settlingReferenceTime, p.settlingFraction);
    settleLqr(i) = settling_time_after_event(tLqr, xLqr(:,i), ...
        p.settlingReferenceTime, p.settlingFraction);
end
metrics.settlingComparison = table(p.emotionNames.', settleOpen, settleLqr, ...
    'VariableNames', {'Emotion', 'OpenLoopSettlingTimeMin', 'LQRSettlingTimeMin'});

metrics.controlEffort = trapz(tLqr, sum(uTildeLqr.^2,2));
metrics.robotControlSummary = table(p.inputNames.', p.u0, max(uLqr,[],1).', ...
    'VariableNames', {'RobotInput', 'NominalLevel', 'MaximumLevel'});
end
