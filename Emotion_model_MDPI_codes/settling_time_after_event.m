function recoveryTime = settling_time_after_event(t, y, eventEnd, fraction)
%SETTLING_TIME_AFTER_EVENT Returns time from final event end to 2%% recovery.

peakMagnitude = max(abs(y));
if peakMagnitude < 1e-12
    recoveryTime = 0;
    return;
end

threshold = fraction * peakMagnitude;
startIndex = find(t >= eventEnd, 1, 'first');
recoveryTime = NaN;
for k = startIndex:numel(t)
    if all(abs(y(k:end)) <= threshold)
        recoveryTime = t(k) - eventEnd;
        return;
    end
end
end
