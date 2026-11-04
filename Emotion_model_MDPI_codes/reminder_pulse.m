function pulse = reminder_pulse(t, startTime, endTime)
%REMINDER_PULSE Smooth unit-amplitude reminder pulse over a specified interval.

if t >= startTime && t <= endTime
    pulse = sin(pi*(t - startTime)/(endTime - startTime))^2;
else
    pulse = 0;
end
end
