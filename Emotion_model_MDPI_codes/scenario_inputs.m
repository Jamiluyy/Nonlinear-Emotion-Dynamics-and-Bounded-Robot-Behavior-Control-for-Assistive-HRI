function z = scenario_inputs(t, scenarioOrParameters)
%SCENARIO_INPUTS Returns centred appraisal inputs for the selected scenario.
% The function accepts either a scenario ID or the complete parameter
% structure. The latter option supports event-sensitivity studies while
% retaining the original nominal scenarios as the default case.

z = struct('P',0,'Ch',0,'Sh',0, ...
    'Ls',0,'Cs',0,'Ms',0,'Rs',0, ...
    'Ga',0,'Ba',0,'Ua',0,'Ra',0, ...
    'Tf',0,'Cf',0,'Mf',0,'Sf',0, ...
    'Vd',0,'Md',0,'Nd',0,'Hd',0, ...
    'Iq',0,'Uq',0,'Eq',0,'nu',0);

% Default values reproduce the submitted nominal scenarios exactly.
eventAmplitudeScale = 1.0;
eventDurationMin = 2.0;
secondReminderStartMin = 8.0;

if isstruct(scenarioOrParameters)
    p = scenarioOrParameters;
    scenarioId = p.scenario;

    if isfield(p, 'eventAmplitudeScale')
        eventAmplitudeScale = p.eventAmplitudeScale;
    end

    if isfield(p, 'eventDurationMin')
        eventDurationMin = p.eventDurationMin;
    end

    if isfield(p, 'secondReminderStartMin')
        secondReminderStartMin = p.secondReminderStartMin;
    end
else
    scenarioId = scenarioOrParameters;
end

switch scenarioId
    case 1
        p1 = reminder_pulse(t, 0, eventDurationMin);

        z.Iq = eventAmplitudeScale * 0.8 * p1;
        z.Uq = eventAmplitudeScale * 0.6 * p1;
        z.Tf = eventAmplitudeScale * 0.4 * p1;
        z.Ga = eventAmplitudeScale * 0.3 * p1;
        z.Cs = eventAmplitudeScale * 0.2 * p1;
        z.nu = -eventAmplitudeScale * 0.5 * p1;

    case 2
        p1 = reminder_pulse(t, 0, eventDurationMin);
        p2 = reminder_pulse(t, secondReminderStartMin, ...
            secondReminderStartMin + eventDurationMin);

        z.Iq = eventAmplitudeScale * (0.8 * p1 + 0.9 * p2);
        z.Uq = eventAmplitudeScale * (0.6 * p1 + 0.7 * p2);
        z.Tf = eventAmplitudeScale * (0.4 * p1 + 0.55 * p2);
        z.Ga = eventAmplitudeScale * (0.3 * p1 + 0.6 * p2);
        z.Cs = eventAmplitudeScale * (0.2 * p1 + 0.35 * p2);
        z.Ms = eventAmplitudeScale * 0.15 * p2;
        z.Mf = eventAmplitudeScale * 0.20 * p2;
        z.nu = -eventAmplitudeScale * (0.5 * p1 + 0.7 * p2);

    otherwise
        error('scenarioId must be either 1 or 2.');
end
end

% 
% function z = scenario_inputs(t, scenarioId)
% %SCENARIO_INPUTS Returns centred appraisal inputs for a selected scenario.
% 
% z = struct('P',0,'Ch',0,'Sh',0, ...
%            'Ls',0,'Cs',0,'Ms',0,'Rs',0, ...
%            'Ga',0,'Ba',0,'Ua',0,'Ra',0, ...
%            'Tf',0,'Cf',0,'Mf',0,'Sf',0, ...
%            'Vd',0,'Md',0,'Nd',0,'Hd',0, ...
%            'Iq',0,'Uq',0,'Eq',0,'nu',0);
% 
% switch scenarioId
%     case 1
%         p1 = reminder_pulse(t, 0, 2);
%         z.Iq = 0.8*p1; z.Uq = 0.6*p1; z.Tf = 0.4*p1;
%         z.Ga = 0.3*p1; z.Cs = 0.2*p1; z.nu = -0.5*p1;
%     case 2
%         p1 = reminder_pulse(t, 0, 2);
%         p2 = reminder_pulse(t, 8, 10);
%         z.Iq = 0.8*p1 + 0.9*p2;
%         z.Uq = 0.6*p1 + 0.7*p2;
%         z.Tf = 0.4*p1 + 0.55*p2;
%         z.Ga = 0.3*p1 + 0.6*p2;
%         z.Cs = 0.2*p1 + 0.35*p2;
%         z.Ms = 0.15*p2;
%         z.Mf = 0.20*p2;
%         z.nu = -0.5*p1 - 0.7*p2;
%     otherwise
%         error('scenarioId must be either 1 or 2.');
% end
% end
