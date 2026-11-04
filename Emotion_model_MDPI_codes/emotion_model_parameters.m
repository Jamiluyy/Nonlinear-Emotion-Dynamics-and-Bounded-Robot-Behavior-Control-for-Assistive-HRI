function p = emotion_model_parameters(scenarioId)
%EMOTION_MODEL_PARAMETERS All tunable model, controller, and scenario values.

p.c = 0.8 * ones(6,1);
p.k = 0.16 * ones(6,1);
p.beta_h = 0.008;

p.gamma.hs = -0.006; p.gamma.sh = -0.006;
p.gamma.ha = -0.006; p.gamma.ah = -0.005;
p.gamma.hf = -0.008; p.gamma.fh = -0.007;
p.gamma.dh = -0.004; p.gamma.hq =  0.004;

% Rows: happiness, sadness, anger, fear, disgust, surprise.
% Columns: support, predictability, adaptation, repair.
p.R = [ 0.008,  0.006,  0.007,  0.005;
       -0.005, -0.004, -0.007, -0.006;
       -0.006, -0.007, -0.005, -0.008;
       -0.007, -0.008, -0.006, -0.004;
        0.000,  0.000,  0.000,  0.000;
        0.000, -0.008,  0.000, -0.005];

p.b.h = [0.030, 0.020, 0.025];
p.b.s = [0.030, 0.025, 0.020, 0.030];
p.b.a = [0.030, 0.025, 0.025, 0.030];
p.b.f = [0.040, 0.030, 0.020, 0.040];
p.b.d = [0.030, 0.020, 0.020, 0.030];
p.b.q = [0.040, 0.035, 0.040];

p.u0 = [0.50; 0.50; 0.50; 0.25];
p.Qlqr = diag([3, 10, 12, 15, 8, 5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
p.Wlqr = 0.1 * eye(4);

p.scenario = scenarioId;
p.simulationEnd = 40;
p.numberOfSamples = 4001;
p.settlingFraction = 0.02;
p.emotionNames = {'Happiness', 'Sadness', 'Anger', 'Fear', 'Disgust', 'Surprise'};
p.inputNames = {'Support', 'Predictability', 'Adaptation', 'Repair'};

switch scenarioId
    case 1
        p.eventEndTimes = 2;
        p.settlingReferenceTime = 2;
        p.scenarioTag = 'scenario1_single_reminder';
    case 2
        p.eventEndTimes = [2, 10];
        p.settlingReferenceTime = 10;
        p.scenarioTag = 'scenario2_repeated_reminder';
    otherwise
        error('scenarioId must be either 1 or 2.');
end
end
