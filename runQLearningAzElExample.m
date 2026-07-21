%% Fast parallel Q-learning example for a moving az/el mask
clear;
clc;
close all;

time_s = (0:0.5:75)';
azElData = createQLearningExampleAzElData(time_s);

startAzEl_deg = [-80,20];
goalAzEl_deg = [80,70];

options = struct;
options.azLim_deg = [-100,100];
options.elLim_deg = [5,85];
options.gridStep_deg = [2,2];
options.azRate_deg_s = 12;
options.elRate_deg_s = 12;
options.startTime_s = 0;
options.planningHorizon_s = 75;
options.goalHold_s = 2;
options.clearance_deg = 1.5;
options.temporalPadding_s = 0.5;

% Q-learning speed and convergence settings.
options.episodes = 4000;
options.useParallel = true;
options.numLearners = 0; % Use all workers in the current/default pool.
options.randomSeed = 7;
options.maxQTableMB = 512;

% Reward shaping and final collision-checked smoothing.
options.guidanceWeight = 3.0;
options.progressRewardWeight = 2.0;
options.turnPenalty = 0.75;
options.smoothPath = true;
options.smoothingMaxLookahead = 250;
options.verbose = true;

result = planAzElQLearning( ...
    azElData,startAzEl_deg,goalAzEl_deg,options);

plotAzElQLearningResult(azElData,result,2);

animationOptions = struct;
animationOptions.frameStride = 1;
animationOptions.pause_s = 0.01;
animationOptions.gifFile = ''; % Example: 'q_learning_az_el_plan.gif'
animationOptions.gifDelay_s = 0.04;
animateAzElQLearningPlan(azElData,result,animationOptions);
