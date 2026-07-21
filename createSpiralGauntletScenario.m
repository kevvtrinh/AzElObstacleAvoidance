function scenario = createSpiralGauntletScenario
%CREATESPIRALGAUNTLETSCENARIO Build a static spiral-wall planning challenge.
%   The start lies outside an Archimedean spiral ribbon and the goal is at
%   its center. A safe route must wind through the open channel between
%   successive turns instead of moving directly toward the goal.

    time_s = (0:1:520)';
    numTimes = numel(time_s);
    spiralCycles = 5.25;
    theta = linspace(0,2*pi*spiralCycles,1600)';
    radius_deg = linspace(26,3,numel(theta))';
    centerline = [radius_deg.*cos(theta),radius_deg.*sin(theta)];

    tangent = [gradient(centerline(:,1)),gradient(centerline(:,2))];
    tangentLength = hypot(tangent(:,1),tangent(:,2));
    unitNormal = [-tangent(:,2)./tangentLength, ...
        tangent(:,1)./tangentLength];
    wallHalfWidth_deg = 0.6;
    firstEdge = centerline+wallHalfWidth_deg*unitNormal;
    secondEdge = centerline-wallHalfWidth_deg*unitNormal;
    spiralWall = [firstEdge;flipud(secondEdge);firstEdge(1,:)];

    data = struct;
    data.targetName = 'Static spiral wall';
    data.time_s = time_s;
    data.az_deg = repmat({zeros(0,1)},numTimes,1);
    data.el_deg = repmat({zeros(0,1)},numTimes,1);
    data.status = repmat("NotVisible",numTimes,1);

    options = struct;
    options.azLim_deg = [-30,30];
    options.elLim_deg = [-30,30];
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 2;
    options.elRate_deg_s = 2;
    options.startTime_s = 0;
    options.planningHorizon_s = time_s(end);
    options.goalHold_s = 0;
    options.clearance_deg = 0.35;
    options.temporalPadding_s = 0;
    options.staticPolygons = {spiralWall};
    options.preventCornerCutting = true;
    options.episodes = 25;
    options.minimumEpisodesPerLearner = 25;
    options.earlyStopSuccessStreak = 10;
    options.guidedExplorationProbability = 0.45;
    options.guidanceWeight = 3;
    options.progressRewardWeight = 2;
    options.turnPenalty = 0.25;
    options.useParallel = false;
    options.randomSeed = 19;
    options.smoothPath = true;
    options.smoothingMaxLookahead = 250;
    options.verbose = false;

    scenario = struct;
    scenario.data = data;
    scenario.startAzEl_deg = [28,0];
    scenario.goalAzEl_deg = [0,0];
    scenario.centerAzEl_deg = [0,0];
    scenario.options = options;
    scenario.spiralWallAzEl_deg = spiralWall;
    scenario.spiralCycles = spiralCycles;
    scenario.minimumWinding_deg = 4*360;
end
