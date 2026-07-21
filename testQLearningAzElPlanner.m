function tests = testQLearningAzElPlanner
%TESTQLEARNINGAZELPLANNER Regression tests for the supplied package.

    tests = functiontests(localfunctions);
end


function testEmptyMapPolicyAndDiagnostics(testCase)
    data = emptyData((0:0.5:20)');
    options = baseOptions([-20,20],[10,30]);
    result = planAzElQLearning(data,[-10,20],[10,20],options);

    verifyTrue(testCase,result.success);
    verifyTrue(testCase,result.diagnostic.collisionFree);
    verifyEqual(testCase,result.diagnostic.requestedTrainingEpisodes, ...
        options.episodes);
    verifyEqual(testCase,result.diagnostic.selectedPolicySource,"qLearning");
    verifyGreaterThan(testCase,result.diagnostic.trainingSuccessRate,0);
    verifyLessThanOrEqual(testCase,max(abs(diff(result.path.az_deg))), ...
        options.azRate_deg_s*result.grid.dt_s+1e-10);
    verifyLessThanOrEqual(testCase,max(abs(diff(result.path.el_deg))), ...
        options.elRate_deg_s*result.grid.dt_s+1e-10);
end


function testTemporaryBarrierRequiresWaitAndSmoothingIsSafe(testCase)
    time_s = (0:0.5:75)';
    data = createQLearningExampleAzElData(time_s);
    options = baseOptions([-100,100],[5,85]);
    options.episodes = 200;
    options.clearance_deg = 1.5;
    options.temporalPadding_s = 0.5;
    options.goalHold_s = 2;
    result = planAzElQLearning(data,[-80,20],[80,70],options);

    verifyTrue(testCase,result.success);
    verifyTrue(testCase,result.diagnostic.collisionFree);
    verifyGreaterThan(testCase,result.diagnostic.waitTime_s,0);
    verifyLessThanOrEqual(testCase, ...
        result.diagnostic.turnCountAfterSmoothing, ...
        result.diagnostic.turnCountBeforeSmoothing);
end


function testSeamCrossingPolygonDoesNotBecomeBoundingBox(testCase)
    seamData = emptyData((0:1:2)');
    seamAz = [178;-178;-178;178;178];
    seamEl = [20;20;30;30;20];
    seamData.az_deg(:) = {seamAz};
    seamData.el_deg(:) = {seamEl};
    options = baseOptions([-180,180],[0,40]);
    [blocked,grid] = buildAzElOccupancy(seamData,options);

    [~,elIndex] = min(abs(grid.el_deg-24));
    [~,leftIndex] = min(abs(grid.az_deg+180));
    [~,centerIndex] = min(abs(grid.az_deg));
    [~,rightIndex] = min(abs(grid.az_deg-180));
    verifyTrue(testCase,blocked(elIndex,leftIndex,1));
    verifyTrue(testCase,blocked(elIndex,rightIndex,1));
    verifyFalse(testCase,blocked(elIndex,centerIndex,1));
end


function testStaticPolygonAndClearance(testCase)
    data = emptyData((0:1:2)');
    polygon = [-1,-1;1,-1;1,1;-1,1;-1,-1];
    options = baseOptions([-4,4],[-4,4]);
    options.staticPolygons = {polygon};
    options.clearance_deg = 2;
    [blocked,grid] = buildAzElOccupancy(data,options);

    [~,zeroAz] = min(abs(grid.az_deg));
    [~,zeroEl] = min(abs(grid.el_deg));
    [~,twoAz] = min(abs(grid.az_deg-2));
    verifyTrue(testCase,blocked(zeroEl,zeroAz,1));
    verifyTrue(testCase,blocked(zeroEl,twoAz,1));
end


function testCornerCuttingOption(testCase)
    data = emptyData([0;1]);
    options = baseOptions([0,2],[0,2]);
    options.gridStep_deg = [2,2];
    options.azRate_deg_s = 2;
    options.elRate_deg_s = 2;
    options.episodes = 5;
    options.minimumEpisodesPerLearner = 1;
    options.earlyStopSuccessStreak = 1;
    options.staticPolygons = { ...
        [1.9,-0.1;2.1,-0.1;2.1,0.1;1.9,0.1;1.9,-0.1], ...
        [-0.1,1.9;0.1,1.9;0.1,2.1;-0.1,2.1;-0.1,1.9]};

    options.preventCornerCutting = false;
    diagonalAllowed = planAzElQLearning(data,[0,0],[2,2],options);
    verifyTrue(testCase,diagonalAllowed.success);

    options.preventCornerCutting = true;
    diagonalBlocked = planAzElQLearning(data,[0,0],[2,2],options);
    verifyFalse(testCase,diagonalBlocked.success);
end


function testWrappedGoalUsesEquivalentCableBranch(testCase)
    data = emptyData((0:1:3)');
    options = baseOptions([0,400],[0,20]);
    options.gridStep_deg = [10,10];
    options.azRate_deg_s = 10;
    options.elRate_deg_s = 10;
    options.goalAzimuthIsWrapped = true;
    result = planAzElQLearning(data,[370,10],[10,10],options);

    verifyTrue(testCase,result.success);
    verifyEqual(testCase,result.path.az_deg(end),370,'AbsTol',1e-12);
end


function testMovingGoalHistory(testCase)
    time_s = (0:1:6)';
    data = emptyData(time_s);
    goalHistory = [6,0;6,0;4,0;4,0;2,0;2,0;0,0];
    options = baseOptions([0,6],[0,2]);
    options.gridStep_deg = [2,2];
    options.azRate_deg_s = 2;
    options.elRate_deg_s = 2;
    result = planAzElQLearning(data,[0,0],goalHistory,options);

    verifyTrue(testCase,result.success);
    planningIndex = result.path.planningTimeIndex(end);
    [~,azIndex] = min(abs(result.grid.az_deg-result.path.az_deg(end)));
    [~,elIndex] = min(abs(result.grid.el_deg-result.path.el_deg(end)));
    finalCell = sub2ind([numel(result.grid.el_deg), ...
        numel(result.grid.az_deg)],elIndex,azIndex);
    verifyTrue(testCase,any(result.goalCells{planningIndex} == finalCell));
end


function testBlockedStartAndOutOfRangeGoalFailCleanly(testCase)
    time_s = (0:1:3)';
    data = emptyData(time_s);
    startPolygon = [-1,-1;1,-1;1,1;-1,1;-1,-1];
    data.az_deg(:) = {startPolygon(:,1)};
    data.el_deg(:) = {startPolygon(:,2)};
    options = baseOptions([-10,10],[-10,10]);
    blockedStart = planAzElQLearning(data,[0,0],[8,8],options);
    verifyFalse(testCase,blockedStart.success);
    verifyEmpty(testCase,blockedStart.path.time_s);

    clearData = emptyData(time_s);
    invalidGoal = planAzElQLearning(clearData,[0,0],[20,0],options);
    verifyFalse(testCase,invalidGoal.success);
    verifyEmpty(testCase,invalidGoal.path.time_s);
end


function testInvalidOptionsAndMemoryLimitFailEarly(testCase)
    data = emptyData((0:1:3)');
    options = baseOptions([-10,10],[-10,10]);
    options.gridStep_deg = [1,1,1];
    verifyError(testCase,@() planAzElQLearning( ...
        data,[0,0],[2,2],options), ...
        'planAzElQLearning:InvalidGridStep');

    options = baseOptions([-10,10],[-10,10]);
    options.maxQTableMB = 1e-6;
    thrown = [];
    try
        planAzElQLearning(data,[0,0],[2,2],options);
    catch exception
        thrown = exception;
    end
    verifyNotEmpty(testCase,thrown);
    verifySubstring(testCase,thrown.message,'Q-table would require');
end


function testSerialRunsAreReproducible(testCase)
    data = emptyData((0:1:10)');
    options = baseOptions([-10,10],[-10,10]);
    options.episodes = 20;
    first = planAzElQLearning(data,[-4,-4],[4,4],options);
    second = planAzElQLearning(data,[-4,-4],[4,4],options);

    verifyEqual(testCase,first.success,second.success);
    verifyEqual(testCase,first.path.az_deg,second.path.az_deg);
    verifyEqual(testCase,first.path.el_deg,second.path.el_deg);
    verifyEqual(testCase,first.diagnostic.trainingEpisodes, ...
        second.diagnostic.trainingEpisodes);
end


function testParallelFallbackIsReportedWhenToolboxIsUnavailable(testCase)
    if license('test','Distrib_Computing_Toolbox')
        % Avoid opening a pool in this serial regression suite. Parallel
        % execution itself is exercised only on configured CI workers.
        verifyTrue(testCase,true);
        return
    end

    data = emptyData((0:1:4)');
    options = baseOptions([-4,4],[-4,4]);
    options.useParallel = true;
    warningState = warning('off','planAzElQLearning:ParallelFallback');
    cleanup = onCleanup(@() warning(warningState));
    result = planAzElQLearning(data,[0,0],[2,2],options);

    verifyTrue(testCase,result.diagnostic.requestedParallel);
    verifyFalse(testCase,result.diagnostic.usedParallel);
    verifyNotEmpty(testCase,result.diagnostic.parallelFallbackReason);
    verifySubstring(testCase,result.diagnostic.parallelFallbackReason, ...
        "license is unavailable");
    clear cleanup
end


function testShortForwardSeamCrossing(testCase)
    data = emptyData((0:1:5)');
    options = seamOptions;
    result = planAzElQLearning(data,[179,30],[-179,30],options);

    verifyTrue(testCase,result.success);
    verifyLessThanOrEqual(testCase,abs(result.path.az_deg(end)-179),2+1e-9);
    verifyEqual(testCase,result.diagnostic.duration_s,2,'AbsTol',1e-9);
    verifyLessThanOrEqual(testCase,max(abs(diff(result.path.az_deg))),1+1e-9);
end


function testShortReverseSeamCrossing(testCase)
    data = emptyData((0:1:5)');
    options = seamOptions;
    result = planAzElQLearning(data,[-179,30],[179,30],options);

    verifyTrue(testCase,result.success);
    verifyLessThanOrEqual(testCase,abs(result.path.az_deg(end)+179),2+1e-9);
    verifyEqual(testCase,result.diagnostic.duration_s,2,'AbsTol',1e-9);
    verifyLessThanOrEqual(testCase,max(abs(diff(result.path.az_deg))),1+1e-9);
end


function testSeamInterpolationUsesShortestArc(testCase)
    samples_deg = interpolateAzimuthDeg(179,-179,[0,0.5,1]);
    verifyEqual(testCase,samples_deg,[179,180,181],'AbsTol',1e-12);
    verifyGreaterThan(testCase,abs(samples_deg(2)),179);
end


function testSeamSlewRateHasNoSpike(testCase)
    wrapped_deg = [178,179,-180,-179];
    continuous_deg = unwrapAzimuthDeg(wrapped_deg);
    velocity_deg_s = diff(continuous_deg);
    verifyEqual(testCase,velocity_deg_s,[1,1,1],'AbsTol',1e-12);
end


function testSeamAccelerationAndJerkRemainZero(testCase)
    continuous_deg = unwrapAzimuthDeg([178,179,-180,-179]);
    velocity_deg_s = diff(continuous_deg);
    acceleration_deg_s2 = diff(velocity_deg_s);
    jerk_deg_s3 = diff(acceleration_deg_s2);
    verifyEqual(testCase,acceleration_deg_s2,[0,0],'AbsTol',1e-12);
    verifyEqual(testCase,jerk_deg_s3,0,'AbsTol',1e-12);
end


function testEquivalentPositiveAndNegative180Endpoints(testCase)
    verifyEqual(testCase,shortestAzimuthDeltaDeg(180,-180),0, ...
        'AbsTol',1e-12);
    verifyEqual(testCase,shortestAzimuthDeltaDeg(-180,180),0, ...
        'AbsTol',1e-12);
end


function testRepeatedSeamCrossingsRemainContinuous(testCase)
    wrapped_deg = [178,179,-180,-179,-178,-179,180,179,178];
    continuous_deg = unwrapAzimuthDeg(wrapped_deg);
    verifyLessThanOrEqual(testCase,max(abs(diff(continuous_deg))),1+1e-12);
    verifyEqual(testCase,continuous_deg, ...
        [178,179,180,181,182,181,180,179,178],'AbsTol',1e-12);
end


function testMovingObstacleSweepCannotCrossPathBetweenFrames(testCase)
    data = emptyData([0;1]);
    firstObstacle = [-2.4,0.6;-1.6,0.6;-1.6,1.4;-2.4,1.4;-2.4,0.6];
    secondObstacle = [1.6,0.6;2.4,0.6;2.4,1.4;1.6,1.4;1.6,0.6];
    data.az_deg = {firstObstacle(:,1);secondObstacle(:,1)};
    data.el_deg = {firstObstacle(:,2);secondObstacle(:,2)};
    options = baseOptions([-3,3],[-1,3]);
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 2;
    options.elRate_deg_s = 2;
    options.episodes = 2;
    options.minimumEpisodesPerLearner = 1;
    options.earlyStopSuccessStreak = 1;

    result = planAzElQLearning(data,[0,0],[0,2],options);
    verifyFalse(testCase,result.success);
end


function testGoalHoldChecksEveryDynamicSweep(testCase)
    data = emptyData([0;1;2]);
    leftObstacle = [-2.4,-0.4;-1.6,-0.4;-1.6,0.4; ...
        -2.4,0.4;-2.4,-0.4];
    rightObstacle = leftObstacle+[4,0];
    data.az_deg = {leftObstacle(:,1);leftObstacle(:,1);rightObstacle(:,1)};
    data.el_deg = {leftObstacle(:,2);leftObstacle(:,2);rightObstacle(:,2)};
    options = baseOptions([-3,3],[-2,2]);
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 2;
    options.elRate_deg_s = 2;
    options.goalHold_s = 2;
    options.episodes = 2;
    options.minimumEpisodesPerLearner = 1;
    options.earlyStopSuccessStreak = 1;

    result = planAzElQLearning(data,[0,0],[0,0],options);
    verifyFalse(testCase,result.success);

    scenario = struct('data',data,'options',options);
    unsafeResult = result;
    unsafeResult.success = true;
    unsafeResult.path = struct( ...
        'planningTimeIndex',1,'timeIndex',1,'time_s',0, ...
        'az_deg',0,'azWrapped_deg',0,'el_deg',0,'isWaiting',false);
    audit = auditPlannerPath(scenario,unsafeResult);
    verifyFalse(testCase,audit.collisionFree);
    verifyFalse(testCase,audit.holdSafe);
    verifyEqual(testCase,audit.offendingObstacleType,"goalHold");
    verifyEqual(testCase,audit.checkedHoldSegments,2);
end


function testThinStaticObstacleHonorsContinuousClearance(testCase)
    data = emptyData([0;1]);
    thinObstacle = [0.4,-0.1;0.6,-0.1;0.6,0.1;0.4,0.1;0.4,-0.1];
    options = baseOptions([-2,2],[-2,2]);
    options.gridStep_deg = [2,2];
    options.staticPolygons = {thinObstacle};
    options.clearance_deg = 0.5;

    result = planAzElQLearning(data,[0,0],[0,0],options);
    verifyFalse(testCase,result.success);
    verifyEmpty(testCase,result.path.time_s);
end


function data = emptyData(time_s)
    data = struct;
    data.targetName = 'Empty mask';
    data.time_s = time_s;
    data.az_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.el_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.status = repmat("NotVisible",numel(time_s),1);
end


function options = baseOptions(azLim_deg,elLim_deg)
    options = struct;
    options.azLim_deg = azLim_deg;
    options.elLim_deg = elLim_deg;
    options.gridStep_deg = [2,2];
    options.azRate_deg_s = 12;
    options.elRate_deg_s = 12;
    options.startTime_s = 0;
    options.goalHold_s = 0;
    options.clearance_deg = 0;
    options.temporalPadding_s = 0;
    options.staticPolygons = {};
    options.preventCornerCutting = true;
    options.goalAzimuthIsWrapped = false;
    options.cableReserve_deg = 0;
    options.episodes = 120;
    options.minimumEpisodesPerLearner = 40;
    options.earlyStopSuccessStreak = 15;
    options.useParallel = false;
    options.randomSeed = 11;
    options.smoothPath = true;
    options.verbose = false;
end


function options = seamOptions
    options = baseOptions([-180,180],[0,60]);
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 1;
    options.elRate_deg_s = 1;
    options.azimuthTopology = "periodic";
    options.goalAzimuthIsWrapped = true;
    options.episodes = 5;
    options.minimumEpisodesPerLearner = 1;
    options.earlyStopSuccessStreak = 1;
end
