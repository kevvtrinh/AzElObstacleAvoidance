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
    verifyTrue(testCase,result.trajectory.success);
    verifyEqual(testCase,result.trajectory.azRate_deg_s(end),0, ...
        'AbsTol',1e-9);
    verifyEqual(testCase,result.trajectory.elRate_deg_s(end),0, ...
        'AbsTol',1e-9);
    verifyEqual(testCase,result.trajectory.azAcceleration_deg_s2(end),0, ...
        'AbsTol',1e-8);
    verifyEqual(testCase,result.trajectory.elAcceleration_deg_s2(end),0, ...
        'AbsTol',1e-8);
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


function testAnalyticAuditFindsStaticCollisionBetweenSamples(testCase)
    data = emptyData([0;1]);
    options = baseOptions([-2,2],[-2,2]);
    options.staticPolygons = {squareAt([0,0],0.2)};
    trajectory = straightAnalyticTrajectory([-1,0],[1,0],0,1);

    audit = auditAzElTrajectory( ...
        struct('data',data,'options',options),trajectory,false);

    verifyFalse(testCase,audit.collisionFree);
    verifyEqual(testCase,audit.offendingObstacleType,"staticCurve");
end


function testAnalyticAuditFindsCollisionBetweenMovingFrames(testCase)
    data = emptyData([0;1]);
    firstObstacle = squareAt([-2,0],0.25);
    secondObstacle = squareAt([2,0],0.25);
    data.az_deg = {firstObstacle(:,1);secondObstacle(:,1)};
    data.el_deg = {firstObstacle(:,2);secondObstacle(:,2)};
    options = baseOptions([-3,3],[-2,2]);
    trajectory = straightAnalyticTrajectory([0,0],[0,0],0,1);

    audit = auditAzElTrajectory( ...
        struct('data',data,'options',options),trajectory,false);

    verifyFalse(testCase,audit.collisionFree);
    verifyEqual(testCase,audit.offendingObstacleType,"dynamicCurve");
end


function testAnalyticAuditTracksMovingContourWhenCountChanges(testCase)
    data = emptyData([0;1]);
    firstObstacle = squareAt([-2,0],0.25);
    movedObstacle = squareAt([2,0],0.25);
    newObstacle = squareAt([10,5],0.25);
    data.az_deg = {firstObstacle(:,1); ...
        [movedObstacle(:,1);NaN;newObstacle(:,1)]};
    data.el_deg = {firstObstacle(:,2); ...
        [movedObstacle(:,2);NaN;newObstacle(:,2)]};
    options = baseOptions([-12,12],[-2,7]);
    trajectory = straightAnalyticTrajectory([0,0],[0,0],0,1);

    audit = auditAzElTrajectory( ...
        struct('data',data,'options',options),trajectory,false);

    verifyFalse(testCase,audit.collisionFree);
    verifyEqual(testCase,audit.offendingObstacleType,"dynamicCurve");
end


function testAnalyticAuditHonorsTemporalPadding(testCase)
    data = emptyData([0;1;2]);
    nearObstacle = squareAt([0,0],0.25);
    farObstacle = squareAt([5,0],0.25);
    data.az_deg = {nearObstacle(:,1);farObstacle(:,1);farObstacle(:,1)};
    data.el_deg = {nearObstacle(:,2);farObstacle(:,2);farObstacle(:,2)};
    options = baseOptions([-1,6],[-2,2]);
    trajectory = straightAnalyticTrajectory([0,0],[0,0],1,2);

    unpaddedAudit = auditAzElTrajectory( ...
        struct('data',data,'options',options),trajectory,false);
    options.temporalPadding_s = 1;
    paddedAudit = auditAzElTrajectory( ...
        struct('data',data,'options',options),trajectory,false);

    verifyTrue(testCase,unpaddedAudit.collisionFree);
    verifyFalse(testCase,paddedAudit.collisionFree);
    verifyEqual(testCase,paddedAudit.offendingObstacleType,"dynamicCurve");
end


function testObstacleFrameInterpolationUsesCommandTime(testCase)
    data = emptyData([0;2]);
    firstObstacle = squareAt([179,0],0.25);
    secondObstacle = firstObstacle;
    secondObstacle(:,1) = secondObstacle(:,1)-358;
    secondObstacle(:,2) = secondObstacle(:,2)+2;
    data.az_deg = {firstObstacle(:,1);secondObstacle(:,1)};
    data.el_deg = {firstObstacle(:,2);secondObstacle(:,2)};

    [az_deg,el_deg] = interpolateAzElObstacleFrame(data,1);

    verifyLessThan(testCase,max(abs(abs(az_deg)-180)),0.3);
    verifyEqual(testCase,el_deg,firstObstacle(:,2)+1,'AbsTol',1e-12);
end


function testPeriodicObstacleDisplayDoesNotBridgeAzimuthSeam(testCase)
    data = emptyData([0;1]);
    seamPolygon = [178,0;-178,0;-178,1;178,1;178,0];
    data.az_deg(:) = {seamPolygon(:,1)};
    data.el_deg(:) = {seamPolygon(:,2)};
    options = struct('azimuthTopology',"periodic", ...
        'azLim_deg',[-180,180]);

    [az_deg,el_deg] = interpolateAzElObstacleFrame(data,0,options);
    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators;true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;

    verifyEqual(testCase,numel(starts),2);
    for contourIndex = 1:numel(starts)
        contour = az_deg(starts(contourIndex):ends(contourIndex));
        verifyLessThanOrEqual(testCase,max(abs(diff(contour))),4+1e-12);
    end
end


function testRequestedSmoothCommandFailureIsNotReportedAsSuccess(testCase)
    data = emptyData([0;1]);
    options = baseOptions([0,1],[0,1]);
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 1;
    options.elRate_deg_s = 1;
    options.trajectoryRequireEndpointRest = true;
    options.maxAzAcceleration_deg_s2 = 1000;
    options.maxElAcceleration_deg_s2 = 1000;
    options.maxAzJerk_deg_s3 = 1000;
    options.maxElJerk_deg_s3 = 1000;

    result = planAzElQLearning(data,[0,0],[1,0],options);
    displayPath = selectAzElDisplayPath(result);

    verifyFalse(testCase,result.success);
    verifyTrue(testCase,result.routeSuccess);
    verifyTrue(testCase,result.diagnostic.routeReachedGoal);
    verifyFalse(testCase,result.trajectory.success);
    verifyFalse(testCase,result.trajectoryAudit.success);
    verifyEmpty(testCase,displayPath.time_s);
end


function testStaticTrajectoryRoundsCornersAndHonorsKinematics(testCase)
    suite = createPlannerGauntletSuite;
    scenario = configureTrajectoryScenario(suite(11));
    result = planAzElQLearning(scenario.data,scenario.startAzEl_deg, ...
        scenario.goalAzEl_deg,scenario.options);

    verifyTrue(testCase,result.success);
    verifyGreaterThan(testCase,maximumMovingHeadingChange(result.path),30, ...
        'The static regression no longer contains a meaningful coarse corner.');
    verifySmoothTrajectory(testCase,scenario,result);
end


function testDynamicTrajectoryRoundsCornersAndHonorsKinematics(testCase)
    suite = createPlannerGauntletSuite;
    scenario = configureTrajectoryScenario(suite(2));
    result = planAzElQLearning(scenario.data,scenario.startAzEl_deg, ...
        scenario.goalAzEl_deg,scenario.options);

    verifyTrue(testCase,result.success);
    verifyGreaterThan(testCase,maximumMovingHeadingChange(result.path),30, ...
        'The dynamic regression no longer contains a meaningful coarse corner.');
    verifySmoothTrajectory(testCase,scenario,result);
end


function scenario = configureTrajectoryScenario(scenario)
    scenario.options.generateSmoothTrajectory = true;
    scenario.options.trajectoryRequireEndpointRest = true;
    scenario.options.trajectorySampleTime_s = 0.1;
    scenario.options.maxAzAcceleration_deg_s2 = 20;
    scenario.options.maxElAcceleration_deg_s2 = 20;
    scenario.options.maxAzJerk_deg_s3 = 200;
    scenario.options.maxElJerk_deg_s3 = 200;
end


function verifySmoothTrajectory(testCase,scenario,result)
    verifyTrue(testCase,isfield(result,'trajectory'));
    trajectory = result.trajectory;
    verifyTrue(testCase,trajectory.success);

    expectedFields = {'time_s','az_deg','azWrapped_deg','el_deg', ...
        'timeIndex','planningTimeIndex','isWaiting','azRate_deg_s', ...
        'elRate_deg_s','azAcceleration_deg_s2', ...
        'elAcceleration_deg_s2','azJerk_deg_s3','elJerk_deg_s3', ...
        'segments'};
    for fieldIndex = 1:numel(expectedFields)
        verifyTrue(testCase,isfield(trajectory,expectedFields{fieldIndex}), ...
            sprintf('Missing trajectory field %s.',expectedFields{fieldIndex}));
    end

    numSamples = numel(trajectory.time_s);
    sampleFields = expectedFields(2:end-1);
    for fieldIndex = 1:numel(sampleFields)
        fieldName = sampleFields{fieldIndex};
        verifyEqual(testCase,numel(trajectory.(fieldName)),numSamples, ...
            sprintf('Trajectory field %s has the wrong length.',fieldName));
    end
    verifyGreaterThan(testCase,numSamples,numel(result.path.time_s));
    verifyTrue(testCase,all(diff(trajectory.time_s) > 0));
    verifyTrue(testCase,all(isfinite([trajectory.time_s(:); ...
        trajectory.az_deg(:);trajectory.el_deg(:); ...
        trajectory.azRate_deg_s(:);trajectory.elRate_deg_s(:); ...
        trajectory.azAcceleration_deg_s2(:); ...
        trajectory.elAcceleration_deg_s2(:); ...
        trajectory.azJerk_deg_s3(:);trajectory.elJerk_deg_s3(:)])));

    verifyEqual(testCase,trajectory.time_s(1),result.path.time_s(1), ...
        'AbsTol',1e-10);
    verifyEqual(testCase,trajectory.az_deg([1,end]), ...
        result.path.az_deg([1,end]),'AbsTol',1e-8);
    verifyEqual(testCase,trajectory.el_deg([1,end]), ...
        result.path.el_deg([1,end]),'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,trajectory.time_s(end), ...
        scenario.data.time_s(end)+1e-10);
    if scenario.options.trajectoryRequireEndpointRest
        endpointIndices = [1,numel(trajectory.time_s)];
        verifyEqual(testCase,trajectory.azRate_deg_s(endpointIndices), ...
            [0;0],'AbsTol',1e-8);
        verifyEqual(testCase,trajectory.elRate_deg_s(endpointIndices), ...
            [0;0],'AbsTol',1e-8);
        verifyEqual(testCase, ...
            trajectory.azAcceleration_deg_s2(endpointIndices), ...
            [0;0],'AbsTol',1e-7);
        verifyEqual(testCase, ...
            trajectory.elAcceleration_deg_s2(endpointIndices), ...
            [0;0],'AbsTol',1e-7);
    end

    tolerance = 1e-7;
    verifyLessThanOrEqual(testCase,max(abs(trajectory.azRate_deg_s)), ...
        scenario.options.azRate_deg_s+tolerance);
    verifyLessThanOrEqual(testCase,max(abs(trajectory.elRate_deg_s)), ...
        scenario.options.elRate_deg_s+tolerance);
    verifyLessThanOrEqual(testCase, ...
        max(abs(trajectory.azAcceleration_deg_s2)), ...
        scenario.options.maxAzAcceleration_deg_s2+tolerance);
    verifyLessThanOrEqual(testCase, ...
        max(abs(trajectory.elAcceleration_deg_s2)), ...
        scenario.options.maxElAcceleration_deg_s2+tolerance);
    verifyLessThanOrEqual(testCase,max(abs(trajectory.azJerk_deg_s3)), ...
        scenario.options.maxAzJerk_deg_s3+tolerance);
    verifyLessThanOrEqual(testCase,max(abs(trajectory.elJerk_deg_s3)), ...
        scenario.options.maxElJerk_deg_s3+tolerance);

    verifyLessThanOrEqual(testCase,maximumMovingHeadingChange(trajectory), ...
        20+1e-9,'The generated trajectory still contains a sharp corner.');

    trajectoryResult = result;
    trajectoryResult.path = trajectory;
    audit = auditPlannerPath(scenario,trajectoryResult);
    verifyTrue(testCase,audit.collisionFree,char(audit.message));
    verifyTrue(testCase,audit.kinematicallyFeasible,char(audit.message));
    verifyTrue(testCase,audit.c2Continuous,char(audit.message));
    verifyTrue(testCase,audit.mechanicalBoundsSafe,char(audit.message));
    verifyLessThanOrEqual(testCase,audit.maxAzRate_deg_s, ...
        scenario.options.azRate_deg_s+tolerance);
    verifyLessThanOrEqual(testCase,audit.maxElRate_deg_s, ...
        scenario.options.elRate_deg_s+tolerance);
    verifyLessThanOrEqual(testCase,audit.maxAzAcceleration_deg_s2, ...
        scenario.options.maxAzAcceleration_deg_s2+tolerance);
    verifyLessThanOrEqual(testCase,audit.maxElAcceleration_deg_s2, ...
        scenario.options.maxElAcceleration_deg_s2+tolerance);
    verifyLessThanOrEqual(testCase,audit.maxAzJerk_deg_s3, ...
        scenario.options.maxAzJerk_deg_s3+tolerance);
    verifyLessThanOrEqual(testCase,audit.maxElJerk_deg_s3, ...
        scenario.options.maxElJerk_deg_s3+tolerance);
end


function maximumChange_deg = maximumMovingHeadingChange(path)
    displacement = [diff(path.az_deg(:)),diff(path.el_deg(:))];
    moving = hypot(displacement(:,1),displacement(:,2)) > 1e-8;
    if nnz(moving) < 2
        maximumChange_deg = 0;
        return
    end
    heading_rad = atan2(displacement(:,2),displacement(:,1));
    adjacentMoving = moving(1:end-1) & moving(2:end);
    if ~any(adjacentMoving)
        maximumChange_deg = 0;
        return
    end
    firstHeading = heading_rad(1:end-1);
    secondHeading = heading_rad(2:end);
    headingChange_rad = atan2( ...
        sin(secondHeading(adjacentMoving)-firstHeading(adjacentMoving)), ...
        cos(secondHeading(adjacentMoving)-firstHeading(adjacentMoving)));
    maximumChange_deg = max(abs(rad2deg(headingChange_rad)));
end


function data = emptyData(time_s)
    data = struct;
    data.targetName = 'Empty mask';
    data.time_s = time_s;
    data.az_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.el_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.status = repmat("NotVisible",numel(time_s),1);
end


function polygon = squareAt(center_deg,halfWidth_deg)
    offsets = halfWidth_deg*[-1,-1;1,-1;1,1;-1,1;-1,-1];
    polygon = center_deg+offsets;
end


function trajectory = straightAnalyticTrajectory( ...
        startPoint_deg,endPoint_deg,startTime_s,endTime_s)
    fraction = (0:5)'/5;
    controlPoints_deg = startPoint_deg+fraction.* ...
        (endPoint_deg-startPoint_deg);
    segment = struct('startTime_s',startTime_s, ...
        'endTime_s',endTime_s, ...
        'controlPoints_deg',controlPoints_deg,'kind',"line");
    trajectory = struct('success',true,'segments',segment);
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
    options.trajectoryRequireEndpointRest = false;
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
