function result = planAzElQLearning(azElData,startAzEl_deg,goalAzEl_deg,options)
%PLANAZELQLEARNING Fast time-dependent Q-learning for az/el avoidance.
%   RESULT = PLANAZELQLEARNING(DATA,START,GOAL,OPTIONS) learns a policy on
%   states (azimuth cell, elevation cell, time index). The nine actions are
%   wait plus the eight neighboring grid moves. Unsafe actions are masked
%   before selection, so exploration never deliberately enters a mask.
%   The selected timed polyline is postprocessed into RESULT.trajectory, a
%   collision-certified C2 quintic command with explicit per-axis rate,
%   acceleration, and jerk limits.
%   RESULT.routeSuccess reports whether the discrete route reached a goal.
%   When smooth commands are requested, RESULT.success is stricter: it is
%   true only when the returned trajectory also passes the execution audit.
%
%   Independent learners can train in PARFOR. Each Q-table stays on its
%   worker; only the compact candidate path returns to the client.

    % ---------------------------------------------------------------------
    % Input and option normalization
    % ---------------------------------------------------------------------
    if nargin < 4 || isempty(options)
        options = struct;
    end
    options = applyDefaults(options,struct( ...
        'azLim_deg',[-180,180], ...
        'elLim_deg',[0,90], ...
        'gridStep_deg',[2,2], ...
        'azRate_deg_s',10, ...
        'elRate_deg_s',10, ...
        'startTime_s',[], ...
        'endTime_s',[], ...
        'planningHorizon_s',[], ...
        'goalHold_s',0, ...
        'clearance_deg',0, ...
        'temporalPadding_s',0, ...
        'staticPolygons',{{}}, ...
        'preventCornerCutting',true, ...
        'goalAzimuthIsWrapped',false, ...
        'azimuthTopology',"mechanical", ...
        'cableReserve_deg',0, ...
        'episodes',4000, ...
        'learningRate',0.25, ...
        'discountFactor',0.995, ...
        'epsilonStart',0.45, ...
        'epsilonEnd',0.02, ...
        'guidedExplorationProbability',0.35, ...
        'guidanceWeight',3.0, ...
        'progressRewardWeight',2.0, ...
        'stepPenalty',1.0, ...
        'waitPenalty',0.20, ...
        'turnPenalty',0.75, ...
        'goalReward',500, ...
        'failurePenalty',200, ...
        'minimumEpisodesPerLearner',300, ...
        'earlyStopSuccessStreak',100, ...
        'useParallel',true, ...
        'numLearners',0, ...
        'randomSeed',7, ...
        'maxQTableMB',512, ...
        'smoothPath',true, ...
        'smoothingMaxLookahead',250, ...
        'generateSmoothTrajectory',true, ...
        'trajectoryRequireEndpointRest',false, ...
        'trajectorySampleTime_s',0.1, ...
        'trajectoryMaxBlendFraction',0.48, ...
        'trajectoryMinBlendFraction',0.02, ...
        'trajectoryBlendAttempts',13, ...
        'verbose',true));
    options = applyTrajectoryDefaults(options);

    options.azimuthTopology = string(validatestring( ...
        options.azimuthTopology,{'mechanical','periodic'},mfilename, ...
        'options.azimuthTopology'));
    validatePlannerOptions(options);
    validateattributes(startAzEl_deg,{'numeric'}, ...
        {'row','numel',2,'real','finite'},mfilename,'startAzEl_deg');
    validateattributes(goalAzEl_deg,{'numeric'}, ...
        {'2d','ncols',2,'real','finite'},mfilename,'goalAzEl_deg');
    validateattributes(options.episodes,{'numeric'}, ...
        {'scalar','integer','positive','finite'});
    validateattributes(options.azRate_deg_s,{'numeric'}, ...
        {'scalar','positive','finite'});
    validateattributes(options.elRate_deg_s,{'numeric'}, ...
        {'scalar','positive','finite'});
    validateattributes(options.cableReserve_deg,{'numeric'}, ...
        {'scalar','nonnegative','finite'});

    % ---------------------------------------------------------------------
    % Planning window and goal normalization
    % ---------------------------------------------------------------------
    originalTime_s = azElData.time_s(:);
    if isempty(options.startTime_s)
        options.startTime_s = originalTime_s(1);
    end
    if options.startTime_s < originalTime_s(1) || ...
            options.startTime_s > originalTime_s(end)
        error('startTime_s must lie inside azElData.time_s.');
    end
    [~,firstOriginalIndex] = min(abs(originalTime_s-options.startTime_s));

    if ~isempty(options.endTime_s)
        requestedEndTime_s = options.endTime_s;
    elseif ~isempty(options.planningHorizon_s)
        requestedEndTime_s = options.startTime_s+options.planningHorizon_s;
    else
        requestedEndTime_s = originalTime_s(end);
    end
    requestedEndTime_s = min(requestedEndTime_s,originalTime_s(end));
    [~,lastOriginalIndex] = min(abs(originalTime_s-requestedEndTime_s));
    if lastOriginalIndex <= firstOriginalIndex
        error('The planning time window requires at least two samples.');
    end

    planningIndices = (firstOriginalIndex:lastOriginalIndex)';
    planningData = sliceAzElData(azElData,planningIndices);
    if size(goalAzEl_deg,1) == numel(originalTime_s)
        planningGoalAzEl_deg = goalAzEl_deg(planningIndices,:);
    elseif size(goalAzEl_deg,1) == 1
        planningGoalAzEl_deg = goalAzEl_deg;
    else
        error('GOAL must be 1-by-2 or have one row per azElData.time_s sample.');
    end

    effectiveAzLim_deg = options.azLim_deg+ ...
        [options.cableReserve_deg,-options.cableReserve_deg];
    if effectiveAzLim_deg(1) >= effectiveAzLim_deg(2)
        error('cableReserve_deg leaves no usable azimuth range.');
    end
    options.azLim_deg = effectiveAzLim_deg;

    % ---------------------------------------------------------------------
    % Discretized occupancy and transition-safety model
    % ---------------------------------------------------------------------
    plannerTic = tic;
    [blocked,grid,maskStats] = ...
        azel.mapping.buildAzElOccupancy(planningData,options);
    grid.originalTimeIndex = planningIndices;
    numEl = numel(grid.el_deg);
    numAz = numel(grid.az_deg);
    numCells = numEl*numAz;
    numTimes = numel(grid.time_s);
    blockedByCell = reshape(blocked,numCells,numTimes);

    if startAzEl_deg(1) < options.azLim_deg(1) || ...
            startAzEl_deg(1) > options.azLim_deg(2) || ...
            startAzEl_deg(2) < grid.el_deg(1) || ...
            startAzEl_deg(2) > grid.el_deg(end)
        error('The mechanical start az/el lies outside the planning limits.');
    end
    [startElIndex,startAzIndex] = snapToGrid( ...
        startAzEl_deg,grid.az_deg,grid.el_deg,isPeriodicGrid(grid));
    startCell = sub2ind([numEl,numAz],startElIndex,startAzIndex);
    if blockedByCell(startCell,1)
        result = failedStartResult(options,grid,maskStats,startAzEl_deg, ...
            goalAzEl_deg,'The snapped start cell is blocked at the start time.');
        printDiagnostic(result);
        return
    end

    goalCells = makeGoalCells(planningGoalAzEl_deg,grid, ...
        options.goalAzimuthIsWrapped);
    if all(cellfun(@isempty,goalCells))
        result = failedStartResult(options,grid,maskStats,startAzEl_deg, ...
            goalAzEl_deg,'No goal branch lies inside the planning limits.');
        printDiagnostic(result);
        return
    end
    goalDistance_deg = makeGoalDistance(goalCells,grid);
    holdSteps = max(0,ceil(options.goalHold_s/grid.dt_s-1e-12));
    actionModel = makeActionModel(grid,options);
    [actionModel.dynamicEdgeSafe,actionModel.dynamicPolygons] = ...
        makeDynamicEdgeSafety(planningData,grid,actionModel,options);
    if ~any(any(actionModel.nextCell(:,2:end) > 0))
        error(['No slew action can reach an adjacent grid cell in one time ' ...
            'sample. Reduce gridStep_deg, increase the rate limits, or use ' ...
            'a larger azElData time step.']);
    end

    numActions = size(actionModel.offsets,1);
    qTableMB = double(numCells)*double(numTimes)*double(numActions)*4/2^20;
    if qTableMB > options.maxQTableMB
        error(['The Q-table would require %.1f MB per learner. Increase ' ...
            'gridStep_deg, shorten planningHorizon_s, tighten az/el limits, ' ...
            'or raise maxQTableMB if the worker memory is sufficient.'],qTableMB);
    end

    % ---------------------------------------------------------------------
    % Independent learners
    % ---------------------------------------------------------------------
    [useParallel,numLearners,parallelFallbackReason] = ...
        configureParallel(options);
    episodesPerLearner = ceil(options.episodes/numLearners);
    candidates = cell(numLearners,1);
    if useParallel
        parfor learnerIndex = 1:numLearners
            candidates{learnerIndex} = trainOneLearner( ...
                blockedByCell,grid,startCell,goalCells,goalDistance_deg, ...
                holdSteps,actionModel,options,episodesPerLearner,learnerIndex);
        end
    else
        candidates{1} = trainOneLearner( ...
            blockedByCell,grid,startCell,goalCells,goalDistance_deg, ...
            holdSteps,actionModel,options,episodesPerLearner,1);
    end

    % ---------------------------------------------------------------------
    % Candidate routes and executable-trajectory certification
    % ---------------------------------------------------------------------
    % Keep deterministic time-expanded graph search as the route fallback.
    % When a smooth executable command is requested, it is also tried after
    % all successful learned routes fail trajectory generation or audit.
    if ~any(cellfun(@(candidate) candidate.success,candidates))
        fallback = makeGraphFallbackCandidate(blockedByCell,grid,startCell, ...
            goalCells,goalDistance_deg,holdSteps,actionModel,options);
        candidates{end+1,1} = fallback;
    end

    candidateTrajectories = cell(size(candidates));
    candidateTrajectoryAudits = cell(size(candidates));
    executableCandidateIndex = NaN;
    trajectoryAttempts = 0;
    if options.generateSmoothTrajectory
        successfulOrder = rankSuccessfulCandidates(candidates);
        for orderIndex = 1:numel(successfulOrder)
            candidateIndex = successfulOrder(orderIndex);
            trajectoryAttempts = trajectoryAttempts+1;
            [candidateTrajectories{candidateIndex}, ...
                candidateTrajectoryAudits{candidateIndex},passed] = ...
                makeAuditedTrajectory(candidates{candidateIndex}, ...
                planningData,grid,options);
            if passed
                executableCandidateIndex = candidateIndex;
                break
            end
        end

        % A learned route can reach the goal yet be impossible to round at
        % its fixed obstacle timing. In that case the deterministic search
        % may supply a different route that admits a certified trajectory.
        if isnan(executableCandidateIndex) && ...
                ~hasGraphFallbackCandidate(candidates)
            fallback = makeGraphFallbackCandidate( ...
                blockedByCell,grid,startCell,goalCells,goalDistance_deg, ...
                holdSteps,actionModel,options);
            candidates{end+1,1} = fallback;
            candidateTrajectories{end+1,1} = [];
            candidateTrajectoryAudits{end+1,1} = [];
            if fallback.success
                candidateIndex = numel(candidates);
                trajectoryAttempts = trajectoryAttempts+1;
                [candidateTrajectories{candidateIndex}, ...
                    candidateTrajectoryAudits{candidateIndex},passed] = ...
                    makeAuditedTrajectory(fallback,planningData,grid,options);
                if passed
                    executableCandidateIndex = candidateIndex;
                end
            end
        end
    end

    % ---------------------------------------------------------------------
    % Public result and diagnostics
    % ---------------------------------------------------------------------
    if ~isnan(executableCandidateIndex)
        bestCandidateIndex = executableCandidateIndex;
    else
        bestCandidateIndex = selectBestCandidate(candidates);
    end
    best = candidates{bestCandidateIndex};
    result = struct;
    routeReachedGoal = logical(best.success);
    result.trajectory = emptyTrajectoryResult( ...
        "smooth trajectory generation was not requested");
    trajectoryAudit = unsuccessfulTrajectoryAudit( ...
        "smooth trajectory generation was not requested");
    if options.generateSmoothTrajectory
        if bestCandidateIndex <= numel(candidateTrajectories) && ...
                ~isempty(candidateTrajectories{bestCandidateIndex})
            result.trajectory = candidateTrajectories{bestCandidateIndex};
            trajectoryAudit = ...
                candidateTrajectoryAudits{bestCandidateIndex};
        elseif routeReachedGoal
            result.trajectory = emptyTrajectoryResult( ...
                "no audited smooth trajectory was found for the route");
            trajectoryAudit = unsuccessfulTrajectoryAudit( ...
                "no audited smooth trajectory was found for the route");
        else
            result.trajectory = emptyTrajectoryResult( ...
                "no route reached the goal");
            trajectoryAudit = unsuccessfulTrajectoryAudit( ...
                "no route reached the goal");
        end
        result.success = routeReachedGoal && trajectoryAudit.success;
    else
        result.success = routeReachedGoal;
    end

    if result.success
        result.status = "SUCCESS";
        if best.learnerIndex == 0
            if options.generateSmoothTrajectory
                result.message = ['The graph-search fallback reached the goal ' ...
                    'with an audited smooth executable trajectory.'];
            else
                result.message = ['The safe graph-search fallback reached the goal; ' ...
                    'increase episodes if a learned policy is required.'];
            end
        elseif options.generateSmoothTrajectory
            result.message = ['A learned route reached the goal with an audited ' ...
                'smooth executable trajectory.'];
        else
            result.message = 'A collision-free Q-learning policy reached the goal.';
        end
    elseif routeReachedGoal && options.generateSmoothTrajectory
        result.status = "FAILED";
        result.message = ['A collision-free route reached the goal, but no ' ...
            'requested smooth trajectory passed the execution audit: ' ...
            char(string(trajectoryAudit.message))];
    else
        result.status = "FAILED";
        result.message = 'No trained policy reached the goal within the time horizon.';
    end
    result.path = best.path;
    result.routeSuccess = routeReachedGoal;
    result.routeReachedGoal = routeReachedGoal;
    result.grid = grid;
    result.options = options;
    result.requestedStartAzEl_deg = startAzEl_deg;
    result.requestedGoalAzEl_deg = goalAzEl_deg;
    result.goalCells = goalCells;
    result.trajectoryAudit = trajectoryAudit;
    result.diagnostic = struct;
    result.diagnostic.planningTime_s = toc(plannerTic);
    result.diagnostic.requestedParallel = logical(options.useParallel);
    result.diagnostic.numLearners = numLearners;
    result.diagnostic.requestedNumLearners = options.numLearners;
    result.diagnostic.usedParallel = useParallel;
    result.diagnostic.parallelFallbackReason = parallelFallbackReason;
    result.diagnostic.selectedLearner = best.learnerIndex;
    trainedCandidates = candidates(1:numLearners);
    completedEpisodes = sum(cellfun( ...
        @(candidate) candidate.episodesCompleted,trainedCandidates));
    successfulEpisodes = sum(cellfun( ...
        @(candidate) candidate.successfulEpisodes,trainedCandidates));
    result.diagnostic.requestedTrainingEpisodes = options.episodes;
    result.diagnostic.scheduledTrainingEpisodes = ...
        episodesPerLearner*numLearners;
    result.diagnostic.trainingEpisodes = completedEpisodes;
    result.diagnostic.successfulTrainingEpisodes = successfulEpisodes;
    result.diagnostic.trainingSuccessRate = ...
        successfulEpisodes/max(1,completedEpisodes);
    result.diagnostic.trainingStopReasons = string(cellfun( ...
        @(candidate) candidate.trainingStopReason,trainedCandidates, ...
        'UniformOutput',false));
    result.diagnostic.earlyStopped = completedEpisodes < ...
        result.diagnostic.scheduledTrainingEpisodes;
    result.diagnostic.selectedPolicySource = "qLearning";
    if best.learnerIndex == 0
        result.diagnostic.selectedPolicySource = "graphFallback";
    end
    result.diagnostic.selectedPathLength_deg = best.pathLength_deg;
    result.diagnostic.routeSuccess = routeReachedGoal;
    result.diagnostic.routeReachedGoal = routeReachedGoal;
    result.diagnostic.executionSuccess = logical(result.success);
    result.diagnostic.qTableMBPerLearner = qTableMB;
    result.diagnostic.maskOccupiedFraction = maskStats.occupiedFraction;
    result.diagnostic.arrivalTime_s = best.path.time_s(end);
    result.diagnostic.duration_s = best.path.time_s(end)-best.path.time_s(1);
    result.diagnostic.waitTime_s = sum(best.path.isWaiting)*grid.dt_s;
    result.diagnostic.turnCountBeforeSmoothing = best.turnCountBeforeSmoothing;
    result.diagnostic.turnCountAfterSmoothing = best.turnCountAfterSmoothing;
    result.diagnostic.finalGoalDistance_deg = best.finalGoalDistance_deg;
    routeCollisionFree = validatePath( ...
        best.path,blockedByCell,grid,options,actionModel.dynamicPolygons);
    result.diagnostic.routeCollisionFree = routeCollisionFree;
    if options.generateSmoothTrajectory
        result.diagnostic.collisionFree = routeCollisionFree && ...
            trajectoryAudit.success;
    else
        result.diagnostic.collisionFree = routeCollisionFree;
    end
    result.diagnostic.trajectoryAttempts = trajectoryAttempts;
    result.diagnostic.trajectoryAuditSuccessful = trajectoryAudit.success;
    result.diagnostic.trajectoryGenerated = ...
        result.trajectory.success && trajectoryAudit.success;
    result.diagnostic.trajectoryCollisionFree = ...
        trajectoryAudit.collisionFree;
    result.diagnostic.trajectoryKinematicallyFeasible = ...
        trajectoryAudit.kinematicallyFeasible;
    result.diagnostic.trajectoryC2Continuous = trajectoryAudit.c2Continuous;
    result.diagnostic.trajectoryMessage = string(trajectoryAudit.message);
    result.diagnostic.lastFeasible_time_s = best.path.time_s(end);
    result.diagnostic.lastFeasible_azEl_deg = ...
        [best.path.az_deg(end),best.path.el_deg(end)];
    printDiagnostic(result);
end


% =========================================================================
% Configuration and input-data helpers
% =========================================================================
function options = applyDefaults(options,defaults)
    names = fieldnames(defaults);
    for k = 1:numel(names)
        if ~isfield(options,names{k}) || isempty(options.(names{k}))
            options.(names{k}) = defaults.(names{k});
        end
    end
end


function options = applyTrajectoryDefaults(options)
    dependentDefaults = { ...
        'maxAzAcceleration_deg_s2',4*options.azRate_deg_s; ...
        'maxElAcceleration_deg_s2',4*options.elRate_deg_s; ...
        'maxAzJerk_deg_s3',20*options.azRate_deg_s; ...
        'maxElJerk_deg_s3',20*options.elRate_deg_s};
    for row = 1:size(dependentDefaults,1)
        name = dependentDefaults{row,1};
        if ~isfield(options,name) || isempty(options.(name))
            options.(name) = dependentDefaults{row,2};
        end
    end
end


function validatePlannerOptions(options)
    validateattributes(options.azLim_deg,{'numeric'}, ...
        {'row','numel',2,'increasing','finite'},mfilename,'options.azLim_deg');
    validateattributes(options.elLim_deg,{'numeric'}, ...
        {'row','numel',2,'increasing','finite'},mfilename,'options.elLim_deg');
    validateattributes(options.gridStep_deg,{'numeric'}, ...
        {'row','positive','finite'},mfilename,'options.gridStep_deg');
    if ~ismember(numel(options.gridStep_deg),[1,2])
        error('planAzElQLearning:InvalidGridStep', ...
            'options.gridStep_deg must be a scalar or a two-element row vector.');
    end

    positiveScalars = {'azRate_deg_s','elRate_deg_s','maxQTableMB', ...
        'trajectorySampleTime_s','maxAzAcceleration_deg_s2', ...
        'maxElAcceleration_deg_s2','maxAzJerk_deg_s3', ...
        'maxElJerk_deg_s3','trajectoryMaxBlendFraction', ...
        'trajectoryMinBlendFraction'};
    for k = 1:numel(positiveScalars)
        name = positiveScalars{k};
        validateattributes(options.(name),{'numeric'}, ...
            {'scalar','positive','finite'},mfilename,['options.' name]);
    end
    nonnegativeScalars = {'cableReserve_deg','temporalPadding_s', ...
        'goalHold_s','guidanceWeight','progressRewardWeight','stepPenalty', ...
        'waitPenalty','turnPenalty','failurePenalty'};
    for k = 1:numel(nonnegativeScalars)
        name = nonnegativeScalars{k};
        validateattributes(options.(name),{'numeric'}, ...
            {'scalar','nonnegative','finite'},mfilename,['options.' name]);
    end
    validateattributes(options.goalReward,{'numeric'}, ...
        {'scalar','positive','finite'},mfilename,'options.goalReward');

    validateProbability(options.learningRate,'learningRate',false);
    validateProbability(options.discountFactor,'discountFactor',true);
    validateProbability(options.epsilonStart,'epsilonStart',false);
    validateProbability(options.epsilonEnd,'epsilonEnd',false);
    validateProbability(options.guidedExplorationProbability, ...
        'guidedExplorationProbability',true);

    positiveIntegers = {'episodes','minimumEpisodesPerLearner', ...
        'earlyStopSuccessStreak','smoothingMaxLookahead', ...
        'trajectoryBlendAttempts'};
    for k = 1:numel(positiveIntegers)
        name = positiveIntegers{k};
        validateattributes(options.(name),{'numeric'}, ...
            {'scalar','integer','positive','finite'},mfilename,['options.' name]);
    end
    validateattributes(options.numLearners,{'numeric'}, ...
        {'scalar','integer','nonnegative','finite'},mfilename, ...
        'options.numLearners');
    validateattributes(options.randomSeed,{'numeric'}, ...
        {'scalar','integer','nonnegative','finite'},mfilename, ...
        'options.randomSeed');

    optionalTimes = {'startTime_s','endTime_s'};
    for k = 1:numel(optionalTimes)
        name = optionalTimes{k};
        if ~isempty(options.(name))
            validateattributes(options.(name),{'numeric'}, ...
                {'scalar','real','finite'},mfilename,['options.' name]);
        end
    end
    if ~isempty(options.planningHorizon_s)
        validateattributes(options.planningHorizon_s,{'numeric'}, ...
            {'scalar','positive','finite'},mfilename, ...
            'options.planningHorizon_s');
    end

    validateattributes(options.clearance_deg,{'numeric'}, ...
        {'row','nonnegative','finite'},mfilename,'options.clearance_deg');
    if ~ismember(numel(options.clearance_deg),[1,2])
        error('planAzElQLearning:InvalidClearance', ...
            'options.clearance_deg must be a scalar or a two-element row vector.');
    end
    if ~iscell(options.staticPolygons)
        error('planAzElQLearning:InvalidStaticPolygons', ...
            'options.staticPolygons must be a cell array of N-by-2 polygons.');
    end
    if strcmpi(string(options.azimuthTopology),"periodic")
        azimuthSpan_deg = diff(options.azLim_deg);
        if abs(azimuthSpan_deg-360) > 1e-9
            error('planAzElQLearning:InvalidPeriodicAzimuthLimits', ...
                ['Periodic azimuth topology requires options.azLim_deg ' ...
                'to span exactly 360 degrees.']);
        end
        azimuthStep_deg = options.gridStep_deg(1);
        if abs(azimuthSpan_deg/azimuthStep_deg- ...
                round(azimuthSpan_deg/azimuthStep_deg)) > 1e-9
            error('planAzElQLearning:InvalidPeriodicAzimuthStep', ...
                'Periodic azimuth grid spacing must divide 360 degrees.');
        end
        if options.cableReserve_deg > 0
            error('planAzElQLearning:PeriodicCableReserve', ...
                ['options.cableReserve_deg is not compatible with periodic ' ...
                'azimuth topology. Use mechanical topology for cable limits.']);
        end
    end

    logicalOptions = {'preventCornerCutting','goalAzimuthIsWrapped', ...
        'useParallel','smoothPath','generateSmoothTrajectory', ...
        'trajectoryRequireEndpointRest','verbose'};
    for k = 1:numel(logicalOptions)
        validateLogicalScalar(options.(logicalOptions{k}),logicalOptions{k});
    end
    if options.trajectoryMinBlendFraction > ...
            options.trajectoryMaxBlendFraction || ...
            options.trajectoryMaxBlendFraction >= 0.5
        error('planAzElQLearning:InvalidTrajectoryBlendRange', ...
            ['Trajectory blend fractions must satisfy 0 < minimum <= ' ...
            'maximum < 0.5.']);
    end
end


function validateProbability(value,name,allowZero)
    validateattributes(value,{'numeric'}, ...
        {'scalar','real','finite'},mfilename,['options.' name]);
    lowerBoundIsValid = value > 0 || (allowZero && value == 0);
    if ~lowerBoundIsValid || value > 1
        error('planAzElQLearning:InvalidProbability', ...
            'options.%s must be %s and no greater than 1.',name, ...
            ternaryText(allowZero,'between 0','greater than 0'));
    end
end


function validateLogicalScalar(value,name)
    isBinaryNumeric = isscalar(value) && isnumeric(value) && isreal(value) && ...
        isfinite(value) && ...
        (value == 0 || value == 1);
    if ~isscalar(value) || ~(islogical(value) || isBinaryNumeric)
        error('planAzElQLearning:InvalidLogicalOption', ...
            'options.%s must be a logical scalar.',name);
    end
end


function output = ternaryText(condition,trueText,falseText)
    if condition
        output = trueText;
    else
        output = falseText;
    end
end


function sliced = sliceAzElData(data,indices)
    sliced = data;
    sliced.time_s = data.time_s(indices);
    sliced.az_deg = sliceHistory(data.az_deg,indices,numel(data.time_s));
    sliced.el_deg = sliceHistory(data.el_deg,indices,numel(data.time_s));
    if isfield(data,'status') && numel(data.status) == numel(data.time_s)
        sliced.status = data.status(indices);
    end
end


function output = sliceHistory(input,indices,numTimes)
    if iscell(input)
        output = input(indices);
    elseif size(input,1) == numTimes
        output = input(indices,:);
    else
        error('Numeric az/el histories require one row per time sample.');
    end
end


% =========================================================================
% Grid, goal, and action-model construction
% =========================================================================
function [elIndex,azIndex] = snapToGrid( ...
        azEl_deg,azGrid_deg,elGrid_deg,isPeriodic)
    if nargin < 4
        isPeriodic = false;
    end
    if isPeriodic
        [~,azIndex] = min(abs(azel.geometry.shortestAzimuthDeltaDeg( ...
            azEl_deg(1),azGrid_deg)));
    else
        [~,azIndex] = min(abs(azGrid_deg-azEl_deg(1)));
    end
    [~,elIndex] = min(abs(elGrid_deg-azEl_deg(2)));
end


function goalCells = makeGoalCells(goalAzEl_deg,grid,isWrapped)
    numTimes = numel(grid.time_s);
    if size(goalAzEl_deg,1) == 1
        goalAzEl_deg = repmat(goalAzEl_deg,numTimes,1);
    end
    goalCells = cell(numTimes,1);
    for timeIndex = 1:numTimes
        baseAz = goalAzEl_deg(timeIndex,1);
        goalEl = goalAzEl_deg(timeIndex,2);
        if isWrapped
            kMin = ceil((grid.az_deg(1)-baseAz)/360);
            kMax = floor((grid.az_deg(end)-baseAz)/360);
            azCandidates = baseAz+360*(kMin:kMax);
        else
            azCandidates = baseAz;
        end
        ids = zeros(0,1);
        for azCandidate = azCandidates
            if azCandidate < grid.az_deg(1) || azCandidate > grid.az_deg(end) || ...
                    goalEl < grid.el_deg(1) || goalEl > grid.el_deg(end)
                continue
            end
            [elIndex,azIndex] = snapToGrid( ...
                [azCandidate,goalEl],grid.az_deg,grid.el_deg, ...
                isPeriodicGrid(grid));
            ids(end+1,1) = sub2ind( ...
                [numel(grid.el_deg),numel(grid.az_deg)],elIndex,azIndex); %#ok<AGROW>
        end
        goalCells{timeIndex} = unique(ids,'stable');
    end
end


function goalDistance_deg = makeGoalDistance(goalCells,grid)
    [AZ_deg,EL_deg] = meshgrid(grid.az_deg,grid.el_deg);
    cellAz_deg = AZ_deg(:);
    cellEl_deg = EL_deg(:);
    numCells = numel(cellAz_deg);
    numTimes = numel(goalCells);
    goalDistance_deg = inf(numCells,numTimes,'single');
    for timeIndex = 1:numTimes
        ids = goalCells{timeIndex};
        for goalCell = reshape(ids,1,[])
            azimuthDifference_deg = cellAz_deg-cellAz_deg(goalCell);
            if isPeriodicGrid(grid)
                azimuthDifference_deg = ...
                    azel.geometry.shortestAzimuthDeltaDeg( ...
                    cellAz_deg(goalCell),cellAz_deg);
            end
            distance = hypot(azimuthDifference_deg, ...
                cellEl_deg-cellEl_deg(goalCell));
            goalDistance_deg(:,timeIndex) = min( ...
                goalDistance_deg(:,timeIndex),single(distance));
        end
    end
end


function model = makeActionModel(grid,options)
    % [delta elevation index, delta azimuth index]. Action 1 is wait.
    offsets = [0,0;0,-1;0,1;-1,0;1,0;-1,-1;-1,1;1,-1;1,1];
    numEl = numel(grid.el_deg);
    numAz = numel(grid.az_deg);
    numCells = numEl*numAz;
    numActions = size(offsets,1);
    nextCell = zeros(numCells,numActions,'uint32');
    cornerCell1 = zeros(numCells,numActions,'uint32');
    cornerCell2 = zeros(numCells,numActions,'uint32');

    for cellIndex = 1:numCells
        [elIndex,azIndex] = ind2sub([numEl,numAz],cellIndex);
        for action = 1:numActions
            nextEl = elIndex+offsets(action,1);
            nextAz = azIndex+offsets(action,2);
            if isPeriodicGrid(grid)
                nextAz = mod(nextAz-1,numAz)+1;
            end
            if nextEl < 1 || nextEl > numEl || ...
                    nextAz < 1 || nextAz > numAz
                continue
            end
            dAz_deg = abs(grid.az_deg(nextAz)-grid.az_deg(azIndex));
            if isPeriodicGrid(grid)
                dAz_deg = abs(azel.geometry.shortestAzimuthDeltaDeg( ...
                    grid.az_deg(azIndex),grid.az_deg(nextAz)));
            end
            dEl_deg = abs(grid.el_deg(nextEl)-grid.el_deg(elIndex));
            if dAz_deg > options.azRate_deg_s*grid.dt_s+1e-12 || ...
                    dEl_deg > options.elRate_deg_s*grid.dt_s+1e-12
                continue
            end
            fromAz_deg = grid.az_deg(azIndex);
            toAz_deg = grid.az_deg(nextAz);
            if isPeriodicGrid(grid)
                toAz_deg = fromAz_deg+ ...
                    azel.geometry.shortestAzimuthDeltaDeg( ...
                    fromAz_deg,toAz_deg);
            end
            fromAzEl_deg = [fromAz_deg,grid.el_deg(elIndex)];
            toAzEl_deg = [toAz_deg,grid.el_deg(nextEl)];
            if segmentIntersectsStaticPolygons( ...
                    fromAzEl_deg,toAzEl_deg,options.staticPolygons, ...
                    max(options.clearance_deg(:)))
                continue
            end
            nextCell(cellIndex,action) = uint32( ...
                sub2ind([numEl,numAz],nextEl,nextAz));
            if offsets(action,1) ~= 0 && offsets(action,2) ~= 0
                cornerCell1(cellIndex,action) = uint32( ...
                    sub2ind([numEl,numAz],elIndex,nextAz));
                cornerCell2(cellIndex,action) = uint32( ...
                    sub2ind([numEl,numAz],nextEl,azIndex));
            end
        end
    end

    representativeAzStep = median(diff(grid.az_deg));
    representativeElStep = median(diff(grid.el_deg));
    headings = atan2(offsets(:,1)*representativeElStep, ...
        offsets(:,2)*representativeAzStep);
    headings(1) = NaN;
    model = struct('offsets',offsets,'nextCell',nextCell, ...
        'cornerCell1',cornerCell1,'cornerCell2',cornerCell2, ...
        'headings',headings);
end


% =========================================================================
% Dynamic transition-safety model
% =========================================================================
function [edgeSafe,sweptPolygonsByTime] = makeDynamicEdgeSafety( ...
        planningData,grid,actionModel,options)
    numTimes = numel(grid.time_s);
    numCells = size(actionModel.nextCell,1);
    numActions = size(actionModel.nextCell,2);
    sweptPolygonsByTime = cell(numTimes-1,1);
    hasDynamicPolygons = false;
    for timeIndex = 1:numTimes-1
        sweptPolygonsByTime{timeIndex} = makeSweptFramePolygons( ...
            planningData,timeIndex);
        hasDynamicPolygons = hasDynamicPolygons || ...
            ~isempty(sweptPolygonsByTime{timeIndex});
    end
    if ~hasDynamicPolygons
        edgeSafe = [];
        return
    end

    edgeSafe = true(numCells,numActions,numTimes-1);
    [AZ_deg,EL_deg] = meshgrid(grid.az_deg,grid.el_deg);
    cellBuffer_deg = 0.5*hypot(median(diff(grid.az_deg)), ...
        median(diff(grid.el_deg)))+max(options.clearance_deg(:));
    for timeIndex = 1:numTimes-1
        polygons = sweptPolygonsByTime{timeIndex};
        if isempty(polygons)
            continue
        end
        sweptBlocked = rasterizeSafetyPolygons( ...
            polygons,AZ_deg,EL_deg,cellBuffer_deg,isPeriodicGrid(grid));
        sweptBlocked = sweptBlocked(:);
        for action = 1:numActions
            nextCells = double(actionModel.nextCell(:,action));
            valid = nextCells > 0;
            safe = false(numCells,1);
            safe(valid) = ~sweptBlocked(valid) & ...
                ~sweptBlocked(nextCells(valid));
            corner1 = double(actionModel.cornerCell1(:,action));
            corner2 = double(actionModel.cornerCell2(:,action));
            diagonal = corner1 > 0;
            safe(diagonal) = safe(diagonal) & ...
                ~sweptBlocked(corner1(diagonal)) & ...
                ~sweptBlocked(corner2(diagonal));
            edgeSafe(:,action,timeIndex) = safe;
        end
    end
end


function blocked = rasterizeSafetyPolygons( ...
        polygons,AZ_deg,EL_deg,buffer_deg,isPeriodic)
    blocked = false(size(AZ_deg));
    if isPeriodic
        shifts_deg = [-360,0,360];
    else
        shifts_deg = 0;
    end
    for polygonIndex = 1:numel(polygons)
        polygon = polygons{polygonIndex};
        if size(polygon,1) < 2
            continue
        end
        for shift_deg = shifts_deg
            shiftedPolygon = polygon;
            shiftedPolygon(:,1) = shiftedPolygon(:,1)+shift_deg;
            if polyarea(shiftedPolygon(:,1),shiftedPolygon(:,2)) > 1e-10
                blocked = blocked | inpolygon(AZ_deg,EL_deg, ...
                    shiftedPolygon(:,1),shiftedPolygon(:,2));
            end
            if norm(shiftedPolygon(1,:)-shiftedPolygon(end,:)) <= 1e-10
                edgeStart = shiftedPolygon(1:end-1,:);
                edgeEnd = shiftedPolygon(2:end,:);
            else
                edgeStart = shiftedPolygon;
                edgeEnd = shiftedPolygon([2:end,1],:);
            end
            for edgeIndex = 1:size(edgeStart,1)
                edgeVector = edgeEnd(edgeIndex,:)-edgeStart(edgeIndex,:);
                lengthSquared = dot(edgeVector,edgeVector);
                if lengthSquared <= eps
                    distance = hypot(AZ_deg-edgeStart(edgeIndex,1), ...
                        EL_deg-edgeStart(edgeIndex,2));
                else
                    fraction = ((AZ_deg-edgeStart(edgeIndex,1))* ...
                        edgeVector(1)+(EL_deg-edgeStart(edgeIndex,2))* ...
                        edgeVector(2))/lengthSquared;
                    fraction = min(1,max(0,fraction));
                    closestAz_deg = edgeStart(edgeIndex,1)+ ...
                        fraction*edgeVector(1);
                    closestEl_deg = edgeStart(edgeIndex,2)+ ...
                        fraction*edgeVector(2);
                    distance = hypot(AZ_deg-closestAz_deg, ...
                        EL_deg-closestEl_deg);
                end
                blocked = blocked | distance <= buffer_deg+1e-10;
            end
        end
    end
end


function sweptPolygons = makeSweptFramePolygons(data,timeIndex)
    firstPolygons = getFramePolygons(data,timeIndex);
    secondPolygons = getFramePolygons(data,timeIndex+1);
    sweptPolygons = [firstPolygons,secondPolygons];
    if numel(firstPolygons) ~= numel(secondPolygons)
        return
    end
    for polygonIndex = 1:numel(firstPolygons)
        firstPolygon = firstPolygons{polygonIndex};
        secondPolygon = secondPolygons{polygonIndex};
        if size(firstPolygon,1) ~= size(secondPolygon,1)
            continue
        end
        if norm(firstPolygon(1,:)-firstPolygon(end,:)) <= 1e-10
            numEdges = size(firstPolygon,1)-1;
        else
            numEdges = size(firstPolygon,1);
        end
        for edgeIndex = 1:numEdges
            nextIndex = mod(edgeIndex,size(firstPolygon,1))+1;
            sweptPolygons{end+1} = [ ...
                firstPolygon(edgeIndex,:);firstPolygon(nextIndex,:); ...
                secondPolygon(nextIndex,:);secondPolygon(edgeIndex,:); ...
                firstPolygon(edgeIndex,:)]; %#ok<AGROW>
        end
    end
end


function polygons = getFramePolygons(data,timeIndex)
    if iscell(data.az_deg)
        az_deg = data.az_deg{timeIndex};
        el_deg = data.el_deg{timeIndex};
    else
        az_deg = data.az_deg(timeIndex,:);
        el_deg = data.el_deg(timeIndex,:);
    end
    az_deg = az_deg(:);
    el_deg = el_deg(:);
    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators;true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;
    polygons = cell(1,numel(starts));
    for contourIndex = 1:numel(starts)
        indices = starts(contourIndex):ends(contourIndex);
        valid = isfinite(az_deg(indices)) & isfinite(el_deg(indices));
        polygons{contourIndex} = [ ...
            az_deg(indices(valid)),el_deg(indices(valid))];
    end
    polygons = polygons(~cellfun(@isempty,polygons));
end


% =========================================================================
% Q-learning and action selection
% =========================================================================
function [useParallel,numLearners,fallbackReason] = configureParallel(options)
    useParallel = false;
    numLearners = 1;
    fallbackReason = "";
    if ~options.useParallel
        return
    end
    if ~license('test','Distrib_Computing_Toolbox')
        fallbackReason = "Parallel Computing Toolbox license is unavailable.";
        warning('planAzElQLearning:ParallelFallback','%s Running serially.', ...
            fallbackReason);
        return
    end
    try
        pool = gcp('nocreate');
        if isempty(pool)
            pool = parpool;
        end
        if options.numLearners > 0
            numLearners = min(round(options.numLearners),pool.NumWorkers);
        else
            numLearners = pool.NumWorkers;
        end
        useParallel = numLearners > 1;
        if ~useParallel
            fallbackReason = "The available parallel pool has only one worker.";
            warning('planAzElQLearning:ParallelFallback','%s Running serially.', ...
                fallbackReason);
        end
    catch exception
        useParallel = false;
        numLearners = 1;
        fallbackReason = "Parallel pool setup failed: " + ...
            string(exception.message);
        warning('planAzElQLearning:ParallelFallback','%s Running serially.', ...
            fallbackReason);
    end
end


function candidate = trainOneLearner(blockedByCell,grid,startCell, ...
        goalCells,goalDistance_deg,holdSteps,actionModel,options, ...
        numEpisodes,learnerIndex)
    % Candidate contract: route status/path/metrics plus learner metadata.
    % Keeping this as a plain struct makes PARFOR transfer inexpensive.
    rng(options.randomSeed+learnerIndex-1,'twister');
    [numCells,numTimes] = size(blockedByCell);
    numActions = size(actionModel.offsets,1);
    qValues = zeros(numCells*numTimes,numActions,'single');
    successfulEpisodes = 0;
    successStreak = 0;
    episodesCompleted = 0;
    trainingStopReason = "episodeLimit";

    for episode = 1:numEpisodes
        episodesCompleted = episode;
        if numEpisodes == 1
            epsilon = options.epsilonEnd;
        else
            blend = (episode-1)/(numEpisodes-1);
            epsilon = options.epsilonStart* ...
                (options.epsilonEnd/options.epsilonStart)^blend;
        end
        cellIndex = startCell;
        timeIndex = 1;
        previousAction = 0;
        reachedGoal = isGoalState( ...
            cellIndex,timeIndex,goalCells,holdSteps,blockedByCell,actionModel);

        while ~reachedGoal && timeIndex < numTimes
            validActions = getValidActions(cellIndex,timeIndex, ...
                blockedByCell,actionModel,options.preventCornerCutting);
            if isempty(validActions)
                break
            end
            stateIndex = (timeIndex-1)*numCells+cellIndex;
            action = chooseAction(qValues,stateIndex,validActions, ...
                cellIndex,timeIndex,previousAction,goalDistance_deg, ...
                actionModel,options,epsilon,true);
            nextCell = double(actionModel.nextCell(cellIndex,action));
            nextTime = timeIndex+1;

            progress_deg = finiteProgress(goalDistance_deg(cellIndex,timeIndex), ...
                goalDistance_deg(nextCell,nextTime));
            turnFraction = actionTurnFraction( ...
                previousAction,action,actionModel);
            reward = -options.stepPenalty+ ...
                options.progressRewardWeight*progress_deg- ...
                options.turnPenalty*turnFraction;
            if action == 1
                reward = reward-options.waitPenalty;
            end

            reachedGoal = isGoalState( ...
                nextCell,nextTime,goalCells,holdSteps,blockedByCell,actionModel);
            if reachedGoal
                reward = reward+options.goalReward;
                target = reward;
            else
                nextValidActions = getValidActions(nextCell,nextTime, ...
                    blockedByCell,actionModel,options.preventCornerCutting);
                if isempty(nextValidActions)
                    reward = reward-options.failurePenalty;
                    target = reward;
                else
                    nextStateIndex = nextTime*numCells+nextCell;
                    target = reward+options.discountFactor*double(max( ...
                        qValues(nextStateIndex,nextValidActions)));
                end
            end
            oldValue = double(qValues(stateIndex,action));
            qValues(stateIndex,action) = single(oldValue+ ...
                options.learningRate*(target-oldValue));

            cellIndex = nextCell;
            timeIndex = nextTime;
            if action ~= 1
                previousAction = action;
            end
        end

        if reachedGoal
            successfulEpisodes = successfulEpisodes+1;
            successStreak = successStreak+1;
        else
            successStreak = 0;
        end
        if episode >= options.minimumEpisodesPerLearner && ...
                successStreak >= options.earlyStopSuccessStreak
            trainingStopReason = "successStreak";
            break
        end
    end

    candidate = extractPolicy(qValues,blockedByCell,grid,startCell, ...
        goalCells,goalDistance_deg,holdSteps,actionModel,options);
    candidate.learnerIndex = learnerIndex;
    candidate.episodesCompleted = episodesCompleted;
    candidate.successfulEpisodes = successfulEpisodes;
    candidate.trainingStopReason = trainingStopReason;
end


function action = chooseAction(qValues,stateIndex,validActions,cellIndex, ...
        timeIndex,previousAction,goalDistance_deg,actionModel,options, ...
        epsilon,isTraining)
    if isTraining && rand < epsilon && ...
            rand > options.guidedExplorationProbability
        action = validActions(randi(numel(validActions)));
        return
    end

    if isempty(qValues)
        scores = zeros(size(validActions));
    else
        scores = double(qValues(stateIndex,validActions));
    end
    for k = 1:numel(validActions)
        testAction = validActions(k);
        nextCell = double(actionModel.nextCell(cellIndex,testAction));
        progress = finiteProgress(goalDistance_deg(cellIndex,timeIndex), ...
            goalDistance_deg(nextCell,timeIndex+1));
        scores(k) = scores(k)+options.guidanceWeight*progress- ...
            options.turnPenalty*actionTurnFraction( ...
            previousAction,testAction,actionModel);
        if testAction == 1
            scores(k) = scores(k)-options.waitPenalty;
        end
    end
    if isTraining
        scores = scores+1e-8*rand(size(scores));
    end
    [~,bestIndex] = max(scores);
    action = validActions(bestIndex);
end


function progress = finiteProgress(currentDistance,nextDistance)
    if isfinite(currentDistance) && isfinite(nextDistance)
        progress = double(currentDistance-nextDistance);
    else
        progress = 0;
    end
end


function fraction = actionTurnFraction(previousAction,action,actionModel)
    if previousAction <= 1 || action <= 1
        fraction = 0;
        return
    end
    angleChange = mod(actionModel.headings(action)- ...
        actionModel.headings(previousAction)+pi,2*pi)-pi;
    fraction = abs(angleChange)/pi;
end


function validActions = getValidActions(cellIndex,timeIndex, ...
        blockedByCell,actionModel,preventCornerCutting)
    if timeIndex >= size(blockedByCell,2)
        validActions = zeros(1,0);
        return
    end
    nextCells = double(actionModel.nextCell(cellIndex,:));
    validActions = find(nextCells > 0);
    if isempty(validActions)
        return
    end
    destinations = nextCells(validActions);
    destinationFree = ~blockedByCell( ...
        sub2ind(size(blockedByCell),destinations, ...
        repmat(timeIndex+1,size(destinations))));
    validActions = validActions(destinationFree);

    if ~isempty(actionModel.dynamicEdgeSafe) && ~isempty(validActions)
        dynamicIndices = sub2ind(size(actionModel.dynamicEdgeSafe), ...
            repmat(cellIndex,size(validActions)),validActions, ...
            repmat(timeIndex,size(validActions)));
        validActions = validActions( ...
            actionModel.dynamicEdgeSafe(dynamicIndices));
    end

    if preventCornerCutting
        keep = true(size(validActions));
        for k = 1:numel(validActions)
            action = validActions(k);
            corner1 = double(actionModel.cornerCell1(cellIndex,action));
            corner2 = double(actionModel.cornerCell2(cellIndex,action));
            if corner1 > 0 && (blockedByCell(corner1,timeIndex) || ...
                    blockedByCell(corner2,timeIndex) || ...
                    blockedByCell(corner1,timeIndex+1) || ...
                    blockedByCell(corner2,timeIndex+1))
                keep(k) = false;
            end
        end
        validActions = validActions(keep);
    end
end


function isGoal = isGoalState( ...
        cellIndex,timeIndex,goalCells,holdSteps,blocked,actionModel)
    isGoal = false;
    if timeIndex+holdSteps > numel(goalCells)
        return
    end
    for testTime = timeIndex:timeIndex+holdSteps
        if ~any(goalCells{testTime} == cellIndex) || blocked(cellIndex,testTime)
            return
        end
        if testTime < timeIndex+holdSteps
            if actionModel.nextCell(cellIndex,1) == 0
                return
            end
            if ~isempty(actionModel.dynamicEdgeSafe) && ...
                    ~actionModel.dynamicEdgeSafe(cellIndex,1,testTime)
                return
            end
        end
    end
    isGoal = true;
end


% =========================================================================
% Policy extraction and deterministic graph fallback
% =========================================================================
function candidate = extractPolicy(qValues,blockedByCell,grid,startCell, ...
        goalCells,goalDistance_deg,holdSteps,actionModel,options)
    [numCells,numTimes] = size(blockedByCell);
    cellHistory = zeros(numTimes,1);
    timeHistory = zeros(numTimes,1);
    cellHistory(1) = startCell;
    timeHistory(1) = 1;
    pathCount = 1;
    cellIndex = startCell;
    timeIndex = 1;
    previousAction = 0;
    success = isGoalState( ...
        cellIndex,timeIndex,goalCells,holdSteps,blockedByCell,actionModel);
    bestDistance = double(goalDistance_deg(cellIndex,timeIndex));
    bestPathCount = 1;

    while ~success && timeIndex < numTimes
        validActions = getValidActions(cellIndex,timeIndex, ...
            blockedByCell,actionModel,options.preventCornerCutting);
        if isempty(validActions)
            break
        end
        stateIndex = (timeIndex-1)*numCells+cellIndex;
        action = chooseAction(qValues,stateIndex,validActions,cellIndex, ...
            timeIndex,previousAction,goalDistance_deg,actionModel,options,0,false);
        cellIndex = double(actionModel.nextCell(cellIndex,action));
        timeIndex = timeIndex+1;
        pathCount = pathCount+1;
        cellHistory(pathCount) = cellIndex;
        timeHistory(pathCount) = timeIndex;
        if action ~= 1
            previousAction = action;
        end

        distance = double(goalDistance_deg(cellIndex,timeIndex));
        if distance < bestDistance
            bestDistance = distance;
            bestPathCount = pathCount;
        end
        success = isGoalState( ...
            cellIndex,timeIndex,goalCells,holdSteps,blockedByCell,actionModel);
    end

    if ~success
        pathCount = bestPathCount;
        cellHistory = cellHistory(1:pathCount);
        timeHistory = timeHistory(1:pathCount);
    else
        cellHistory = cellHistory(1:pathCount);
        timeHistory = timeHistory(1:pathCount);
    end
    path = cellsToPath(cellHistory,timeHistory,grid);
    turnCountBefore = countPathTurns(path);
    if options.smoothPath
        smoothedPath = smoothTimedPath(path,blockedByCell,grid,options, ...
            actionModel.dynamicPolygons);
        if validatePath(smoothedPath,blockedByCell,grid,options, ...
                actionModel.dynamicPolygons)
            path = smoothedPath;
        end
    end

    candidate = struct;
    candidate.success = success;
    candidate.path = path;
    candidate.turnCountBeforeSmoothing = turnCountBefore;
    candidate.turnCountAfterSmoothing = countPathTurns(path);
    candidate.finalGoalDistance_deg = double(goalDistance_deg( ...
        cellHistory(end),timeHistory(end)));
    candidate.pathLength_deg = sum(hypot( ...
        diff(path.az_deg),diff(path.el_deg)));
end


function candidate = extractGraphFallback(blockedByCell,grid,startCell, ...
        goalCells,goalDistance_deg,holdSteps,actionModel,options)
    [numCells,numTimes] = size(blockedByCell);
    predecessorCell = zeros(numCells,numTimes,'uint32');
    currentReachable = false(numCells,1);
    currentReachable(startCell) = true;
    terminalCell = startCell;
    terminalTime = 1;
    bestDistance = double(goalDistance_deg(startCell,1));
    success = isGoalState( ...
        startCell,1,goalCells,holdSteps,blockedByCell,actionModel);

    for timeIndex = 1:numTimes-1
        if success
            break
        end
        currentCells = find(currentReachable);
        if isempty(currentCells)
            break
        end
        [~,cellOrder] = sort(double( ...
            goalDistance_deg(currentCells,timeIndex)),'ascend');
        currentCells = currentCells(cellOrder);
        nextReachable = false(numCells,1);

        for currentNumber = 1:numel(currentCells)
            cellIndex = currentCells(currentNumber);
            validActions = getValidActions(cellIndex,timeIndex, ...
                blockedByCell,actionModel,options.preventCornerCutting);
            nextCells = double(actionModel.nextCell(cellIndex,validActions));
            actionScores = double(goalDistance_deg(nextCells,timeIndex+1));
            actionScores(validActions == 1) = ...
                actionScores(validActions == 1)+options.waitPenalty;
            [~,actionOrder] = sort(actionScores,'ascend');
            nextCells = nextCells(actionOrder);

            for nextNumber = 1:numel(nextCells)
                nextCell = nextCells(nextNumber);
                if nextReachable(nextCell)
                    continue
                end
                nextReachable(nextCell) = true;
                predecessorCell(nextCell,timeIndex+1) = uint32(cellIndex);
                distance = double(goalDistance_deg(nextCell,timeIndex+1));
                if distance < bestDistance
                    bestDistance = distance;
                    terminalCell = nextCell;
                    terminalTime = timeIndex+1;
                end
                if isGoalState(nextCell,timeIndex+1,goalCells, ...
                        holdSteps,blockedByCell,actionModel)
                    success = true;
                    terminalCell = nextCell;
                    terminalTime = timeIndex+1;
                    break
                end
            end
            if success
                break
            end
        end
        currentReachable = nextReachable;
    end

    cellHistory = zeros(terminalTime,1);
    cellHistory(terminalTime) = terminalCell;
    for timeIndex = terminalTime:-1:2
        cellHistory(timeIndex-1) = double( ...
            predecessorCell(cellHistory(timeIndex),timeIndex));
    end
    timeHistory = (1:terminalTime)';
    path = cellsToPath(cellHistory,timeHistory,grid);
    turnCountBefore = countPathTurns(path);
    if options.smoothPath
        smoothedPath = smoothTimedPath(path,blockedByCell,grid,options, ...
            actionModel.dynamicPolygons);
        if validatePath(smoothedPath,blockedByCell,grid,options, ...
                actionModel.dynamicPolygons)
            path = smoothedPath;
        end
    end

    candidate = struct;
    candidate.success = success;
    candidate.path = path;
    candidate.turnCountBeforeSmoothing = turnCountBefore;
    candidate.turnCountAfterSmoothing = countPathTurns(path);
    candidate.finalGoalDistance_deg = double(goalDistance_deg( ...
        terminalCell,terminalTime));
    candidate.pathLength_deg = sum(hypot( ...
        diff(path.az_deg),diff(path.el_deg)));
end


% =========================================================================
% Timed-path postprocessing and continuous route geometry
% =========================================================================
function path = cellsToPath(cellHistory,timeHistory,grid)
    [elIndex,azIndex] = ind2sub( ...
        [numel(grid.el_deg),numel(grid.az_deg)],cellHistory);
    path = struct;
    path.planningTimeIndex = timeHistory(:);
    path.timeIndex = grid.originalTimeIndex(timeHistory);
    path.time_s = grid.time_s(timeHistory);
    wrappedAz_deg = reshape(grid.az_deg(azIndex),[],1);
    if isPeriodicGrid(grid)
        path.az_deg = azel.geometry.unwrapAzimuthDeg( ...
            wrappedAz_deg,wrappedAz_deg(1));
        path.azWrapped_deg = mod(path.az_deg-grid.azLim_deg(1),360)+ ...
            grid.azLim_deg(1);
    else
        path.az_deg = wrappedAz_deg;
        path.azWrapped_deg = wrappedAz_deg;
    end
    path.el_deg = reshape(grid.el_deg(elIndex),[],1);
    path.isWaiting = [false;abs(diff(path.az_deg)) < 1e-12 & ...
        abs(diff(path.el_deg)) < 1e-12];
end


function path = smoothTimedPath( ...
        path,blockedByCell,grid,options,dynamicPolygons)
    numSamples = numel(path.timeIndex);
    if numSamples < 3
        return
    end
    stationaryEdge = hypot(diff(path.az_deg),diff(path.el_deg)) < 1e-12;
    fixedSample = false(numSamples,1);
    fixedSample([1,end]) = true;
    fixedSample(1:end-1) = fixedSample(1:end-1) | stationaryEdge;
    fixedSample(2:end) = fixedSample(2:end) | stationaryEdge;
    fixedIndices = find(fixedSample);
    smoothedAz_deg = path.az_deg;
    smoothedEl_deg = path.el_deg;
    maxLookahead = max(2,round(options.smoothingMaxLookahead));

    for fixedNumber = 1:numel(fixedIndices)-1
        rangeEnd = fixedIndices(fixedNumber+1);
        anchor = fixedIndices(fixedNumber);
        while anchor < rangeEnd
            furthestCandidate = min(rangeEnd,anchor+maxLookahead);
            accepted = anchor+1;
            for candidate = furthestCandidate:-1:anchor+1
                if straightSegmentIsSafe(path,anchor,candidate, ...
                        blockedByCell,grid,options,dynamicPolygons)
                    accepted = candidate;
                    break
                end
            end
            sampleCount = accepted-anchor+1;
            smoothedAz_deg(anchor:accepted) = linspace( ...
                path.az_deg(anchor),path.az_deg(accepted),sampleCount)';
            smoothedEl_deg(anchor:accepted) = linspace( ...
                path.el_deg(anchor),path.el_deg(accepted),sampleCount)';
            anchor = accepted;
        end
    end
    path.az_deg = smoothedAz_deg;
    path.el_deg = smoothedEl_deg;
    path.isWaiting = [false;abs(diff(path.az_deg)) < 1e-12 & ...
        abs(diff(path.el_deg)) < 1e-12];
end


function isSafe = straightSegmentIsSafe(path,firstIndex,lastIndex, ...
        blockedByCell,grid,options,dynamicPolygons)
    numSteps = lastIndex-firstIndex;
    dAzPerStep = abs(path.az_deg(lastIndex)-path.az_deg(firstIndex))/numSteps;
    dElPerStep = abs(path.el_deg(lastIndex)-path.el_deg(firstIndex))/numSteps;
    if dAzPerStep > options.azRate_deg_s*grid.dt_s+1e-10 || ...
            dElPerStep > options.elRate_deg_s*grid.dt_s+1e-10
        isSafe = false;
        return
    end
    if segmentIntersectsStaticPolygons( ...
            [path.az_deg(firstIndex),path.el_deg(firstIndex)], ...
            [path.az_deg(lastIndex),path.el_deg(lastIndex)], ...
            options.staticPolygons,max(options.clearance_deg(:)))
        isSafe = false;
        return
    end
    azSamples_deg = linspace(path.az_deg(firstIndex), ...
        path.az_deg(lastIndex),numSteps+1);
    elSamples_deg = linspace(path.el_deg(firstIndex), ...
        path.el_deg(lastIndex),numSteps+1);
    for localIndex = 1:numSteps
        timeIndex = path.planningTimeIndex(firstIndex+localIndex-1);
        if segmentIntersectsStaticPolygons( ...
                [azSamples_deg(localIndex),elSamples_deg(localIndex)], ...
                [azSamples_deg(localIndex+1),elSamples_deg(localIndex+1)], ...
                dynamicPolygons{timeIndex},max(options.clearance_deg(:)))
            isSafe = false;
            return
        end
    end
    isSafe = true;
    for localIndex = 0:numSteps
        azIndex = nearestAzimuthIndex( ...
            grid.az_deg,azSamples_deg(localIndex+1),isPeriodicGrid(grid));
        [~,elIndex] = min(abs(grid.el_deg-elSamples_deg(localIndex+1)));
        cellIndex = sub2ind( ...
            [numel(grid.el_deg),numel(grid.az_deg)],elIndex,azIndex);
        timeIndex = path.planningTimeIndex(firstIndex+localIndex);
        if blockedByCell(cellIndex,timeIndex)
            isSafe = false;
            return
        end
    end
end


function valid = validatePath( ...
        path,blockedByCell,grid,options,dynamicPolygons)
    valid = true;
    for k = 1:numel(path.timeIndex)
        azIndex = nearestAzimuthIndex( ...
            grid.az_deg,path.az_deg(k),isPeriodicGrid(grid));
        [~,elIndex] = min(abs(grid.el_deg-path.el_deg(k)));
        cellIndex = sub2ind( ...
            [numel(grid.el_deg),numel(grid.az_deg)],elIndex,azIndex);
        if blockedByCell(cellIndex,path.planningTimeIndex(k))
            valid = false;
            return
        end
        pointAzEl_deg = [path.az_deg(k),path.el_deg(k)];
        if segmentIntersectsStaticPolygons(pointAzEl_deg,pointAzEl_deg, ...
                options.staticPolygons,max(options.clearance_deg(:)))
            valid = false;
            return
        end
        transitionIndex = path.planningTimeIndex(k);
        if transitionIndex <= numel(dynamicPolygons) && ...
                segmentIntersectsStaticPolygons( ...
                pointAzEl_deg,pointAzEl_deg, ...
                dynamicPolygons{transitionIndex}, ...
                max(options.clearance_deg(:)))
            valid = false;
            return
        end
    end
    for k = 2:numel(path.timeIndex)
        if segmentIntersectsStaticPolygons( ...
                [path.az_deg(k-1),path.el_deg(k-1)], ...
                [path.az_deg(k),path.el_deg(k)],options.staticPolygons, ...
                max(options.clearance_deg(:)))
            valid = false;
            return
        end
        timeIndex = path.planningTimeIndex(k-1);
        if segmentIntersectsStaticPolygons( ...
                [path.az_deg(k-1),path.el_deg(k-1)], ...
                [path.az_deg(k),path.el_deg(k)], ...
                dynamicPolygons{timeIndex},max(options.clearance_deg(:)))
            valid = false;
            return
        end
    end
end


function azIndex = nearestAzimuthIndex(azGrid_deg,azimuth_deg,isPeriodic)
    if isPeriodic
        [~,azIndex] = min(abs(azel.geometry.shortestAzimuthDeltaDeg( ...
            azimuth_deg,azGrid_deg)));
    else
        [~,azIndex] = min(abs(azGrid_deg-azimuth_deg));
    end
end


function periodic = isPeriodicGrid(grid)
    periodic = isfield(grid,'azimuthTopology') && ...
        strcmpi(string(grid.azimuthTopology),"periodic");
end


function intersects = segmentIntersectsStaticPolygons( ...
        firstPoint,secondPoint,staticPolygons,clearance_deg)
    intersects = false;
    for polygonIndex = 1:numel(staticPolygons)
        polygon = staticPolygons{polygonIndex};
        if isempty(polygon)
            continue
        end
        [insideFirst,onFirst] = inpolygon( ...
            firstPoint(1),firstPoint(2),polygon(:,1),polygon(:,2));
        [insideSecond,onSecond] = inpolygon( ...
            secondPoint(1),secondPoint(2),polygon(:,1),polygon(:,2));
        boundaryIntersection = segmentIntersectsPolygonBoundary( ...
            firstPoint,secondPoint,polygon);
        clearanceViolation = clearance_deg > 0 && ...
            minimumSegmentPolygonDistance( ...
            firstPoint,secondPoint,polygon) <= clearance_deg+1e-10;
        if insideFirst || onFirst || insideSecond || onSecond || ...
                boundaryIntersection || clearanceViolation
            intersects = true;
            return
        end
    end
end


function distance = minimumSegmentPolygonDistance( ...
        firstPoint,secondPoint,polygon)
    tolerance = 1e-10;
    if norm(polygon(1,:)-polygon(end,:)) <= tolerance
        edgeStart = polygon(1:end-1,:);
        edgeEnd = polygon(2:end,:);
    else
        edgeStart = polygon;
        edgeEnd = polygon([2:end,1],:);
    end
    distanceFromFirst = pointToSegmentsDistance( ...
        firstPoint,edgeStart,edgeEnd);
    distanceFromSecond = pointToSegmentsDistance( ...
        secondPoint,edgeStart,edgeEnd);
    pathStart = repmat(firstPoint,size(edgeStart,1),1);
    pathEnd = repmat(secondPoint,size(edgeStart,1),1);
    distanceFromEdgeStart = pointToSegmentsDistance( ...
        edgeStart,pathStart,pathEnd);
    distanceFromEdgeEnd = pointToSegmentsDistance( ...
        edgeEnd,pathStart,pathEnd);
    distance = min([distanceFromFirst;distanceFromSecond; ...
        distanceFromEdgeStart;distanceFromEdgeEnd]);
end


function distance = pointToSegmentsDistance(point,segmentStart,segmentEnd)
    if size(point,1) == 1 && size(segmentStart,1) > 1
        point = repmat(point,size(segmentStart,1),1);
    end
    segmentVector = segmentEnd-segmentStart;
    lengthSquared = sum(segmentVector.^2,2);
    fraction = zeros(size(lengthSquared));
    nonzero = lengthSquared > eps;
    offset = point-segmentStart;
    fraction(nonzero) = sum(offset(nonzero,:).* ...
        segmentVector(nonzero,:),2)./lengthSquared(nonzero);
    fraction = min(1,max(0,fraction));
    closestPoint = segmentStart+fraction.*segmentVector;
    distance = hypot(point(:,1)-closestPoint(:,1), ...
        point(:,2)-closestPoint(:,2));
end


function intersects = segmentIntersectsPolygonBoundary( ...
        firstPoint,secondPoint,polygon)
    tolerance = 1e-10;
    if norm(secondPoint-firstPoint) <= tolerance
        intersects = false;
        return
    end
    if norm(polygon(1,:)-polygon(end,:)) <= tolerance
        edgeStart = polygon(1:end-1,:);
        edgeEnd = polygon(2:end,:);
    else
        edgeStart = polygon;
        edgeEnd = polygon([2:end,1],:);
    end

    pathVector = secondPoint-firstPoint;
    edgeVector = edgeEnd-edgeStart;
    offset = edgeStart-firstPoint;
    denominator = cross2d(repmat(pathVector,size(edgeVector,1),1), ...
        edgeVector);
    nonparallel = abs(denominator) > tolerance;
    pathFraction = zeros(size(denominator));
    edgeFraction = zeros(size(denominator));
    pathFraction(nonparallel) = cross2d( ...
        offset(nonparallel,:),edgeVector(nonparallel,:))./ ...
        denominator(nonparallel);
    edgeFraction(nonparallel) = cross2d( ...
        offset(nonparallel,:), ...
        repmat(pathVector,sum(nonparallel),1))./denominator(nonparallel);
    if any(nonparallel & pathFraction >= -tolerance & ...
            pathFraction <= 1+tolerance & edgeFraction >= -tolerance & ...
            edgeFraction <= 1+tolerance)
        intersects = true;
        return
    end

    parallel = ~nonparallel;
    collinear = parallel & abs(cross2d(offset, ...
        repmat(pathVector,size(offset,1),1))) <= tolerance;
    if ~any(collinear)
        intersects = false;
        return
    end
    pathLengthSquared = dot(pathVector,pathVector);
    startProjection = ((edgeStart(:,1)-firstPoint(1))*pathVector(1)+ ...
        (edgeStart(:,2)-firstPoint(2))*pathVector(2))/pathLengthSquared;
    endProjection = ((edgeEnd(:,1)-firstPoint(1))*pathVector(1)+ ...
        (edgeEnd(:,2)-firstPoint(2))*pathVector(2))/pathLengthSquared;
    overlaps = max(min(startProjection,endProjection),0) <= ...
        min(max(startProjection,endProjection),1)+tolerance;
    intersects = any(collinear & overlaps);
end


function value = cross2d(firstVector,secondVector)
    value = firstVector(:,1).*secondVector(:,2)- ...
        firstVector(:,2).*secondVector(:,1);
end


function turnCount = countPathTurns(path)
    dAz = diff(path.az_deg);
    dEl = diff(path.el_deg);
    moving = hypot(dAz,dEl) > 1e-12;
    headings = atan2(dEl(moving),dAz(moving));
    if numel(headings) < 2
        turnCount = 0;
        return
    end
    headingChange = mod(diff(headings)+pi,2*pi)-pi;
    turnCount = sum(abs(headingChange) > deg2rad(1));
end


% =========================================================================
% Candidate ranking and executable-command selection
% =========================================================================
function bestIndex = selectBestCandidate(candidates)
    order = rankCandidates(candidates);
    bestIndex = order(1);
end


function order = rankSuccessfulCandidates(candidates)
    order = rankCandidates(candidates);
    successful = cellfun(@(candidate) candidate.success,candidates);
    order = order(successful(order));
end


function order = rankCandidates(candidates)
    % Lexicographic priority is deliberate: successful routes first, then
    % earliest arrival, fewer turns, and shorter length. Failed candidates
    % instead prefer closest approach, then the latest feasible time.
    numCandidates = numel(candidates);
    metrics = zeros(numCandidates,4);
    for k = 1:numCandidates
        candidate = candidates{k};
        if candidate.success
            metrics(k,:) = [0,candidate.path.time_s(end), ...
                candidate.turnCountAfterSmoothing,candidate.pathLength_deg];
        else
            metrics(k,:) = [1,candidate.finalGoalDistance_deg, ...
                -candidate.path.time_s(end),candidate.turnCountAfterSmoothing];
        end
    end
    [~,order] = sortrows(metrics,[1,2,3,4]);
end


function present = hasGraphFallbackCandidate(candidates)
    present = any(cellfun(@(candidate) ...
        isfield(candidate,'learnerIndex') && candidate.learnerIndex == 0, ...
        candidates));
end


function candidate = makeGraphFallbackCandidate( ...
        blockedByCell,grid,startCell,goalCells,goalDistance_deg, ...
        holdSteps,actionModel,options)
    candidate = extractGraphFallback(blockedByCell,grid,startCell, ...
        goalCells,goalDistance_deg,holdSteps,actionModel,options);
    candidate.learnerIndex = 0;
    candidate.episodesCompleted = 0;
    candidate.successfulEpisodes = 0;
    candidate.trainingStopReason = "notRun";
end


function [trajectory,audit,passed] = makeAuditedTrajectory( ...
        candidate,planningData,grid,options)
    if numel(candidate.path.time_s) < 2
        trajectory = pointTrajectoryResult(candidate.path,grid,options);
        audit = auditPointTrajectory(candidate.path,planningData,options);
        passed = trajectory.success && audit.success;
        if ~passed
            trajectory.success = false;
            trajectory.message = string(audit.message);
        end
        return
    end

    [trajectory,trajectoryInfo] = ...
        azel.trajectory.smoothAzElTrajectory( ...
        candidate.path,planningData,options);
    if ~trajectory.success || isempty(trajectory.segments)
        audit = unsuccessfulTrajectoryAudit(trajectoryInfo.message);
        trajectory.success = false;
        passed = false;
        return
    end

    trajectoryScenario = struct('data',planningData,'options',options);
    audit = azel.audit.auditAzElTrajectory( ...
        trajectoryScenario,trajectory,true);
    audit = normalizeTrajectoryAudit(audit);
    passed = trajectory.success && audit.success;
    if ~passed
        trajectory.success = false;
        trajectory.message = string(audit.message);
    end
end


function audit = auditPointTrajectory(path,planningData,options)
    routeResult = struct('success',true,'path',path);
    pathAudit = azel.audit.auditPlannerPath( ...
        struct('data',planningData,'options',options),routeResult);
    mechanicalBoundsSafe = pointInsidePlannerLimits( ...
        [path.az_deg(1),path.el_deg(1)],options);
    collisionFree = isfield(pathAudit,'collisionFree') && ...
        pathAudit.collisionFree;
    holdSafe = isfield(pathAudit,'holdSafe') && pathAudit.holdSafe;
    audit = struct( ...
        'success',collisionFree && holdSafe && mechanicalBoundsSafe, ...
        'collisionFree',collisionFree,'holdSafe',holdSafe, ...
        'kinematicallyFeasible',true, ...
        'mechanicalBoundsSafe',mechanicalBoundsSafe, ...
        'c2Continuous',true,'message',string(pathAudit.message));
    if ~mechanicalBoundsSafe
        audit.message = "stationary trajectory lies outside the mechanical limits";
    end
end


function inside = pointInsidePlannerLimits(point_deg,options)
    tolerance = 1e-10;
    periodic = strcmpi(string(options.azimuthTopology),"periodic");
    azimuthInside = periodic || (point_deg(1) >= options.azLim_deg(1)-tolerance && ...
        point_deg(1) <= options.azLim_deg(2)+tolerance);
    elevationInside = point_deg(2) >= options.elLim_deg(1)-tolerance && ...
        point_deg(2) <= options.elLim_deg(2)+tolerance;
    inside = azimuthInside && elevationInside;
end


function audit = normalizeTrajectoryAudit(audit)
    defaults = unsuccessfulTrajectoryAudit("trajectory audit failed");
    names = fieldnames(defaults);
    for fieldIndex = 1:numel(names)
        name = names{fieldIndex};
        if ~isfield(audit,name)
            audit.(name) = defaults.(name);
        end
    end
    audit.success = logical(audit.collisionFree && audit.holdSafe && ...
        audit.kinematicallyFeasible && audit.mechanicalBoundsSafe && ...
        audit.c2Continuous);
end


function audit = unsuccessfulTrajectoryAudit(message)
    audit = struct('success',false,'collisionFree',false, ...
        'holdSafe',false,'kinematicallyFeasible',false, ...
        'mechanicalBoundsSafe',false,'c2Continuous',false, ...
        'message',string(message));
end


function trajectory = emptyTrajectoryResult(message)
    trajectory = struct( ...
        'success',false,'message',string(message), ...
        'time_s',zeros(0,1),'az_deg',zeros(0,1), ...
        'azWrapped_deg',zeros(0,1),'el_deg',zeros(0,1), ...
        'timeIndex',zeros(0,1),'planningTimeIndex',zeros(0,1), ...
        'isWaiting',false(0,1),'azRate_deg_s',zeros(0,1), ...
        'elRate_deg_s',zeros(0,1), ...
        'azAcceleration_deg_s2',zeros(0,1), ...
        'elAcceleration_deg_s2',zeros(0,1), ...
        'azJerk_deg_s3',zeros(0,1),'elJerk_deg_s3',zeros(0,1), ...
        'segments',struct('startTime_s',{},'endTime_s',{}, ...
        'controlPoints_deg',{},'kind',{}), ...
        'c2Continuous',false,'numFillets',0,'maxHeadingJump_deg',NaN);
end


function trajectory = pointTrajectoryResult(path,grid,options)
    trajectory = emptyTrajectoryResult( ...
        "stationary point requires no slew trajectory");
    trajectory.success = true;
    trajectory.time_s = path.time_s(:);
    trajectory.az_deg = path.az_deg(:);
    trajectory.el_deg = path.el_deg(:);
    if isfield(path,'azWrapped_deg')
        trajectory.azWrapped_deg = path.azWrapped_deg(:);
    elseif strcmpi(string(options.azimuthTopology),"periodic")
        trajectory.azWrapped_deg = mod(path.az_deg-grid.azLim_deg(1),360)+ ...
            grid.azLim_deg(1);
    else
        trajectory.azWrapped_deg = path.az_deg(:);
    end
    trajectory.timeIndex = path.timeIndex(:);
    trajectory.planningTimeIndex = path.planningTimeIndex(:);
    trajectory.isWaiting = true(size(path.time_s(:)));
    trajectory.azRate_deg_s = zeros(size(path.time_s(:)));
    trajectory.elRate_deg_s = zeros(size(path.time_s(:)));
    trajectory.azAcceleration_deg_s2 = zeros(size(path.time_s(:)));
    trajectory.elAcceleration_deg_s2 = zeros(size(path.time_s(:)));
    trajectory.azJerk_deg_s3 = zeros(size(path.time_s(:)));
    trajectory.elJerk_deg_s3 = zeros(size(path.time_s(:)));
    trajectory.c2Continuous = true;
    trajectory.maxHeadingJump_deg = 0;
end


% =========================================================================
% Result construction and diagnostics
% =========================================================================
function result = failedStartResult(options,grid,maskStats,startAzEl_deg, ...
        goalAzEl_deg,message)
    result = struct;
    result.success = false;
    result.routeSuccess = false;
    result.routeReachedGoal = false;
    result.status = "FAILED";
    result.message = message;
    result.path = struct('planningTimeIndex',[],'timeIndex',[], ...
        'time_s',[],'az_deg',[],'el_deg',[],'isWaiting',[]);
    result.trajectory = emptyTrajectoryResult(message);
    result.trajectoryAudit = unsuccessfulTrajectoryAudit(message);
    result.grid = grid;
    result.options = options;
    result.requestedStartAzEl_deg = startAzEl_deg;
    result.requestedGoalAzEl_deg = goalAzEl_deg;
    result.goalCells = {};
    result.diagnostic = struct('planningTime_s',0, ...
        'requestedParallel',logical(options.useParallel),'numLearners',0, ...
        'requestedNumLearners',options.numLearners,'usedParallel',false, ...
        'parallelFallbackReason',"",'selectedLearner',NaN, ...
        'requestedTrainingEpisodes',options.episodes, ...
        'scheduledTrainingEpisodes',0,'trainingEpisodes',0, ...
        'successfulTrainingEpisodes',0,'trainingSuccessRate',0, ...
        'trainingStopReasons',strings(0,1),'earlyStopped',false, ...
        'selectedPolicySource',"none",'selectedPathLength_deg',NaN, ...
        'routeSuccess',false,'routeReachedGoal',false, ...
        'executionSuccess',false,'routeCollisionFree',false, ...
        'qTableMBPerLearner',NaN, ...
        'maskOccupiedFraction',maskStats.occupiedFraction, ...
        'arrivalTime_s',NaN,'duration_s',NaN,'waitTime_s',NaN, ...
        'turnCountBeforeSmoothing',NaN,'turnCountAfterSmoothing',NaN, ...
        'finalGoalDistance_deg',NaN,'collisionFree',false, ...
        'trajectoryGenerated',false,'trajectoryCollisionFree',false, ...
        'trajectoryKinematicallyFeasible',false, ...
        'trajectoryC2Continuous',false, ...
        'trajectoryAttempts',0,'trajectoryAuditSuccessful',false, ...
        'trajectoryMessage',string(message), ...
        'lastFeasible_time_s',NaN, ...
        'lastFeasible_azEl_deg',[NaN,NaN]);
end


function printDiagnostic(result)
    if ~result.options.verbose
        return
    end
    fprintf('Q-learning AZ/EL planner diagnostic\n');
    fprintf('  Status: %s\n',result.status);
    fprintf('  %s\n',result.message);
    if isempty(result.path.time_s)
        return
    end
    fprintf('  Parallel requested: %d, used: %d; learners: %d\n', ...
        result.diagnostic.requestedParallel, ...
        result.diagnostic.usedParallel,result.diagnostic.numLearners);
    if strlength(result.diagnostic.parallelFallbackReason) > 0
        fprintf('  Parallel fallback: %s\n', ...
            result.diagnostic.parallelFallbackReason);
    end
    fprintf('  Training episodes: %d of %d requested (%d reached the goal, %.1f%%)\n', ...
        result.diagnostic.trainingEpisodes, ...
        result.diagnostic.requestedTrainingEpisodes, ...
        result.diagnostic.successfulTrainingEpisodes, ...
        100*result.diagnostic.trainingSuccessRate);
    fprintf('  Training stop reason(s): %s\n',strjoin( ...
        unique(result.diagnostic.trainingStopReasons,'stable'),', '));
    fprintf('  Selected policy source: %s (learner %d)\n', ...
        result.diagnostic.selectedPolicySource, ...
        result.diagnostic.selectedLearner);
    fprintf('  Last feasible: t = %.6g s at az/el = [%.6g %.6g] deg\n', ...
        result.path.time_s(end),result.path.az_deg(end),result.path.el_deg(end));
    if result.success
        fprintf('  Travel: %.6g s, including %.6g s waiting\n', ...
            result.diagnostic.duration_s,result.diagnostic.waitTime_s);
    end
    fprintf('  Path turns: %d before smoothing, %d after\n', ...
        result.diagnostic.turnCountBeforeSmoothing, ...
        result.diagnostic.turnCountAfterSmoothing);
    if result.options.generateSmoothTrajectory
        fprintf('  Smooth command: generated %d, C2 %d, safe %d, feasible %d\n', ...
            result.diagnostic.trajectoryGenerated, ...
            result.diagnostic.trajectoryC2Continuous, ...
            result.diagnostic.trajectoryCollisionFree, ...
            result.diagnostic.trajectoryKinematicallyFeasible);
        if ~result.diagnostic.trajectoryGenerated
            fprintf('  Smooth-command diagnostic: %s\n', ...
                result.diagnostic.trajectoryMessage);
        end
    end
    fprintf('  Planning and training time: %.6g s\n', ...
        result.diagnostic.planningTime_s);
end
