function audit = auditAzElTrajectory(scenario,trajectory,checkGoalHold)
%AUDITAZELTRAJECTORY Certify a piecewise-quintic az/el command trajectory.
%   AUDIT = AUDITAZELTRAJECTORY(SCENARIO,TRAJECTORY) independently checks
%   the analytic Bezier segments in TRAJECTORY. Curves are certified against
%   static polygons and linearly moving dynamic polygons by adaptive,
%   conservative subdivision. Component rate, acceleration, jerk, mechanical
%   limits, and C2 continuity are checked from the Bezier control points.

    if nargin < 3
        checkGoalHold = true;
    end

    audit = makeAudit;
    if ~isfield(scenario,'data') || ~isfield(scenario,'options')
        audit = failAudit(audit,"scenario is missing data or options", ...
            NaN,[NaN,NaN],"input");
        return
    end
    if ~isstruct(trajectory) || ~isfield(trajectory,'segments') || ...
            isempty(trajectory.segments)
        audit = failAudit(audit,"trajectory contains no analytic segments", ...
            NaN,[NaN,NaN],"input");
        return
    end
    if isfield(trajectory,'success') && ~trajectory.success
        audit = failAudit(audit,"trajectory generator reported infeasible", ...
            NaN,[NaN,NaN],"trajectory");
        return
    end

    options = scenario.options;
    data = scenario.data;
    dataTime_s = data.time_s(:);
    if numel(dataTime_s) < 2 || any(~isfinite(dataTime_s)) || ...
            any(diff(dataTime_s) <= 0)
        audit = failAudit(audit,"obstacle data times are invalid", ...
            NaN,[NaN,NaN],"input");
        return
    end
    audit.requiredClearance_deg = optionMaximum( ...
        options,'clearance_deg',0);
    periodicAzimuth = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    staticPolygons = optionCell(options,'staticPolygons');
    paddingSteps = temporalPaddingSteps(options,dataTime_s);

    [audit,valid] = certifyKinematics( ...
        audit,trajectory.segments,options);
    if ~valid
        audit = finishAudit(audit);
        return
    end

    firstTime_s = trajectory.segments(1).startTime_s;
    finalTime_s = trajectory.segments(end).endTime_s;
    tolerance_s = timeTolerance(dataTime_s);
    if firstTime_s < dataTime_s(1)-tolerance_s || ...
            finalTime_s > dataTime_s(end)+tolerance_s
        audit = failAudit(audit, ...
            "trajectory lies outside the obstacle-data time range", ...
            NaN,[firstTime_s,finalTime_s],"input");
        audit = finishAudit(audit);
        return
    end

    for segmentIndex = 1:numel(trajectory.segments)
        segment = trajectory.segments(segmentIndex);
        controlPoints_deg = segment.controlPoints_deg;
        for polygonIndex = 1:numel(staticPolygons)
            polygon = preparePolygon(staticPolygons{polygonIndex});
            if isempty(polygon)
                continue
            end
            [safe,observedDistance,certifiedDistance] = ...
                certifyPolygonMotionAcrossCopies( ...
                controlPoints_deg,polygon,polygon, ...
                audit.requiredClearance_deg,periodicAzimuth);
            audit = updateClearance( ...
                audit,observedDistance,certifiedDistance);
            if ~safe
                audit = failAudit(audit, ...
                    "quintic trajectory violates a static obstacle", ...
                    segmentIndex,[segment.startTime_s,segment.endTime_s], ...
                    "staticCurve");
                audit = finishAudit(audit);
                return
            end
        end

        splitTimes_s = unique([segment.startTime_s; ...
            dataTime_s(dataTime_s > segment.startTime_s+tolerance_s & ...
            dataTime_s < segment.endTime_s-tolerance_s); ...
            segment.endTime_s]);
        for intervalIndex = 1:numel(splitTimes_s)-1
            interval_s = splitTimes_s(intervalIndex:intervalIndex+1)';
            if interval_s(2)-interval_s(1) <= tolerance_s
                continue
            end
            localStart = (interval_s(1)-segment.startTime_s)/ ...
                (segment.endTime_s-segment.startTime_s);
            localEnd = (interval_s(2)-segment.startTime_s)/ ...
                (segment.endTime_s-segment.startTime_s);
            subcurve = restrictBezier(controlPoints_deg,localStart,localEnd);
            [frameIndex,firstFraction,lastFraction] = frameBracket( ...
                dataTime_s,interval_s);
            [safe,observedDistance,certifiedDistance] = ...
                certifyDataInterval(subcurve,data,frameIndex, ...
                firstFraction,lastFraction,audit.requiredClearance_deg, ...
                periodicAzimuth,paddingSteps);
            audit.checkedPathSegments = audit.checkedPathSegments+1;
            audit = updateClearance( ...
                audit,observedDistance,certifiedDistance);
            if ~safe
                audit = failAudit(audit, ...
                    "quintic trajectory intersects a swept dynamic obstacle", ...
                    segmentIndex,interval_s,"dynamicCurve");
                audit = finishAudit(audit);
                return
            end
        end
    end

    if checkGoalHold && optionMaximum(options,'goalHold_s',0) > 0
        [audit,valid] = certifyGoalHold( ...
            audit,data,trajectory,options,periodicAzimuth);
        if ~valid
            audit = finishAudit(audit);
            return
        end
    end
    audit.message = "safe analytic trajectory";
    audit = finishAudit(audit);
end


function audit = makeAudit
    audit = struct( ...
        'collisionFree',true, ...
        'holdSafe',true, ...
        'kinematicallyFeasible',true, ...
        'mechanicalBoundsSafe',true, ...
        'c2Continuous',true, ...
        'requiredClearance_deg',0, ...
        'minimumClearance_deg',Inf, ...
        'observedMinimumClearance_deg',Inf, ...
        'certifiedMinimumClearance_deg',Inf, ...
        'clearanceMargin_deg',Inf, ...
        'checkedPathSegments',0, ...
        'checkedHoldSegments',0, ...
        'offendingPathSegment',NaN, ...
        'offendingInterval_s',[NaN,NaN], ...
        'offendingObstacleType',"", ...
        'maxAzRate_deg_s',0, ...
        'maxElRate_deg_s',0, ...
        'maxAzAcceleration_deg_s2',0, ...
        'maxElAcceleration_deg_s2',0, ...
        'maxAzJerk_deg_s3',0, ...
        'maxElJerk_deg_s3',0, ...
        'message',"safe analytic trajectory", ...
        'source',"trajectory");
end


function [audit,valid] = certifyKinematics(audit,segments,options)
    valid = true;
    tolerance = 1e-8;
    periodicAzimuth = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    previousEnd = [];
    previousVelocity = [];
    previousAcceleration = [];
    previousTime = NaN;
    firstVelocity = [];
    firstAcceleration = [];

    for segmentIndex = 1:numel(segments)
        segment = segments(segmentIndex);
        if ~isfield(segment,'startTime_s') || ...
                ~isfield(segment,'endTime_s') || ...
                ~isfield(segment,'controlPoints_deg')
            audit = failAudit(audit,"analytic segment fields are missing", ...
                segmentIndex,[NaN,NaN],"input");
            valid = false;
            return
        end
        duration_s = segment.endTime_s-segment.startTime_s;
        controlPoints_deg = segment.controlPoints_deg;
        if ~isscalar(duration_s) || ~isfinite(duration_s) || ...
                duration_s <= 0 || ~isequal(size(controlPoints_deg),[6,2]) || ...
                any(~isfinite(controlPoints_deg(:)))
            audit = failAudit(audit,"analytic segment is malformed", ...
                segmentIndex,[segment.startTime_s,segment.endTime_s], ...
                "input");
            valid = false;
            return
        end
        velocity = 5*diff(controlPoints_deg,1,1)/duration_s;
        acceleration = 4*diff(velocity,1,1)/duration_s;
        jerk = 3*diff(acceleration,1,1)/duration_s;
        if segmentIndex == 1
            firstVelocity = velocity(1,:);
            firstAcceleration = acceleration(1,:);
        end
        audit.maxAzRate_deg_s = max( ...
            audit.maxAzRate_deg_s,max(abs(velocity(:,1))));
        audit.maxElRate_deg_s = max( ...
            audit.maxElRate_deg_s,max(abs(velocity(:,2))));
        audit.maxAzAcceleration_deg_s2 = max( ...
            audit.maxAzAcceleration_deg_s2,max(abs(acceleration(:,1))));
        audit.maxElAcceleration_deg_s2 = max( ...
            audit.maxElAcceleration_deg_s2,max(abs(acceleration(:,2))));
        audit.maxAzJerk_deg_s3 = max( ...
            audit.maxAzJerk_deg_s3,max(abs(jerk(:,1))));
        audit.maxElJerk_deg_s3 = max( ...
            audit.maxElJerk_deg_s3,max(abs(jerk(:,2))));

        if ~isempty(previousEnd)
            scale = max(1,max(abs([previousEnd;controlPoints_deg(1,:)]), ...
                [],'all'));
            continuous = abs(segment.startTime_s-previousTime) <= ...
                tolerance*max(1,abs(previousTime)) && ...
                norm(controlPoints_deg(1,:)-previousEnd,Inf) <= ...
                tolerance*scale && ...
                norm(velocity(1,:)-previousVelocity,Inf) <= ...
                tolerance*max(1,norm(previousVelocity,Inf)) && ...
                norm(acceleration(1,:)-previousAcceleration,Inf) <= ...
                tolerance*max(1,norm(previousAcceleration,Inf));
            if ~continuous
                audit.c2Continuous = false;
                audit.kinematicallyFeasible = false;
                audit = failAudit(audit, ...
                    "trajectory segments are not C2-continuous", ...
                    segmentIndex-1,[previousTime,segment.startTime_s], ...
                    "continuity");
                valid = false;
                return
            end
        end
        previousEnd = controlPoints_deg(end,:);
        previousVelocity = velocity(end,:);
        previousAcceleration = acceleration(end,:);
        previousTime = segment.endTime_s;

        if ~periodicAzimuth && isfield(options,'azLim_deg') && ...
                (min(controlPoints_deg(:,1)) < options.azLim_deg(1)-tolerance || ...
                max(controlPoints_deg(:,1)) > options.azLim_deg(2)+tolerance)
            audit.mechanicalBoundsSafe = false;
        end
        if isfield(options,'elLim_deg') && ...
                (min(controlPoints_deg(:,2)) < options.elLim_deg(1)-tolerance || ...
                max(controlPoints_deg(:,2)) > options.elLim_deg(2)+tolerance)
            audit.mechanicalBoundsSafe = false;
        end
    end

    requireEndpointRest = false;
    if isfield(options,'trajectoryRequireEndpointRest') && ...
            ~isempty(options.trajectoryRequireEndpointRest)
        requireEndpointRest = logical(options.trajectoryRequireEndpointRest);
    end
    if requireEndpointRest
        endpointTolerance = 1e-8;
        endpointsAtRest = norm(firstVelocity,Inf) <= endpointTolerance && ...
            norm(firstAcceleration,Inf) <= endpointTolerance && ...
            norm(previousVelocity,Inf) <= endpointTolerance && ...
            norm(previousAcceleration,Inf) <= endpointTolerance;
        if ~endpointsAtRest
            audit.kinematicallyFeasible = false;
            audit = failAudit(audit, ...
                "trajectory endpoint rest condition is not satisfied", ...
                NaN,[segments(1).startTime_s,segments(end).endTime_s], ...
                "endpointKinematics");
            valid = false;
            return
        end
    end

    withinLimits = ...
        audit.maxAzRate_deg_s <= optionMaximum( ...
        options,'azRate_deg_s',Inf)+tolerance && ...
        audit.maxElRate_deg_s <= optionMaximum( ...
        options,'elRate_deg_s',Inf)+tolerance && ...
        audit.maxAzAcceleration_deg_s2 <= optionMaximum( ...
        options,'maxAzAcceleration_deg_s2',Inf)+tolerance && ...
        audit.maxElAcceleration_deg_s2 <= optionMaximum( ...
        options,'maxElAcceleration_deg_s2',Inf)+tolerance && ...
        audit.maxAzJerk_deg_s3 <= optionMaximum( ...
        options,'maxAzJerk_deg_s3',Inf)+tolerance && ...
        audit.maxElJerk_deg_s3 <= optionMaximum( ...
        options,'maxElJerk_deg_s3',Inf)+tolerance;
    audit.kinematicallyFeasible = withinLimits && ...
        audit.mechanicalBoundsSafe && audit.c2Continuous;
    if ~audit.mechanicalBoundsSafe
        audit = failAudit(audit,"trajectory leaves the mechanical limits", ...
            NaN,[segments(1).startTime_s,segments(end).endTime_s], ...
            "mechanicalBounds");
        valid = false;
    elseif ~withinLimits
        audit = failAudit(audit,"trajectory exceeds a kinematic limit", ...
            NaN,[segments(1).startTime_s,segments(end).endTime_s], ...
            "kinematics");
        valid = false;
    end
end


function [safe,observedDistance,certifiedDistance] = ...
        certifyDataInterval(curve,data,frameIndex,firstFraction, ...
        lastFraction,clearance_deg,isPeriodic,paddingSteps)
    safe = true;
    observedDistance = Inf;
    certifiedDistance = Inf;
    if paddingSteps <= 0
        firstPolygons = getFramePolygons(data,frameIndex);
        secondPolygons = getFramePolygons(data,frameIndex+1);
        [safe,observedDistance,certifiedDistance] = ...
            certifyDynamicPolygonSets(curve,firstPolygons,secondPolygons, ...
            firstFraction,lastFraction,clearance_deg,isPeriodic,false);
        return
    end

    % Temporal padding means that every command point must also be safe
    % against the corresponding phase of each neighboring frame interval.
    % This is the continuous-time counterpart of the frame unions used by
    % buildAzElOccupancy.
    numIntervals = numel(data.time_s)-1;
    firstSource = max(1,frameIndex-paddingSteps);
    lastSource = min(numIntervals,frameIndex+paddingSteps);
    for sourceIndex = firstSource:lastSource
        firstPolygons = getFramePolygons(data,sourceIndex);
        secondPolygons = getFramePolygons(data,sourceIndex+1);
        [localSafe,localObserved,localCertified] = ...
            certifyDynamicPolygonSets(curve,firstPolygons,secondPolygons, ...
            firstFraction,lastFraction,clearance_deg,isPeriodic,false);
        observedDistance = min(observedDistance,localObserved);
        certifiedDistance = min(certifiedDistance,localCertified);
        if ~localSafe
            safe = false;
            return
        end
    end
end


function [safe,observedDistance,certifiedDistance] = ...
        certifyDynamicPolygonSets(curve,firstPolygons,secondPolygons, ...
        firstFraction,lastFraction,clearance_deg,isPeriodic, ...
        useSweptEnvelope)
    safe = true;
    observedDistance = Inf;
    certifiedDistance = Inf;
    firstPolygons = preparePolygonSet(firstPolygons);
    secondPolygons = preparePolygonSet(secondPolygons);
    if isempty(firstPolygons) && isempty(secondPolygons)
        return
    end

    [pairs,unmatchedFirst,unmatchedSecond] = matchPolygonSets( ...
        firstPolygons,secondPolygons,isPeriodic);
    for pairIndex = 1:size(pairs,1)
        firstPolygon = firstPolygons{pairs(pairIndex,1)};
        secondPolygon = secondPolygons{pairs(pairIndex,2)};
        if useSweptEnvelope || ...
                size(firstPolygon,1) ~= size(secondPolygon,1)
            envelope = makePolygonEnvelope( ...
                {firstPolygon,secondPolygon},isPeriodic);
            [polygonSafe,polygonObserved,polygonCertified] = ...
                certifyPolygonMotionAcrossCopies( ...
                curve,envelope,envelope,clearance_deg,isPeriodic);
        else
            [firstPolygon,secondPolygon] = alignPolygonPairLocally( ...
                firstPolygon,secondPolygon,isPeriodic);
            intervalFirst = firstPolygon+firstFraction* ...
                (secondPolygon-firstPolygon);
            intervalSecond = firstPolygon+lastFraction* ...
                (secondPolygon-firstPolygon);
            [polygonSafe,polygonObserved,polygonCertified] = ...
                certifyPolygonMotionAcrossCopies(curve,intervalFirst, ...
                intervalSecond,clearance_deg,isPeriodic);
        end
        observedDistance = min(observedDistance,polygonObserved);
        certifiedDistance = min(certifiedDistance,polygonCertified);
        if ~polygonSafe
            safe = false;
            return
        end
    end

    if ~isempty(unmatchedFirst) && ~isempty(unmatchedSecond)
        % Ambiguous disappearance/appearance cannot be assigned a motion
        % correspondence safely. Certify their joint swept envelope instead
        % of guessing a pairing that could miss an intermediate collision.
        envelope = makePolygonEnvelope( ...
            [firstPolygons(unmatchedFirst), ...
            secondPolygons(unmatchedSecond)],isPeriodic);
        [polygonSafe,polygonObserved,polygonCertified] = ...
            certifyPolygonMotionAcrossCopies( ...
            curve,envelope,envelope,clearance_deg,isPeriodic);
        observedDistance = min(observedDistance,polygonObserved);
        certifiedDistance = min(certifiedDistance,polygonCertified);
        if ~polygonSafe
            safe = false;
        end
        return
    end

    unmatched = [firstPolygons(unmatchedFirst), ...
        secondPolygons(unmatchedSecond)];
    for polygonIndex = 1:numel(unmatched)
        polygon = unmatched{polygonIndex};
        [polygonSafe,polygonObserved,polygonCertified] = ...
            certifyPolygonMotionAcrossCopies( ...
            curve,polygon,polygon,clearance_deg,isPeriodic);
        observedDistance = min(observedDistance,polygonObserved);
        certifiedDistance = min(certifiedDistance,polygonCertified);
        if ~polygonSafe
            safe = false;
            return
        end
    end
end


function polygons = preparePolygonSet(polygons)
    prepared = cell(size(polygons));
    keep = false(size(polygons));
    for polygonIndex = 1:numel(polygons)
        prepared{polygonIndex} = preparePolygon(polygons{polygonIndex});
        keep(polygonIndex) = ~isempty(prepared{polygonIndex});
    end
    polygons = prepared(keep);
end


function [pairs,unmatchedFirst,unmatchedSecond] = ...
        matchPolygonSets(firstPolygons,secondPolygons,isPeriodic)
    numFirst = numel(firstPolygons);
    numSecond = numel(secondPolygons);
    if numFirst == numSecond
        pairs = [(1:numFirst)',(1:numSecond)'];
        unmatchedFirst = zeros(1,0);
        unmatchedSecond = zeros(1,0);
        return
    end
    if numFirst == 0
        pairs = zeros(0,2);
        unmatchedFirst = zeros(1,0);
        unmatchedSecond = 1:numSecond;
        return
    elseif numSecond == 0
        pairs = zeros(0,2);
        unmatchedFirst = 1:numFirst;
        unmatchedSecond = zeros(1,0);
        return
    end

    firstCentroid = polygonCentroids(firstPolygons);
    secondCentroid = polygonCentroids(secondPolygons);
    azimuthDifference = firstCentroid(:,1)-secondCentroid(:,1)';
    if isPeriodic
        azimuthDifference = mod(azimuthDifference+180,360)-180;
    end
    elevationDifference = firstCentroid(:,2)-secondCentroid(:,2)';
    distance = hypot(azimuthDifference,elevationDifference);
    [firstNearestDistance,firstNearest] = min(distance,[],2);
    [secondNearestDistance,secondNearest] = min(distance,[],1);
    tolerance = 1e-10*max(1,max(distance(:)));
    pairs = zeros(0,2);
    usedFirst = false(1,numFirst);
    usedSecond = false(1,numSecond);
    for firstIndex = 1:numFirst
        secondIndex = firstNearest(firstIndex);
        if secondNearest(secondIndex) ~= firstIndex
            continue
        end
        firstTies = nnz(abs(distance(firstIndex,:)- ...
            firstNearestDistance(firstIndex)) <= tolerance);
        secondTies = nnz(abs(distance(:,secondIndex)- ...
            secondNearestDistance(secondIndex)) <= tolerance);
        if firstTies ~= 1 || secondTies ~= 1
            continue
        end
        pairs(end+1,:) = [firstIndex,secondIndex]; %#ok<AGROW>
        usedFirst(firstIndex) = true;
        usedSecond(secondIndex) = true;
    end
    unmatchedFirst = find(~usedFirst);
    unmatchedSecond = find(~usedSecond);
end


function centroids = polygonCentroids(polygons)
    centroids = zeros(numel(polygons),2);
    for polygonIndex = 1:numel(polygons)
        centroids(polygonIndex,:) = mean(polygons{polygonIndex},1);
    end
end


function [safe,observedDistance,certifiedDistance] = ...
        certifyPolygonMotionAcrossCopies(curve,firstPolygon,secondPolygon, ...
        clearance_deg,isPeriodic)
    safe = true;
    observedDistance = Inf;
    certifiedDistance = Inf;
    [firstPolygon,secondPolygon] = alignPolygonPairLocally( ...
        firstPolygon,secondPolygon,isPeriodic);
    shifts_deg = periodicCopyShifts( ...
        curve,firstPolygon,secondPolygon,clearance_deg,isPeriodic);
    for shift_deg = shifts_deg
        shiftedFirst = firstPolygon;
        shiftedSecond = secondPolygon;
        shiftedFirst(:,1) = shiftedFirst(:,1)+shift_deg;
        shiftedSecond(:,1) = shiftedSecond(:,1)+shift_deg;
        [copySafe,copyObserved,copyCertified] = ...
            certifyBezierAgainstMotion(curve,shiftedFirst, ...
            shiftedSecond,clearance_deg,0);
        observedDistance = min(observedDistance,copyObserved);
        certifiedDistance = min(certifiedDistance,copyCertified);
        if ~copySafe
            safe = false;
            return
        end
    end
end


function shifts_deg = periodicCopyShifts( ...
        curve,firstPolygon,secondPolygon,clearance_deg,isPeriodic)
    if ~isPeriodic
        shifts_deg = 0;
        return
    end
    polygonAzimuth_deg = [firstPolygon(:,1);secondPolygon(:,1)];
    curveMinimum_deg = min(curve(:,1))-clearance_deg;
    curveMaximum_deg = max(curve(:,1))+clearance_deg;
    firstWrap = floor((curveMinimum_deg-max(polygonAzimuth_deg))/360);
    lastWrap = ceil((curveMaximum_deg-min(polygonAzimuth_deg))/360);
    shifts_deg = 360*(firstWrap:lastWrap);
end


function [safe,observedDistance,certifiedDistance] = ...
        certifyBezierAgainstMotion(curve,firstPolygon,secondPolygon, ...
        clearance_deg,depth)
    maxDepth = 24;
    tolerance = 1e-10;
    midpointCurve = splitBezier(curve,0.5);
    point = midpointCurve{1}(end,:);
    midpointPolygon = 0.5*(firstPolygon+secondPolygon);
    observedDistance = pointPolygonDistance(point,midpointPolygon);
    curveVelocity = 5*diff(curve,1,1);
    curveMotionBound = max(hypot( ...
        curveVelocity(:,1),curveVelocity(:,2)));
    polygonMotion = secondPolygon-firstPolygon;
    if isempty(polygonMotion)
        polygonMotionBound = 0;
    else
        polygonMotionBound = max(hypot( ...
            polygonMotion(:,1),polygonMotion(:,2)));
    end
    certifiedDistance = max(0,observedDistance- ...
        0.5*(curveMotionBound+polygonMotionBound));

    if observedDistance <= clearance_deg+tolerance
        safe = false;
        return
    end
    if certifiedDistance > clearance_deg+tolerance
        safe = true;
        return
    end
    if depth >= maxDepth
        safe = false;
        return
    end

    halves = splitBezier(curve,0.5);
    midpointPolygon = 0.5*(firstPolygon+secondPolygon);
    [firstSafe,firstObserved,firstCertified] = ...
        certifyBezierAgainstMotion(halves{1},firstPolygon, ...
        midpointPolygon,clearance_deg,depth+1);
    observedDistance = min(observedDistance,firstObserved);
    certifiedDistance = firstCertified;
    if ~firstSafe
        safe = false;
        return
    end
    [secondSafe,secondObserved,secondCertified] = ...
        certifyBezierAgainstMotion(halves{2},midpointPolygon, ...
        secondPolygon,clearance_deg,depth+1);
    safe = secondSafe;
    observedDistance = min(observedDistance,secondObserved);
    certifiedDistance = min(firstCertified,secondCertified);
end


function [audit,valid] = certifyGoalHold( ...
        audit,data,trajectory,options,isPeriodic)
    valid = true;
    finalSegment = trajectory.segments(end);
    finalDuration_s = finalSegment.endTime_s-finalSegment.startTime_s;
    finalVelocity = 5*diff( ...
        finalSegment.controlPoints_deg,1,1)/finalDuration_s;
    finalAcceleration = 4*diff(finalVelocity,1,1)/finalDuration_s;
    endpointTolerance = 1e-8;
    if norm(finalVelocity(end,:),Inf) > endpointTolerance || ...
            norm(finalAcceleration(end,:),Inf) > endpointTolerance
        audit.holdSafe = false;
        audit.kinematicallyFeasible = false;
        audit = failAudit(audit, ...
            "trajectory does not come to rest before the goal hold", ...
            numel(trajectory.segments), ...
            finalSegment.endTime_s*[1,1],"goalHoldKinematics");
        valid = false;
        return
    end
    holdDuration_s = optionMaximum(options,'goalHold_s',0);
    startTime_s = trajectory.segments(end).endTime_s;
    endTime_s = startTime_s+holdDuration_s;
    dataTime_s = data.time_s(:);
    tolerance_s = timeTolerance(dataTime_s);
    paddingSteps = temporalPaddingSteps(options,dataTime_s);
    if endTime_s > dataTime_s(end)+tolerance_s
        audit.holdSafe = false;
        audit = failAudit(audit,"goal hold extends beyond obstacle data", ...
            numel(trajectory.segments),[startTime_s,endTime_s],"goalHold");
        valid = false;
        return
    end
    point = trajectory.segments(end).controlPoints_deg(end,:);
    boundaries_s = unique([startTime_s; ...
        dataTime_s(dataTime_s > startTime_s+tolerance_s & ...
        dataTime_s < endTime_s-tolerance_s);endTime_s]);
    curve = repmat(point,6,1);
    for intervalIndex = 1:numel(boundaries_s)-1
        interval_s = boundaries_s(intervalIndex:intervalIndex+1)';
        [frameIndex,firstFraction,lastFraction] = frameBracket( ...
            dataTime_s,interval_s);
        [safe,observedDistance,certifiedDistance] = ...
            certifyDataInterval(curve,data,frameIndex,firstFraction, ...
            lastFraction,audit.requiredClearance_deg,isPeriodic,paddingSteps);
        audit.checkedHoldSegments = audit.checkedHoldSegments+1;
        audit = updateClearance(audit,observedDistance,certifiedDistance);
        if ~safe
            audit.holdSafe = false;
            audit = failAudit(audit, ...
                "dynamic obstacle crosses the goal during the hold interval", ...
                numel(trajectory.segments),interval_s,"goalHold");
            valid = false;
            return
        end
    end
end


function restricted = restrictBezier(controlPoints,firstFraction,lastFraction)
    firstFraction = min(1,max(0,firstFraction));
    lastFraction = min(1,max(firstFraction,lastFraction));
    if firstFraction <= eps && lastFraction >= 1-eps
        restricted = controlPoints;
        return
    end
    if lastFraction < 1
        parts = splitBezier(controlPoints,lastFraction);
        restricted = parts{1};
    else
        restricted = controlPoints;
    end
    if firstFraction > 0
        localFraction = firstFraction/max(lastFraction,eps);
        parts = splitBezier(restricted,localFraction);
        restricted = parts{2};
    end
end


function parts = splitBezier(controlPoints,fraction)
    degree = size(controlPoints,1)-1;
    triangle = zeros(degree+1,degree+1,size(controlPoints,2));
    triangle(:,1,:) = controlPoints;
    for level = 2:degree+1
        count = degree-level+2;
        triangle(1:count,level,:) = (1-fraction)* ...
            triangle(1:count,level-1,:)+fraction* ...
            triangle(2:count+1,level-1,:);
    end
    left = zeros(size(controlPoints));
    right = zeros(size(controlPoints));
    for level = 1:degree+1
        left(level,:) = reshape(triangle(1,level,:),1,[]);
        right(degree-level+2,:) = reshape( ...
            triangle(degree-level+2,level,:),1,[]);
    end
    parts = {left,right};
end


function [frameIndex,firstFraction,lastFraction] = ...
        frameBracket(dataTime_s,interval_s)
    midpoint_s = mean(interval_s);
    frameIndex = find(dataTime_s <= midpoint_s,1,'last');
    frameIndex = min(max(1,frameIndex),numel(dataTime_s)-1);
    duration_s = dataTime_s(frameIndex+1)-dataTime_s(frameIndex);
    firstFraction = (interval_s(1)-dataTime_s(frameIndex))/duration_s;
    lastFraction = (interval_s(2)-dataTime_s(frameIndex))/duration_s;
    firstFraction = min(1,max(0,firstFraction));
    lastFraction = min(1,max(firstFraction,lastFraction));
end


function polygons = getFramePolygons(data,frameIndex)
    if iscell(data.az_deg)
        az_deg = data.az_deg{frameIndex};
        el_deg = data.el_deg{frameIndex};
    else
        az_deg = data.az_deg(frameIndex,:);
        el_deg = data.el_deg(frameIndex,:);
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


function polygon = preparePolygon(polygon)
    if isempty(polygon)
        polygon = zeros(0,2);
        return
    end
    polygon = polygon(all(isfinite(polygon),2),:);
    if isempty(polygon)
        return
    end
    polygon(:,1) = rad2deg(unwrap(deg2rad(polygon(:,1))));
    if size(polygon,1) > 1 && ...
            norm(polygon(1,:)-polygon(end,:)) <= 1e-10
        polygon = polygon(1:end-1,:);
    end
end


function [firstPolygon,secondPolygon] = alignPolygonPairLocally( ...
        firstPolygon,secondPolygon,isPeriodic)
    if ~isPeriodic
        return
    end
    secondPolygon(:,1) = secondPolygon(:,1)+360*round( ...
        (mean(firstPolygon(:,1))-mean(secondPolygon(:,1)))/360);
end


function envelope = makePolygonEnvelope(polygons,isPeriodic)
    vertices = zeros(0,2);
    referenceAzimuth_deg = NaN;
    for polygonIndex = 1:numel(polygons)
        polygon = preparePolygon(polygons{polygonIndex});
        if isempty(polygon)
            continue
        end
        if isPeriodic
            if isnan(referenceAzimuth_deg)
                referenceAzimuth_deg = mean(polygon(:,1));
            else
                polygon(:,1) = polygon(:,1)+360*round( ...
                    (referenceAzimuth_deg-mean(polygon(:,1)))/360);
            end
        end
        vertices = [vertices;polygon]; %#ok<AGROW>
    end
    vertices = unique(vertices,'rows','stable');
    if size(vertices,1) >= 3 && ...
            rank(vertices(2:end,:)-vertices(1,:)) >= 2
        hull = convhull(vertices(:,1),vertices(:,2));
        envelope = vertices(hull(1:end-1),:);
    else
        envelope = vertices;
    end
end


function distance = pointPolygonDistance(point,polygon)
    if isempty(polygon)
        distance = Inf;
        return
    end
    if size(polygon,1) >= 3
        [inside,onBoundary] = inpolygon( ...
            point(1),point(2),polygon(:,1),polygon(:,2));
        if inside || onBoundary
            distance = 0;
            return
        end
    end
    if size(polygon,1) == 1
        distance = norm(point-polygon(1,:));
        return
    end
    edgeStarts = polygon;
    if size(polygon,1) == 2
        edgeEnds = polygon([2,1],:);
    else
        edgeEnds = polygon([2:end,1],:);
    end
    edge = edgeEnds-edgeStarts;
    lengthSquared = sum(edge.^2,2);
    offset = point-edgeStarts;
    fraction = sum(offset.*edge,2)./max(lengthSquared,eps);
    fraction = min(1,max(0,fraction));
    closest = edgeStarts+fraction.*edge;
    distances = hypot(point(1)-closest(:,1),point(2)-closest(:,2));
    distance = min(distances);
end


function value = optionMaximum(options,name,defaultValue)
    if isfield(options,name) && ~isempty(options.(name))
        optionValue = options.(name);
        value = max(optionValue(:));
    else
        value = defaultValue;
    end
end


function value = optionCell(options,name)
    if isfield(options,name) && ~isempty(options.(name))
        value = options.(name);
    else
        value = {};
    end
end


function paddingSteps = temporalPaddingSteps(options,dataTime_s)
    padding_s = optionMaximum(options,'temporalPadding_s',0);
    if padding_s <= 0
        paddingSteps = 0;
        return
    end
    representativeStep_s = median(diff(dataTime_s));
    paddingSteps = max(0,ceil(padding_s/representativeStep_s-1e-12));
end


function tolerance_s = timeTolerance(time_s)
    tolerance_s = 1e-10*max(1,max(abs(time_s)));
end


function audit = updateClearance(audit,observed,certified)
    audit.observedMinimumClearance_deg = min( ...
        audit.observedMinimumClearance_deg,observed);
    audit.minimumClearance_deg = min(audit.minimumClearance_deg,certified);
    audit.certifiedMinimumClearance_deg = audit.minimumClearance_deg;
end


function audit = failAudit(audit,message,pathSegment,interval_s,obstacleType)
    audit.collisionFree = false;
    audit.offendingPathSegment = pathSegment;
    audit.offendingInterval_s = interval_s;
    audit.offendingObstacleType = obstacleType;
    audit.message = message;
end


function audit = finishAudit(audit)
    audit.clearanceMargin_deg = ...
        audit.minimumClearance_deg-audit.requiredClearance_deg;
end
