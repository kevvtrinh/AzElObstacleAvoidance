function [trajectory,info] = smoothAzElTrajectory(path,data,options)
%SMOOTHAZELTRAJECTORY Build a collision-checked C2 azimuth/elevation command.
%   [TRAJECTORY,INFO] = SMOOTHAZELTRAJECTORY(PATH,DATA,OPTIONS) converts
%   the timed, piecewise-linear planner path into straight, wait, and
%   quintic Bezier segments.  The blends retain the planner's absolute
%   timing.  A blend may use the ends of a wait interval, but a stationary
%   core remains because adjacent blends each consume less than half of it.
%   Every such timing change is re-certified against the moving obstacles.

    requiredPathFields = {'time_s','az_deg','el_deg'};
    for fieldIndex = 1:numel(requiredPathFields)
        if ~isfield(path,requiredPathFields{fieldIndex})
            error('smoothAzElTrajectory:InvalidPath', ...
                'PATH must contain %s.',requiredPathFields{fieldIndex});
        end
    end
    if ~isfield(options,'trajectoryRequireEndpointRest') || ...
            isempty(options.trajectoryRequireEndpointRest)
        options.trajectoryRequireEndpointRest = false;
    end
    validateTrajectoryOptions(options);

    time_s = path.time_s(:);
    az_deg = path.az_deg(:);
    el_deg = path.el_deg(:);
    if numel(az_deg) ~= numel(time_s) || numel(el_deg) ~= numel(time_s) || ...
            isempty(time_s) || any(~isfinite([time_s;az_deg;el_deg])) || ...
            any(diff(time_s) <= 0)
        error('smoothAzElTrajectory:InvalidPath', ...
            'PATH samples must be finite, equally sized, and strictly timed.');
    end

    if isscalar(time_s)
        trajectory = makePointTrajectory(path,data,options);
        pointResult = struct('success',true,'path',path);
        pointAudit = auditPlannerPath( ...
            struct('data',data,'options',options),pointResult);
        pointSafe = pointAudit.collisionFree && ...
            controlPointsInsideLimits([az_deg,el_deg],options);
        trajectory.success = pointSafe;
        if pointSafe
            trajectory.message = "Collision-free stationary trajectory.";
        else
            trajectory.message = "The stationary trajectory failed its safety audit.";
        end
        info = makeInfo(trajectory,pointSafe,pointAudit);
        return
    end

    baseSegments = compressTimedPath(time_s,[az_deg,el_deg]);
    events = makeTransitionEvents(baseSegments,options);
    if options.trajectoryBlendAttempts == 1
        fractions = options.trajectoryMaxBlendFraction;
    else
        fractions = linspace(options.trajectoryMaxBlendFraction, ...
            options.trajectoryMinBlendFraction,options.trajectoryBlendAttempts);
    end

    % Each blend owns at most the configured fraction of either adjacent
    % straight segment.  Because the maximum is below one half, blends at
    % opposite ends of a short segment remain disjoint.
    lastAudit = struct;
    for eventIndex = 1:numel(events)
        accepted = false;
        fractionPairs = makeFractionPairs( ...
            events(eventIndex),fractions,baseSegments);
        for pairIndex = 1:size(fractionPairs,1)
            trialEvents = events;
            trialEvents(eventIndex) = configureEventPair( ...
                trialEvents(eventIndex),baseSegments, ...
                fractionPairs(pairIndex,1),fractionPairs(pairIndex,2));
            transition = eventToSegments( ...
                trialEvents(eventIndex),baseSegments);
            bounds = trajectoryDerivativeControlBounds(transition);
            if ~withinKinematicLimits(bounds,options) || ...
                    ~controlPointsInsideLimits( ...
                    vertcat(transition.controlPoints_deg),options)
                continue
            end

            % Audit only this local replacement. Future boundaries still
            % contain the original sharp join until their own iteration,
            % so auditing the partially assembled path here would reject
            % C2 continuity before reaching this fillet's geometry.
            trialTrajectory = struct('success',true,'segments',transition);
            localAuditOptions = options;
            % A local replacement legitimately inherits nonzero velocity at
            % an interior boundary. Endpoint rest is a property of the fully
            % assembled command, not of every isolated fillet audit.
            localAuditOptions.trajectoryRequireEndpointRest = false;
            lastAudit = auditAzElTrajectory( ...
                struct('data',data,'options',localAuditOptions), ...
                trialTrajectory,false);
            if isfield(lastAudit,'collisionFree') && lastAudit.collisionFree
                events = trialEvents;
                accepted = true;
                break
            end
        end

        if ~accepted
            message = sprintf([ ...
                'No collision-free C2 blend at t = %.6g s satisfies the ' ...
                'configured rate, acceleration, and jerk limits.'], ...
                events(eventIndex).boundaryTime_s);
            segments = assembleSegments(baseSegments,events);
            trajectory = sampleTrajectory( ...
                segments,path,data,options,false);
            trajectory.message = string(message);
            info = makeInfo(trajectory,false,lastAudit);
            return
        end
    end

    segments = assembleSegments(baseSegments,events);
    trajectory = sampleTrajectory(segments,path,data,options,true);
    extrema = trajectoryDerivativeExtrema(segments);
    if ~withinKinematicLimits(extrema,options)
        trajectory.success = false;
        trajectory.message = "The assembled trajectory exceeds a kinematic limit.";
        info = makeInfo(trajectory,false,lastAudit);
        return
    end
    if ~trajectory.c2Continuous
        trajectory.success = false;
        trajectory.message = "The assembled trajectory is not C2 continuous.";
        info = makeInfo(trajectory,false,lastAudit);
        return
    end

    lastAudit = auditAzElTrajectory( ...
        struct('data',data,'options',options),trajectory,false);
    collisionFree = isfield(lastAudit,'collisionFree') && ...
        lastAudit.collisionFree;
    trajectory.success = collisionFree;
    if collisionFree
        trajectory.message = "Collision-free C2 trajectory generated.";
    else
        trajectory.message = "The final smooth trajectory failed collision audit.";
    end
    info = makeInfo(trajectory,collisionFree,lastAudit);
end


function validateTrajectoryOptions(options)
    requiredPositive = {'trajectorySampleTime_s','azRate_deg_s', ...
        'elRate_deg_s','maxAzAcceleration_deg_s2', ...
        'maxElAcceleration_deg_s2','maxAzJerk_deg_s3', ...
        'maxElJerk_deg_s3'};
    for fieldIndex = 1:numel(requiredPositive)
        name = requiredPositive{fieldIndex};
        if ~isfield(options,name)
            error('smoothAzElTrajectory:MissingOption', ...
                'OPTIONS must contain %s.',name);
        end
        validateattributes(options.(name),{'numeric'}, ...
            {'scalar','real','finite','positive'},mfilename,['options.' name]);
    end

    fractionFields = {'trajectoryMaxBlendFraction', ...
        'trajectoryMinBlendFraction'};
    for fieldIndex = 1:numel(fractionFields)
        name = fractionFields{fieldIndex};
        if ~isfield(options,name)
            error('smoothAzElTrajectory:MissingOption', ...
                'OPTIONS must contain %s.',name);
        end
        validateattributes(options.(name),{'numeric'}, ...
            {'scalar','real','finite','positive','<',0.5}, ...
            mfilename,['options.' name]);
    end
    if options.trajectoryMinBlendFraction > ...
            options.trajectoryMaxBlendFraction
        error('smoothAzElTrajectory:InvalidBlendRange', ...
            'trajectoryMinBlendFraction cannot exceed the maximum.');
    end
    if ~isfield(options,'trajectoryBlendAttempts')
        error('smoothAzElTrajectory:MissingOption', ...
            'OPTIONS must contain trajectoryBlendAttempts.');
    end
    validateattributes(options.trajectoryBlendAttempts,{'numeric'}, ...
        {'scalar','integer','positive','finite'},mfilename, ...
        'options.trajectoryBlendAttempts');
    validateattributes(options.trajectoryRequireEndpointRest, ...
        {'logical','numeric'},{'scalar','binary'},mfilename, ...
        'options.trajectoryRequireEndpointRest');
end


function baseSegments = compressTimedPath(time_s,position_deg)
    dt_s = diff(time_s);
    velocity_deg_s = diff(position_deg,1,1)./dt_s;
    numEdges = numel(dt_s);
    template = struct('startTime_s',0,'endTime_s',0, ...
        'startPoint_deg',[0,0],'endPoint_deg',[0,0], ...
        'velocity_deg_s',[0,0],'kind',"line");
    baseSegments = repmat(template,0,1);
    firstEdge = 1;
    for edgeIndex = 2:numEdges
        if sameVelocity(velocity_deg_s(firstEdge,:), ...
                velocity_deg_s(edgeIndex,:))
            continue
        end
        baseSegments(end+1,1) = makeBaseSegment( ...
            firstEdge,edgeIndex-1,time_s,position_deg,velocity_deg_s); %#ok<AGROW>
        firstEdge = edgeIndex;
    end
    baseSegments(end+1,1) = makeBaseSegment( ...
        firstEdge,numEdges,time_s,position_deg,velocity_deg_s);
end


function same = sameVelocity(firstVelocity,secondVelocity)
    scale = max([1,norm(firstVelocity),norm(secondVelocity)]);
    same = norm(firstVelocity-secondVelocity) <= 1e-9*scale;
end


function segment = makeBaseSegment( ...
        firstEdge,lastEdge,time_s,position_deg,velocity_deg_s)
    segment = struct;
    segment.startTime_s = time_s(firstEdge);
    segment.endTime_s = time_s(lastEdge+1);
    segment.startPoint_deg = position_deg(firstEdge,:);
    segment.endPoint_deg = position_deg(lastEdge+1,:);
    segment.velocity_deg_s = velocity_deg_s(firstEdge,:);
    if norm(segment.velocity_deg_s) <= 1e-10
        segment.kind = "wait";
    else
        segment.kind = "line";
    end
end


function events = makeTransitionEvents(baseSegments,options)
    template = struct('boundaryTime_s',0,'leftBase',0,'rightBase',0, ...
        'leftExtent_s',0,'rightExtent_s',0,'fraction',0, ...
        'controlPoints_deg',zeros(6,2),'kind',"fillet",'configured',false);
    events = repmat(template,0,1);
    if options.trajectoryRequireEndpointRest && ...
            baseSegments(1).kind ~= "wait"
        event = template;
        event.boundaryTime_s = baseSegments(1).startTime_s;
        event.leftBase = 0;
        event.rightBase = 1;
        event.kind = "startRest";
        event = configureEventPair(event,baseSegments,0,0);
        events(end+1,1) = event;
    end
    for segmentIndex = 1:numel(baseSegments)-1
        left = baseSegments(segmentIndex);
        right = baseSegments(segmentIndex+1);
        if sameVelocity(left.velocity_deg_s,right.velocity_deg_s)
            continue
        end
        event = template;
        event.boundaryTime_s = left.endTime_s;
        event.leftBase = segmentIndex;
        event.rightBase = segmentIndex+1;
        if left.kind == "wait"
            event.kind = "waitDeparture";
        elseif right.kind == "wait"
            event.kind = "waitArrival";
        end
        % Initialize with zero extent.  This leaves the original collision-
        % free polyline in place while earlier transitions are assessed.
        event = configureEvent(event,baseSegments,0);
        events(end+1,1) = event; %#ok<AGROW>
    end
    requireFinalRest = options.trajectoryRequireEndpointRest || ...
        (isfield(options,'goalHold_s') && options.goalHold_s > 0);
    if requireFinalRest && ...
            baseSegments(end).kind ~= "wait"
        event = template;
        event.boundaryTime_s = baseSegments(end).endTime_s;
        event.leftBase = numel(baseSegments);
        event.rightBase = 0;
        event.kind = "goalStop";
        event = configureEventPair(event,baseSegments,0,0);
        events(end+1,1) = event;
    end
    if isempty(events) && options.trajectoryBlendAttempts < 1
        error('smoothAzElTrajectory:InternalError','Invalid blend setup.');
    end
end


function event = configureEvent(event,baseSegments,fraction)
    event = configureEventPair( ...
        event,baseSegments,fraction,fraction);
end


function event = configureEventPair( ...
        event,baseSegments,leftFraction,rightFraction)
    event.fraction = max(leftFraction,rightFraction);
    if event.leftBase > 0
        left = baseSegments(event.leftBase);
        event.leftExtent_s = leftFraction* ...
            (left.endTime_s-left.startTime_s);
    else
        event.leftExtent_s = 0;
    end
    if event.rightBase > 0
        right = baseSegments(event.rightBase);
        event.rightExtent_s = rightFraction* ...
            (right.endTime_s-right.startTime_s);
    else
        event.rightExtent_s = 0;
    end
    if event.leftExtent_s+event.rightExtent_s > 0
        transition = eventToSegment(event,baseSegments);
        event.controlPoints_deg = transition.controlPoints_deg;
        event.configured = true;
    else
        event.controlPoints_deg = zeros(6,2);
        event.configured = false;
    end
end


function pairs = makeFractionPairs(event,fractions,baseSegments)
    if event.kind == "startRest"
        pairs = [zeros(numel(fractions),1),fractions(:)];
        return
    end
    if event.kind == "goalStop"
        pairs = [fractions(:),zeros(numel(fractions),1)];
        return
    end
    if event.kind ~= "waitArrival" && event.kind ~= "waitDeparture"
        pairs = [fractions(:),fractions(:)];
        return
    end
    values = [0,fractions];
    [left,right] = ndgrid(values,values);
    candidates = [left(:),right(:)];
    if event.kind == "waitArrival"
        % Consuming only the stationary side would start at the join with a
        % nonzero incoming rate and return to the same point, creating a loop.
        candidates = candidates(candidates(:,1) > 0,:);
    else
        % The mirror-image zero-moving-side case loops before departure.
        candidates = candidates(candidates(:,2) > 0,:);
    end
    leftDuration_s = 0;
    rightDuration_s = 0;
    if event.leftBase > 0
        left = baseSegments(event.leftBase);
        leftDuration_s = left.endTime_s-left.startTime_s;
    end
    if event.rightBase > 0
        right = baseSegments(event.rightBase);
        rightDuration_s = right.endTime_s-right.startTime_s;
    end
    leftExtent_s = candidates(:,1)*leftDuration_s;
    rightExtent_s = candidates(:,2)*rightDuration_s;
    score = leftExtent_s+rightExtent_s;
    imbalance = abs(leftExtent_s-rightExtent_s);
    [~,order] = sortrows([-score,imbalance],[1,2]);
    pairs = unique(candidates(order,:),'rows','stable');
end


function segment = eventToSegment(event,baseSegments)
    startTime_s = event.boundaryTime_s-event.leftExtent_s;
    endTime_s = event.boundaryTime_s+event.rightExtent_s;
    duration_s = endTime_s-startTime_s;
    if duration_s <= 0
        error('smoothAzElTrajectory:ZeroBlend','A blend has zero duration.');
    end

    if event.leftBase > 0
        left = baseSegments(event.leftBase);
        startPoint_deg = evaluateBaseSegment(left,startTime_s);
        startVelocity_deg_s = left.velocity_deg_s;
    else
        right = baseSegments(event.rightBase);
        startPoint_deg = right.startPoint_deg;
        startVelocity_deg_s = [0,0];
    end
    if event.rightBase > 0
        right = baseSegments(event.rightBase);
        endPoint_deg = evaluateBaseSegment(right,endTime_s);
        endVelocity_deg_s = right.velocity_deg_s;
    else
        endPoint_deg = left.endPoint_deg;
        endVelocity_deg_s = [0,0];
    end
    controlPoints_deg = quinticHermiteBezier( ...
        startPoint_deg,endPoint_deg,startVelocity_deg_s, ...
        endVelocity_deg_s,duration_s);
    segment = makeOutputSegment( ...
        startTime_s,endTime_s,controlPoints_deg,event.kind);
end


function segments = eventToSegments(event,baseSegments)
    % Subdivision leaves the exact quintic curve unchanged while tightening
    % the convex-hull derivative certificate used by the independent audit.
    % This matters for one-sided wait blends: an unsplit quintic has loose
    % velocity control points even when its true rate is comfortably safe.
    segments = eventToSegment(event,baseSegments);
    for subdivision = 1:2
        refined = repmat(segments(1),2*numel(segments),1);
        for segmentIndex = 1:numel(segments)
            pieces = splitOutputSegment(segments(segmentIndex),0.5);
            refined(2*segmentIndex-1:2*segmentIndex) = pieces;
        end
        segments = refined;
    end
end


function pieces = splitOutputSegment(segment,fraction)
    controlPoints = segment.controlPoints_deg;
    degree = size(controlPoints,1)-1;
    triangle = zeros(degree+1,degree+1,size(controlPoints,2));
    triangle(:,1,:) = controlPoints;
    for column = 2:degree+1
        count = degree-column+2;
        triangle(1:count,column,:) = (1-fraction)* ...
            triangle(1:count,column-1,:)+fraction* ...
            triangle(2:count+1,column-1,:);
    end
    left = zeros(size(controlPoints));
    right = zeros(size(controlPoints));
    for index = 1:degree+1
        left(index,:) = reshape(triangle(1,index,:),1,[]);
        right(degree-index+2,:) = reshape( ...
            triangle(degree-index+2,index,:),1,[]);
    end
    splitTime_s = segment.startTime_s+fraction* ...
        (segment.endTime_s-segment.startTime_s);
    pieces = repmat(segment,2,1);
    pieces(1) = makeOutputSegment(segment.startTime_s,splitTime_s, ...
        left,segment.kind);
    pieces(2) = makeOutputSegment(splitTime_s,segment.endTime_s, ...
        right,segment.kind);
end


function point_deg = evaluateBaseSegment(segment,time_s)
    point_deg = segment.startPoint_deg+ ...
        (time_s-segment.startTime_s)*segment.velocity_deg_s;
end


function controlPoints_deg = quinticHermiteBezier( ...
        startPoint_deg,endPoint_deg,startVelocity_deg_s, ...
        endVelocity_deg_s,duration_s)
    controlPoints_deg = zeros(6,2);
    controlPoints_deg(1,:) = startPoint_deg;
    controlPoints_deg(2,:) = startPoint_deg+ ...
        duration_s*startVelocity_deg_s/5;
    controlPoints_deg(3,:) = startPoint_deg+ ...
        2*duration_s*startVelocity_deg_s/5;
    controlPoints_deg(6,:) = endPoint_deg;
    controlPoints_deg(5,:) = endPoint_deg- ...
        duration_s*endVelocity_deg_s/5;
    controlPoints_deg(4,:) = endPoint_deg- ...
        2*duration_s*endVelocity_deg_s/5;
end


function segments = assembleSegments(baseSegments,events)
    numBase = numel(baseSegments);
    trimStart_s = zeros(numBase,1);
    trimEnd_s = zeros(numBase,1);
    for eventIndex = 1:numel(events)
        event = events(eventIndex);
        if ~event.configured
            continue
        end
        if event.leftBase > 0
            trimEnd_s(event.leftBase) = event.leftExtent_s;
        end
        if event.rightBase > 0
            trimStart_s(event.rightBase) = event.rightExtent_s;
        end
    end

    template = makeOutputSegment(0,1,zeros(6,2),"line");
    segments = repmat(template,0,1);
    for baseIndex = 1:numBase
        base = baseSegments(baseIndex);
        startTime_s = base.startTime_s+trimStart_s(baseIndex);
        endTime_s = base.endTime_s-trimEnd_s(baseIndex);
        if endTime_s-startTime_s > 1e-10
            firstPoint = evaluateBaseSegment(base,startTime_s);
            secondPoint = evaluateBaseSegment(base,endTime_s);
            controlPoints = straightBezier(firstPoint,secondPoint);
            segments(end+1,1) = makeOutputSegment( ...
                startTime_s,endTime_s,controlPoints,base.kind); %#ok<AGROW>
        end
    end
    for eventIndex = 1:numel(events)
        if events(eventIndex).configured
            transition = eventToSegments(events(eventIndex),baseSegments);
            segments = [segments;transition]; %#ok<AGROW>
        end
    end
    [~,order] = sort([segments.startTime_s]);
    segments = segments(order);
end


function controlPoints_deg = straightBezier(firstPoint,secondPoint)
    fraction = (0:5)'/5;
    controlPoints_deg = firstPoint+fraction.*(secondPoint-firstPoint);
end


function segment = makeOutputSegment( ...
        startTime_s,endTime_s,controlPoints_deg,kind)
    segment = struct('startTime_s',startTime_s,'endTime_s',endTime_s, ...
        'controlPoints_deg',controlPoints_deg,'kind',string(kind));
end


function trajectory = sampleTrajectory(segments,path,data,options,success)
    startTime_s = path.time_s(1);
    endTime_s = path.time_s(end);
    regularTimes_s = (startTime_s:options.trajectorySampleTime_s:endTime_s)';
    segmentTimes_s = reshape([[segments.startTime_s]; ...
        [segments.endTime_s]],[],1);
    sampleTimes_s = sort([regularTimes_s;endTime_s;path.time_s(:); ...
        segmentTimes_s]);
    if isempty(sampleTimes_s)
        sampleTimes_s = startTime_s;
    end
    absoluteScale_s = max(1,max(abs(sampleTimes_s)));
    span_s = max(1,endTime_s-startTime_s);
    timeTolerance_s = max(64*eps(absoluteScale_s),1e-13*span_s);
    keep = [true;diff(sampleTimes_s) > timeTolerance_s];
    sampleTimes_s = sampleTimes_s(keep);

    numSamples = numel(sampleTimes_s);
    position_deg = zeros(numSamples,2);
    velocity_deg_s = zeros(numSamples,2);
    acceleration_deg_s2 = zeros(numSamples,2);
    jerk_deg_s3 = zeros(numSamples,2);
    segmentIndex = 1;
    for sampleIndex = 1:numSamples
        time_s = sampleTimes_s(sampleIndex);
        while segmentIndex < numel(segments) && ...
                time_s >= segments(segmentIndex).endTime_s
            segmentIndex = segmentIndex+1;
        end
        segment = segments(segmentIndex);
        duration_s = segment.endTime_s-segment.startTime_s;
        parameter = min(1,max(0,(time_s-segment.startTime_s)/duration_s));
        [position_deg(sampleIndex,:),velocity_deg_s(sampleIndex,:), ...
            acceleration_deg_s2(sampleIndex,:),jerk_deg_s3(sampleIndex,:)] = ...
            evaluateSegment(segment,parameter);
    end

    trajectory = struct;
    trajectory.success = logical(success);
    trajectory.message = "Smooth trajectory candidate.";
    trajectory.time_s = sampleTimes_s;
    trajectory.az_deg = position_deg(:,1);
    trajectory.el_deg = position_deg(:,2);
    trajectory.azWrapped_deg = wrapTrajectoryAzimuth( ...
        trajectory.az_deg,options);
    trajectory.timeIndex = mapPathIndex( ...
        sampleTimes_s,path,data,'timeIndex');
    trajectory.planningTimeIndex = mapPathIndex( ...
        sampleTimes_s,path,data,'planningTimeIndex');
    trajectory.isWaiting = hypot(velocity_deg_s(:,1), ...
        velocity_deg_s(:,2)) <= 1e-9;
    trajectory.azRate_deg_s = velocity_deg_s(:,1);
    trajectory.elRate_deg_s = velocity_deg_s(:,2);
    trajectory.azAcceleration_deg_s2 = acceleration_deg_s2(:,1);
    trajectory.elAcceleration_deg_s2 = acceleration_deg_s2(:,2);
    trajectory.azJerk_deg_s3 = jerk_deg_s3(:,1);
    trajectory.elJerk_deg_s3 = jerk_deg_s3(:,2);
    trajectory.segments = segments;
    trajectory.c2Continuous = checkC2Continuity(segments);
    trajectory.numFillets = countFilletGroups(segments);
    trajectory.maxHeadingJump_deg = maximumHeadingJump(trajectory);
end


function wrappedAzimuth_deg = wrapTrajectoryAzimuth(azimuth_deg,options)
    periodic = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    if periodic && isfield(options,'azLim_deg')
        wrappedAzimuth_deg = mod(azimuth_deg-options.azLim_deg(1),360)+ ...
            options.azLim_deg(1);
    else
        wrappedAzimuth_deg = azimuth_deg;
    end
end


function mappedIndex = mapPathIndex(sampleTimes_s,path,data,fieldName)
    if isfield(path,fieldName) && numel(path.(fieldName)) == numel(path.time_s)
        sourceIndex = path.(fieldName)(:);
    elseif isfield(data,'time_s') && ~isempty(data.time_s)
        sourceIndex = zeros(numel(path.time_s),1);
        for pathIndex = 1:numel(path.time_s)
            [~,sourceIndex(pathIndex)] = min( ...
                abs(data.time_s(:)-path.time_s(pathIndex)));
        end
    else
        sourceIndex = (1:numel(path.time_s))';
    end
    mappedIndex = zeros(numel(sampleTimes_s),1);
    for sampleIndex = 1:numel(sampleTimes_s)
        [~,nearest] = min(abs(path.time_s(:)-sampleTimes_s(sampleIndex)));
        mappedIndex(sampleIndex) = sourceIndex(nearest);
    end
end


function [position,velocity,acceleration,jerk] = ...
        evaluateSegment(segment,parameter)
    duration_s = segment.endTime_s-segment.startTime_s;
    position = evaluateBezier(segment.controlPoints_deg,parameter);
    velocityControl = 5*diff(segment.controlPoints_deg,1,1)/duration_s;
    accelerationControl = 4*diff(velocityControl,1,1)/duration_s;
    jerkControl = 3*diff(accelerationControl,1,1)/duration_s;
    velocity = evaluateBezier(velocityControl,parameter);
    acceleration = evaluateBezier(accelerationControl,parameter);
    jerk = evaluateBezier(jerkControl,parameter);
end


function value = evaluateBezier(controlPoints,parameter)
    degree = size(controlPoints,1)-1;
    parameter = parameter(:);
    value = zeros(numel(parameter),size(controlPoints,2));
    for controlIndex = 0:degree
        weight = nchoosek(degree,controlIndex)* ...
            parameter.^controlIndex.*(1-parameter).^(degree-controlIndex);
        value = value+weight*controlPoints(controlIndex+1,:);
    end
end


function extrema = segmentDerivativeExtrema(segment)
    duration_s = segment.endTime_s-segment.startTime_s;
    velocityControl = 5*diff(segment.controlPoints_deg,1,1)/duration_s;
    accelerationControl = 4*diff(velocityControl,1,1)/duration_s;
    jerkControl = 3*diff(accelerationControl,1,1)/duration_s;
    extrema = struct;
    extrema.maxAzRate_deg_s = maximumAbsoluteBezier(velocityControl(:,1));
    extrema.maxElRate_deg_s = maximumAbsoluteBezier(velocityControl(:,2));
    extrema.maxAzAcceleration_deg_s2 = ...
        maximumAbsoluteBezier(accelerationControl(:,1));
    extrema.maxElAcceleration_deg_s2 = ...
        maximumAbsoluteBezier(accelerationControl(:,2));
    extrema.maxAzJerk_deg_s3 = maximumAbsoluteBezier(jerkControl(:,1));
    extrema.maxElJerk_deg_s3 = maximumAbsoluteBezier(jerkControl(:,2));
end


function bounds = segmentDerivativeControlBounds(segment)
    duration_s = segment.endTime_s-segment.startTime_s;
    velocityControl = 5*diff(segment.controlPoints_deg,1,1)/duration_s;
    accelerationControl = 4*diff(velocityControl,1,1)/duration_s;
    jerkControl = 3*diff(accelerationControl,1,1)/duration_s;
    bounds = struct;
    bounds.maxAzRate_deg_s = max(abs(velocityControl(:,1)));
    bounds.maxElRate_deg_s = max(abs(velocityControl(:,2)));
    bounds.maxAzAcceleration_deg_s2 = ...
        max(abs(accelerationControl(:,1)));
    bounds.maxElAcceleration_deg_s2 = ...
        max(abs(accelerationControl(:,2)));
    bounds.maxAzJerk_deg_s3 = max(abs(jerkControl(:,1)));
    bounds.maxElJerk_deg_s3 = max(abs(jerkControl(:,2)));
end


function extrema = trajectoryDerivativeExtrema(segments)
    extrema = zeroExtrema;
    for segmentIndex = 1:numel(segments)
        local = segmentDerivativeExtrema(segments(segmentIndex));
        names = fieldnames(extrema);
        for nameIndex = 1:numel(names)
            name = names{nameIndex};
            extrema.(name) = max(extrema.(name),local.(name));
        end
    end
end


function bounds = trajectoryDerivativeControlBounds(segments)
    bounds = zeroExtrema;
    for segmentIndex = 1:numel(segments)
        local = segmentDerivativeControlBounds(segments(segmentIndex));
        names = fieldnames(bounds);
        for nameIndex = 1:numel(names)
            name = names{nameIndex};
            bounds.(name) = max(bounds.(name),local.(name));
        end
    end
end


function extrema = zeroExtrema
    extrema = struct('maxAzRate_deg_s',0,'maxElRate_deg_s',0, ...
        'maxAzAcceleration_deg_s2',0,'maxElAcceleration_deg_s2',0, ...
        'maxAzJerk_deg_s3',0,'maxElJerk_deg_s3',0);
end


function maximum = maximumAbsoluteBezier(controlPoints)
    controlPoints = controlPoints(:);
    degree = numel(controlPoints)-1;
    candidates = [0;1];
    if degree > 0
        derivativeControl = degree*diff(controlPoints);
        stationary = realBezierRoots(derivativeControl);
        candidates = unique([candidates;stationary]);
    end
    values = evaluateBezier(controlPoints,candidates);
    maximum = max(abs(values));
    if isempty(maximum)
        maximum = 0;
    end
end


function realRoots = realBezierRoots(controlPoints)
    powerAscending = bernsteinToPower(controlPoints(:));
    scale = max(1,max(abs(powerAscending)));
    last = find(abs(powerAscending) > 1e-12*scale,1,'last');
    if isempty(last) || last == 1
        realRoots = zeros(0,1);
        return
    end
    polynomialRoots = roots(flipud(powerAscending(1:last)));
    realMask = abs(imag(polynomialRoots)) <= 1e-9* ...
        max(1,abs(real(polynomialRoots)));
    realRoots = real(polynomialRoots(realMask));
    realRoots = realRoots(realRoots > 0 & realRoots < 1);
end


function powerAscending = bernsteinToPower(controlPoints)
    degree = numel(controlPoints)-1;
    powerAscending = zeros(degree+1,1);
    for power = 0:degree
        sumValue = 0;
        for index = 0:power
            sumValue = sumValue+(-1)^(power-index)* ...
                nchoosek(power,index)*controlPoints(index+1);
        end
        powerAscending(power+1) = nchoosek(degree,power)*sumValue;
    end
end


function within = withinKinematicLimits(extrema,options)
    tolerance = 1e-9;
    within = extrema.maxAzRate_deg_s <= options.azRate_deg_s+tolerance && ...
        extrema.maxElRate_deg_s <= options.elRate_deg_s+tolerance && ...
        extrema.maxAzAcceleration_deg_s2 <= ...
        options.maxAzAcceleration_deg_s2+tolerance && ...
        extrema.maxElAcceleration_deg_s2 <= ...
        options.maxElAcceleration_deg_s2+tolerance && ...
        extrema.maxAzJerk_deg_s3 <= options.maxAzJerk_deg_s3+tolerance && ...
        extrema.maxElJerk_deg_s3 <= options.maxElJerk_deg_s3+tolerance;
end


function inside = controlPointsInsideLimits(controlPoints,options)
    tolerance = 1e-9;
    inside = true;
    if isfield(options,'elLim_deg')
        inside = inside && all(controlPoints(:,2) >= ...
            options.elLim_deg(1)-tolerance & controlPoints(:,2) <= ...
            options.elLim_deg(2)+tolerance);
    end
    periodic = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    if ~periodic && isfield(options,'azLim_deg')
        inside = inside && all(controlPoints(:,1) >= ...
            options.azLim_deg(1)-tolerance & controlPoints(:,1) <= ...
            options.azLim_deg(2)+tolerance);
    end
end


function continuous = checkC2Continuity(segments)
    continuous = true;
    for segmentIndex = 1:numel(segments)-1
        first = segments(segmentIndex);
        second = segments(segmentIndex+1);
        firstDuration = first.endTime_s-first.startTime_s;
        secondDuration = second.endTime_s-second.startTime_s;
        firstVelocity = 5*diff(first.controlPoints_deg,1,1)/firstDuration;
        secondVelocity = 5*diff(second.controlPoints_deg,1,1)/secondDuration;
        firstAcceleration = 4*diff(firstVelocity,1,1)/firstDuration;
        secondAcceleration = 4*diff(secondVelocity,1,1)/secondDuration;
        values = [first.endTime_s-second.startTime_s; ...
            (first.controlPoints_deg(end,:)-second.controlPoints_deg(1,:))'; ...
            (firstVelocity(end,:)-secondVelocity(1,:))'; ...
            (firstAcceleration(end,:)-secondAcceleration(1,:))'];
        scale = max(1,max(abs([first.controlPoints_deg(:); ...
            second.controlPoints_deg(:);firstVelocity(:);secondVelocity(:)])));
        if any(abs(values) > 1e-7*scale)
            continuous = false;
            return
        end
    end
end


function maximumChange_deg = maximumHeadingJump(trajectory)
    displacement = [diff(trajectory.az_deg),diff(trajectory.el_deg)];
    moving = hypot(displacement(:,1),displacement(:,2)) > 1e-9;
    heading = atan2(displacement(moving,2),displacement(moving,1));
    if numel(heading) < 2
        maximumChange_deg = 0;
        return
    end
    change = atan2(sin(diff(heading)),cos(diff(heading)));
    maximumChange_deg = max(abs(rad2deg(change)));
end


function count = countFilletGroups(segments)
    transition = arrayfun(@(segment) segment.kind ~= "line" && ...
        segment.kind ~= "wait",segments);
    count = sum(transition & [true;~transition(1:end-1)]);
end


function info = makeInfo(trajectory,collisionFree,audit)
    if isempty(trajectory.segments)
        extrema = zeroExtrema;
    else
        extrema = trajectoryDerivativeExtrema(trajectory.segments);
    end
    info = extrema;
    info.success = logical(trajectory.success && collisionFree);
    info.message = trajectory.message;
    info.numFillets = trajectory.numFillets;
    info.c2Continuous = trajectory.c2Continuous;
    info.collisionFree = logical(collisionFree);
    info.maxHeadingJump_deg = trajectory.maxHeadingJump_deg;
    info.audit = audit;
end


function trajectory = makePointTrajectory(path,data,options)
    trajectory = struct;
    trajectory.success = true;
    trajectory.message = "Stationary one-sample trajectory.";
    trajectory.time_s = path.time_s(:);
    trajectory.az_deg = path.az_deg(:);
    trajectory.el_deg = path.el_deg(:);
    trajectory.azWrapped_deg = wrapTrajectoryAzimuth( ...
        trajectory.az_deg,options);
    trajectory.timeIndex = mapPathIndex( ...
        trajectory.time_s,path,data,'timeIndex');
    trajectory.planningTimeIndex = mapPathIndex( ...
        trajectory.time_s,path,data,'planningTimeIndex');
    trajectory.isWaiting = true;
    trajectory.azRate_deg_s = 0;
    trajectory.elRate_deg_s = 0;
    trajectory.azAcceleration_deg_s2 = 0;
    trajectory.elAcceleration_deg_s2 = 0;
    trajectory.azJerk_deg_s3 = 0;
    trajectory.elJerk_deg_s3 = 0;
    trajectory.segments = repmat( ...
        makeOutputSegment(0,1,zeros(6,2),"wait"),0,1);
    trajectory.c2Continuous = true;
    trajectory.numFillets = 0;
    trajectory.maxHeadingJump_deg = 0;
end
