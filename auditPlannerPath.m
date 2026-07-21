function audit = auditPlannerPath(scenario,result)
%AUDITPLANNERPATH Independently verify continuous path clearance.
%   AUDIT = AUDITPLANNERPATH(SCENARIO,RESULT) checks every reported path
%   segment against the original static polygons and conservative swept
%   dynamic polygons. When RESULT contains an analytic smooth trajectory,
%   the audit dispatches to AUDITAZELTRAJECTORY and certifies the quintic
%   curve itself. Dynamic vertices are assumed to move linearly between
%   adjacent data frames. A successful result is also checked while holding
%   its final position for OPTIONS.goalHold_s.

    audit = struct( ...
        'collisionFree',true, ...
        'holdSafe',true, ...
        'requiredClearance_deg',0, ...
        'minimumClearance_deg',Inf, ...
        'clearanceMargin_deg',Inf, ...
        'checkedPathSegments',0, ...
        'checkedHoldSegments',0, ...
        'offendingPathSegment',NaN, ...
        'offendingInterval_s',[NaN,NaN], ...
        'offendingObstacleType',"", ...
        'message',"safe continuous path");

    if ~isfield(scenario,'data') || ~isfield(scenario,'options')
        audit = failAudit(audit,"scenario is missing data or options", ...
            NaN,[NaN,NaN],"input");
        return
    end
    if isfield(result,'path') && isstruct(result.path) && ...
            isfield(result.path,'segments') && ~isempty(result.path.segments)
        audit = auditAzElTrajectory(scenario,result.path,true);
        return
    end
    if isfield(result,'trajectory') && isstruct(result.trajectory) && ...
            isfield(result.trajectory,'segments') && ...
            ~isempty(result.trajectory.segments)
        audit = auditAzElTrajectory(scenario,result.trajectory,true);
        return
    end
    if ~isfield(result,'path') || isempty(result.path) || ...
            ~isfield(result.path,'time_s') || isempty(result.path.time_s)
        if isfield(result,'success') && result.success
            audit = failAudit(audit, ...
                "successful result contains no path samples", ...
                NaN,[NaN,NaN],"input");
        else
            audit.message = "no path samples to audit";
        end
        return
    end

    path = result.path;
    numSamples = numel(path.time_s);
    requiredFields = {'az_deg','el_deg'};
    for fieldIndex = 1:numel(requiredFields)
        fieldName = requiredFields{fieldIndex};
        if ~isfield(path,fieldName) || numel(path.(fieldName)) ~= numSamples
            audit = failAudit(audit, ...
                "path coordinate histories have inconsistent lengths", ...
                NaN,[NaN,NaN],"input");
            return
        end
    end
    if any(~isfinite(path.time_s(:))) || any(~isfinite(path.az_deg(:))) || ...
            any(~isfinite(path.el_deg(:))) || any(diff(path.time_s(:)) < 0)
        audit = failAudit(audit,"path contains invalid samples", ...
            NaN,[NaN,NaN],"input");
        return
    end

    options = scenario.options;
    if isfield(options,'clearance_deg') && ~isempty(options.clearance_deg)
        audit.requiredClearance_deg = max(options.clearance_deg(:));
    end
    periodicAzimuth = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    staticPolygons = {};
    if isfield(options,'staticPolygons') && ~isempty(options.staticPolygons)
        staticPolygons = options.staticPolygons;
    end
    data = scenario.data;
    frameIndices = locateFrameIndices(data,path);
    if isempty(frameIndices)
        audit = failAudit(audit, ...
            "path times cannot be mapped to obstacle frames", ...
            NaN,[NaN,NaN],"input");
        return
    end

    for sampleIndex = 1:numSamples
        point = [path.az_deg(sampleIndex),path.el_deg(sampleIndex)];
        [safe,distance] = segmentSetIsSafe(point,point,staticPolygons, ...
            audit.requiredClearance_deg,periodicAzimuth);
        audit.minimumClearance_deg = min( ...
            audit.minimumClearance_deg,distance);
        if ~safe
            audit = failAudit(audit,"static obstacle at a path sample", ...
                max(0,sampleIndex-1),path.time_s(sampleIndex)*[1,1], ...
                "static");
            audit = finishAudit(audit);
            return
        end

        framePolygons = getFramePolygons(data,frameIndices(sampleIndex));
        [safe,distance] = segmentSetIsSafe(point,point,framePolygons, ...
            audit.requiredClearance_deg,periodicAzimuth);
        audit.minimumClearance_deg = min( ...
            audit.minimumClearance_deg,distance);
        if ~safe
            audit = failAudit(audit,"dynamic obstacle at a path sample", ...
                max(0,sampleIndex-1),path.time_s(sampleIndex)*[1,1], ...
                "dynamicFrame");
            audit = finishAudit(audit);
            return
        end
    end

    for segmentIndex = 1:numSamples-1
        firstPoint = [path.az_deg(segmentIndex),path.el_deg(segmentIndex)];
        secondPoint = [path.az_deg(segmentIndex+1),path.el_deg(segmentIndex+1)];
        interval_s = path.time_s([segmentIndex,segmentIndex+1])';
        [safe,distance] = segmentSetIsSafe(firstPoint,secondPoint, ...
            staticPolygons,audit.requiredClearance_deg,periodicAzimuth);
        audit.minimumClearance_deg = min( ...
            audit.minimumClearance_deg,distance);
        audit.checkedPathSegments = audit.checkedPathSegments+1;
        if ~safe
            audit = failAudit(audit,"path segment intersects a static obstacle", ...
                segmentIndex,interval_s,"static");
            audit = finishAudit(audit);
            return
        end

        firstFrame = frameIndices(segmentIndex);
        lastFrame = frameIndices(segmentIndex+1);
        if lastFrame < firstFrame
            audit = failAudit(audit,"path frame indices are not monotonic", ...
                segmentIndex,interval_s,"input");
            audit = finishAudit(audit);
            return
        end
        for frameIndex = firstFrame:lastFrame-1
            frameInterval_s = data.time_s([frameIndex,frameIndex+1])';
            [subsegmentStart,subsegmentEnd] = interpolatePathInterval( ...
                firstPoint,secondPoint,interval_s,frameInterval_s);
            [safe,distance] = dynamicIntervalIsSafe( ...
                subsegmentStart,subsegmentEnd,data,frameIndex, ...
                audit.requiredClearance_deg,periodicAzimuth);
            audit.minimumClearance_deg = min( ...
                audit.minimumClearance_deg,distance);
            if ~safe
                audit = failAudit(audit, ...
                    "path segment intersects a swept dynamic obstacle", ...
                    segmentIndex,frameInterval_s,"dynamicSweep");
                audit = finishAudit(audit);
                return
            end
        end
    end

    if isfield(result,'success') && result.success
        [audit,holdPassed] = auditGoalHold( ...
            audit,data,path,frameIndices(end),options,periodicAzimuth);
        if ~holdPassed
            audit = finishAudit(audit);
            return
        end
    end
    audit = finishAudit(audit);
end


function [audit,passed] = auditGoalHold( ...
        audit,data,path,finalFrame,options,periodicAzimuth)
    passed = true;
    if ~isfield(options,'goalHold_s') || options.goalHold_s <= 0
        return
    end
    dt_s = median(diff(data.time_s(:)));
    holdSteps = max(0,ceil(options.goalHold_s/dt_s-1e-12));
    if finalFrame+holdSteps > numel(data.time_s)
        audit.holdSafe = false;
        audit = failAudit(audit,"goal hold extends beyond obstacle data", ...
            numel(path.time_s),[path.time_s(end),path.time_s(end)+ ...
            options.goalHold_s],"goalHold");
        passed = false;
        return
    end

    holdPoint = [path.az_deg(end),path.el_deg(end)];
    for frameIndex = finalFrame:finalFrame+holdSteps-1
        [safe,distance] = dynamicIntervalIsSafe( ...
            holdPoint,holdPoint,data,frameIndex, ...
            audit.requiredClearance_deg,periodicAzimuth);
        audit.minimumClearance_deg = min( ...
            audit.minimumClearance_deg,distance);
        audit.checkedHoldSegments = audit.checkedHoldSegments+1;
        if ~safe
            audit.holdSafe = false;
            interval_s = data.time_s([frameIndex,frameIndex+1])';
            audit = failAudit(audit, ...
                "dynamic obstacle crosses the goal during the hold interval", ...
                numel(path.time_s),interval_s,"goalHold");
            passed = false;
            return
        end
    end
end


function frameIndices = locateFrameIndices(data,path)
    frameIndices = zeros(numel(path.time_s),1);
    time_s = data.time_s(:);
    if isempty(time_s)
        frameIndices = [];
        return
    end
    if isfield(path,'timeIndex') && numel(path.timeIndex) == numel(path.time_s)
        candidate = round(path.timeIndex(:));
        if all(candidate >= 1 & candidate <= numel(time_s)) && ...
                all(abs(time_s(candidate)-path.time_s(:)) <= ...
                1e-8*max(1,max(abs(time_s))))
            frameIndices = candidate;
            return
        end
    end
    tolerance = 1e-8*max(1,max(abs(time_s)));
    for sampleIndex = 1:numel(path.time_s)
        [difference,frameIndices(sampleIndex)] = min( ...
            abs(time_s-path.time_s(sampleIndex)));
        if difference > tolerance
            frameIndices = [];
            return
        end
    end
end


function [firstPoint,secondPoint] = interpolatePathInterval( ...
        pathStart,pathEnd,pathInterval_s,frameInterval_s)
    duration_s = pathInterval_s(2)-pathInterval_s(1);
    if duration_s <= 0
        firstPoint = pathStart;
        secondPoint = pathEnd;
        return
    end
    fraction = (frameInterval_s-pathInterval_s(1))/duration_s;
    fraction = min(1,max(0,fraction));
    firstPoint = pathStart+fraction(1)*(pathEnd-pathStart);
    secondPoint = pathStart+fraction(2)*(pathEnd-pathStart);
end


function [safe,minimumDistance] = dynamicIntervalIsSafe( ...
        pathStart,pathEnd,data,frameIndex,clearance_deg,isPeriodic)
    firstPolygons = getFramePolygons(data,frameIndex);
    secondPolygons = getFramePolygons(data,frameIndex+1);
    if numel(firstPolygons) ~= numel(secondPolygons)
        [safe,minimumDistance] = segmentSetIsSafe( ...
            pathStart,pathEnd,[firstPolygons,secondPolygons], ...
            clearance_deg,isPeriodic);
        return
    end

    safe = true;
    minimumDistance = Inf;
    for polygonIndex = 1:numel(firstPolygons)
        firstPolygon = openPolygon(preparePolygon( ...
            firstPolygons{polygonIndex}));
        secondPolygon = openPolygon(preparePolygon( ...
            secondPolygons{polygonIndex}));
        if isempty(firstPolygon) || isempty(secondPolygon)
            [polygonSafe,distance] = segmentSetIsSafe( ...
                pathStart,pathEnd,{[firstPolygon;secondPolygon]}, ...
                clearance_deg,isPeriodic);
            minimumDistance = min(minimumDistance,distance);
            if ~polygonSafe
                safe = false;
                return
            end
            continue
        end
        [firstPolygon,secondPolygon] = alignPolygonPair( ...
            firstPolygon,secondPolygon,pathStart,pathEnd,isPeriodic);
        if size(firstPolygon,1) == size(secondPolygon,1)
            [polygonSafe,distance] = movingPolygonIntervalIsSafe( ...
                pathStart,pathEnd,firstPolygon,secondPolygon,clearance_deg);
        else
            envelope = conservativeMismatchEnvelope( ...
                firstPolygon,secondPolygon);
            [polygonSafe,distance] = segmentSetIsSafe( ...
                pathStart,pathEnd,{envelope},clearance_deg,false);
        end
        minimumDistance = min(minimumDistance,distance);
        if ~polygonSafe
            safe = false;
            return
        end
    end
end


function [firstPolygon,secondPolygon] = alignPolygonPair( ...
        firstPolygon,secondPolygon,pathStart,pathEnd,isPeriodic)
    if ~isPeriodic
        return
    end
    secondShift = 360*round((mean(firstPolygon(:,1))- ...
        mean(secondPolygon(:,1)))/360);
    secondPolygon(:,1) = secondPolygon(:,1)+secondShift;
    pathCenterAz_deg = 0.5*(pathStart(1)+pathEnd(1));
    pairCenterAz_deg = 0.5*(mean(firstPolygon(:,1))+ ...
        mean(secondPolygon(:,1)));
    pairShift = 360*round((pathCenterAz_deg-pairCenterAz_deg)/360);
    firstPolygon(:,1) = firstPolygon(:,1)+pairShift;
    secondPolygon(:,1) = secondPolygon(:,1)+pairShift;
end


function envelope = conservativeMismatchEnvelope(firstPolygon,secondPolygon)
    vertices = unique([firstPolygon;secondPolygon],'rows','stable');
    if size(vertices,1) >= 3 && ...
            rank(vertices(2:end,:)-vertices(1,:)) >= 2
        hullIndices = convhull(vertices(:,1),vertices(:,2));
        envelope = vertices(hullIndices,:);
    else
        envelope = vertices;
    end
end


function [safe,minimumDistance] = movingPolygonIntervalIsSafe( ...
        pathStart,pathEnd,firstPolygon,secondPolygon,clearance_deg)
    pathTravel = norm(pathEnd-pathStart);
    vertexTravel = hypot(secondPolygon(:,1)-firstPolygon(:,1), ...
        secondPolygon(:,2)-firstPolygon(:,2));
    relativeMotionBound = pathTravel+max(vertexTravel,[],'omitnan');
    [safe,minimumDistance] = certifyMotionInterval( ...
        pathStart,pathEnd,firstPolygon,secondPolygon,clearance_deg, ...
        relativeMotionBound,0,1,0);
end


function [safe,minimumDistance] = certifyMotionInterval( ...
        pathStart,pathEnd,firstPolygon,secondPolygon,clearance_deg, ...
        relativeMotionBound,firstFraction,lastFraction,depth)
    tolerance = 1e-10;
    maxDepth = 22;
    midpoint = 0.5*(firstFraction+lastFraction);
    pathPoint = pathStart+midpoint*(pathEnd-pathStart);
    polygon = firstPolygon+midpoint*(secondPolygon-firstPolygon);
    midpointDistance = segmentPolygonDistance(pathPoint,pathPoint,polygon);
    halfWidth = 0.5*(lastFraction-firstFraction);
    lowerBound = max(0,midpointDistance-relativeMotionBound*halfWidth);
    minimumDistance = lowerBound;

    if midpointDistance <= clearance_deg+tolerance
        safe = false;
        minimumDistance = midpointDistance;
        return
    end
    if lowerBound > clearance_deg+tolerance
        safe = true;
        return
    end
    if depth >= maxDepth
        safe = false;
        return
    end

    [safe,firstDistance] = certifyMotionInterval( ...
        pathStart,pathEnd,firstPolygon,secondPolygon,clearance_deg, ...
        relativeMotionBound,firstFraction,midpoint,depth+1);
    minimumDistance = firstDistance;
    if ~safe
        return
    end
    [safe,secondDistance] = certifyMotionInterval( ...
        pathStart,pathEnd,firstPolygon,secondPolygon,clearance_deg, ...
        relativeMotionBound,midpoint,lastFraction,depth+1);
    minimumDistance = min(firstDistance,secondDistance);
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


function [safe,minimumDistance] = segmentSetIsSafe( ...
        firstPoint,secondPoint,polygons,clearance_deg,isPeriodic)
    safe = true;
    minimumDistance = Inf;
    for polygonIndex = 1:numel(polygons)
        polygon = preparePolygon(polygons{polygonIndex});
        if isempty(polygon)
            continue
        end
        polygon = alignPolygonToSegment( ...
            polygon,firstPoint,secondPoint,isPeriodic);
        distance = segmentPolygonDistance(firstPoint,secondPoint,polygon);
        minimumDistance = min(minimumDistance,distance);
        if distance <= clearance_deg+1e-10
            safe = false;
            return
        end
    end
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
end


function polygon = alignPolygonToSegment( ...
        polygon,firstPoint,secondPoint,isPeriodic)
    if ~isPeriodic
        return
    end
    segmentCenterAz_deg = 0.5*(firstPoint(1)+secondPoint(1));
    shift = 360*round((segmentCenterAz_deg-mean(polygon(:,1)))/360);
    polygon(:,1) = polygon(:,1)+shift;
end


function polygon = openPolygon(polygon)
    if size(polygon,1) > 1 && norm(polygon(1,:)-polygon(end,:)) <= 1e-10
        polygon = polygon(1:end-1,:);
    end
end


function distance = segmentPolygonDistance(firstPoint,secondPoint,polygon)
    polygon = openPolygon(polygon);
    if isempty(polygon)
        distance = Inf;
        return
    end
    if size(polygon,1) >= 3
        [insideFirst,onFirst] = inpolygon( ...
            firstPoint(1),firstPoint(2),polygon(:,1),polygon(:,2));
        [insideSecond,onSecond] = inpolygon( ...
            secondPoint(1),secondPoint(2),polygon(:,1),polygon(:,2));
        if insideFirst || onFirst || insideSecond || onSecond
            distance = 0;
            return
        end
    end
    if size(polygon,1) == 1
        distance = pointSegmentDistance(polygon(1,:),firstPoint,secondPoint);
        return
    end

    if size(polygon,1) == 2
        edgeStarts = polygon(1,:);
        edgeEnds = polygon(2,:);
    else
        edgeStarts = polygon;
        edgeEnds = polygon([2:end,1],:);
    end
    distance = Inf;
    for edgeIndex = 1:size(edgeStarts,1)
        edgeStart = edgeStarts(edgeIndex,:);
        edgeEnd = edgeEnds(edgeIndex,:);
        if lineSegmentsIntersect(firstPoint,secondPoint,edgeStart,edgeEnd)
            distance = 0;
            return
        end
        edgeDistance = min([ ...
            pointSegmentDistance(firstPoint,edgeStart,edgeEnd), ...
            pointSegmentDistance(secondPoint,edgeStart,edgeEnd), ...
            pointSegmentDistance(edgeStart,firstPoint,secondPoint), ...
            pointSegmentDistance(edgeEnd,firstPoint,secondPoint)]);
        distance = min(distance,edgeDistance);
    end
end


function intersects = lineSegmentsIntersect(a,b,c,d)
    tolerance = 1e-10;
    firstOrientation = orientation2d(a,b,c);
    secondOrientation = orientation2d(a,b,d);
    thirdOrientation = orientation2d(c,d,a);
    fourthOrientation = orientation2d(c,d,b);
    properIntersection = ...
        ((firstOrientation > tolerance && secondOrientation < -tolerance) || ...
        (firstOrientation < -tolerance && secondOrientation > tolerance)) && ...
        ((thirdOrientation > tolerance && fourthOrientation < -tolerance) || ...
        (thirdOrientation < -tolerance && fourthOrientation > tolerance));
    intersects = properIntersection || ...
        (abs(firstOrientation) <= tolerance && pointOnSegment(c,a,b,tolerance)) || ...
        (abs(secondOrientation) <= tolerance && pointOnSegment(d,a,b,tolerance)) || ...
        (abs(thirdOrientation) <= tolerance && pointOnSegment(a,c,d,tolerance)) || ...
        (abs(fourthOrientation) <= tolerance && pointOnSegment(b,c,d,tolerance));
end


function value = orientation2d(a,b,c)
    value = (b(1)-a(1))*(c(2)-a(2))- ...
        (b(2)-a(2))*(c(1)-a(1));
end


function onSegment = pointOnSegment(point,firstPoint,secondPoint,tolerance)
    onSegment = point(1) >= min(firstPoint(1),secondPoint(1))-tolerance && ...
        point(1) <= max(firstPoint(1),secondPoint(1))+tolerance && ...
        point(2) >= min(firstPoint(2),secondPoint(2))-tolerance && ...
        point(2) <= max(firstPoint(2),secondPoint(2))+tolerance;
end


function distance = pointSegmentDistance(point,firstPoint,secondPoint)
    segment = secondPoint-firstPoint;
    lengthSquared = dot(segment,segment);
    if lengthSquared <= eps
        distance = hypot(point(1)-firstPoint(1),point(2)-firstPoint(2));
        return
    end
    fraction = dot(point-firstPoint,segment)/lengthSquared;
    fraction = min(1,max(0,fraction));
    closestPoint = firstPoint+fraction*segment;
    distance = hypot(point(1)-closestPoint(1),point(2)-closestPoint(2));
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
