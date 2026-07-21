function path = selectAzElDisplayPath(result)
%SELECTAZELDISPLAYPATH Select only a command that is valid for execution.
%   When smooth-command generation was requested, a goal-reaching raw route
%   is never returned as an implicit display fallback after trajectory
%   generation or audit fails. This prevents plots and animations from
%   presenting a sharp planning polyline as though it were executable.

    path = result.path;
    smoothRequested = isfield(result,'options') && ...
        isfield(result.options,'generateSmoothTrajectory') && ...
        result.options.generateSmoothTrajectory;
    trajectoryAvailable = isfield(result,'trajectory') && ...
        isstruct(result.trajectory) && ...
        isfield(result.trajectory,'success') && ...
        result.trajectory.success && ...
        isfield(result.trajectory,'time_s') && ...
        ~isempty(result.trajectory.time_s);
    trajectoryAudited = trajectoryAuditPassed(result);

    if trajectoryAvailable && (~smoothRequested || trajectoryAudited)
        path = result.trajectory;
        return
    end

    if smoothRequested && routeReachedGoal(result)
        path = emptyDisplayPath;
    end
end


function passed = trajectoryAuditPassed(result)
    passed = false;
    if isfield(result,'trajectoryAudit') && ...
            isstruct(result.trajectoryAudit) && ...
            isfield(result.trajectoryAudit,'success')
        passed = logical(result.trajectoryAudit.success);
        return
    end
    if isfield(result,'diagnostic') && ...
            isfield(result.diagnostic,'trajectoryAuditSuccessful')
        passed = logical(result.diagnostic.trajectoryAuditSuccessful);
    end
end


function reached = routeReachedGoal(result)
    reached = false;
    if isfield(result,'routeReachedGoal')
        reached = logical(result.routeReachedGoal);
        return
    elseif isfield(result,'routeSuccess')
        reached = logical(result.routeSuccess);
        return
    end
    if isfield(result,'diagnostic')
        if isfield(result.diagnostic,'routeReachedGoal')
            reached = logical(result.diagnostic.routeReachedGoal);
            return
        end
        if isfield(result.diagnostic,'routeSuccess')
            reached = logical(result.diagnostic.routeSuccess);
            return
        end
    end
    if isfield(result,'success')
        reached = logical(result.success);
    end
end


function path = emptyDisplayPath
    path = struct('planningTimeIndex',zeros(0,1), ...
        'timeIndex',zeros(0,1),'time_s',zeros(0,1), ...
        'az_deg',zeros(0,1),'azWrapped_deg',zeros(0,1), ...
        'el_deg',zeros(0,1),'isWaiting',false(0,1));
end
