function report = runPlannerGauntletSuite(suite,throwOnFailure)
%RUNPLANNERGAUNTLETSUITE Execute and score the full scenario stress suite.

    if nargin < 1 || isempty(suite)
        suite = createPlannerGauntletSuite;
    end
    if nargin < 2
        throwOnFailure = true;
    end

    entryTemplate = struct('name',"",'setup',"",'metric',"", ...
        'expectedSuccess',true,'passed',false,'reason',"", ...
        'result',struct,'scenario',struct,'audit',struct);
    entries = repmat(entryTemplate,numel(suite),1);
    fprintf('=== Planner scenario gauntlet suite ===\n');
    for scenarioIndex = 1:numel(suite)
        scenario = suite(scenarioIndex);
        result = planAzElQLearning(scenario.data, ...
            scenario.startAzEl_deg,scenario.goalAzEl_deg,scenario.options);
        audit = auditPlannerPath(scenario,result);
        [passed,reason] = evaluateScenario(scenario,result,audit);
        entries(scenarioIndex).name = scenario.name;
        entries(scenarioIndex).setup = scenario.setup;
        entries(scenarioIndex).metric = scenario.metric;
        entries(scenarioIndex).expectedSuccess = scenario.expectedSuccess;
        entries(scenarioIndex).passed = passed;
        entries(scenarioIndex).reason = reason;
        entries(scenarioIndex).result = result;
        entries(scenarioIndex).scenario = scenario;
        entries(scenarioIndex).audit = audit;
        fprintf('%s %-24s %s\n',passLabel(passed),scenario.name,reason);
    end

    passed = [entries.passed];
    report = struct;
    report.passed = all(passed);
    report.numPassed = nnz(passed);
    report.numScenarios = numel(entries);
    report.entries = entries;
    if report.passed
        fprintf('ALL %d PLANNER SCENARIOS PASSED\n',numel(entries));
    elseif throwOnFailure
        failedNames = strjoin([entries(~passed).name],', ');
        error('runPlannerGauntletSuite:ScenarioFailure', ...
            'Planner gauntlet failure(s): %s',failedNames);
    end
end


function [passed,reason] = evaluateScenario(scenario,result,audit)
    if ~scenario.expectedSuccess
        routeReachedGoal = isfield(result,'diagnostic') && ...
            isfield(result.diagnostic,'routeReachedGoal') && ...
            result.diagnostic.routeReachedGoal;
        passed = ~routeReachedGoal && ~result.success && audit.collisionFree;
        if routeReachedGoal
            reason = "a raw route reached a supposedly impossible goal";
        elseif result.success
            reason = "incorrectly reported a route";
        elseif ~audit.collisionFree
            reason = "unsafe partial path: " + audit.message;
        else
            reason = "correctly reported unreachable";
        end
        return
    end

    if ~result.success
        passed = false;
        reason = "goal not reached";
        return
    end
    if ~audit.collisionFree
        passed = false;
        reason = "independent audit failed: " + audit.message;
        return
    end
    if ~result.diagnostic.collisionFree
        passed = false;
        reason = "planner collision diagnostic failed";
        return
    end
    if isfield(result.options,'generateSmoothTrajectory') && ...
            result.options.generateSmoothTrajectory
        if ~isfield(result,'trajectory') || ~result.trajectory.success
            passed = false;
            reason = "no dynamically feasible smooth trajectory";
            return
        end
        if ~isfield(audit,'kinematicallyFeasible') || ...
                ~audit.kinematicallyFeasible || ~audit.c2Continuous
            passed = false;
            reason = "smooth trajectory failed its kinematic audit";
            return
        end
    end
    dt_s = result.grid.dt_s;
    rateSafe = all(abs(diff(result.path.az_deg)) <= ...
        scenario.options.azRate_deg_s*dt_s+1e-9) && ...
        all(abs(diff(result.path.el_deg)) <= ...
        scenario.options.elRate_deg_s*dt_s+1e-9);
    if ~rateSafe
        passed = false;
        reason = "slew-rate limit exceeded";
        return
    end

    [passed,reason] = evaluateMetric(scenario,result);
end


function [passed,reason] = evaluateMetric(scenario,result)
    path = result.path;
    passed = true;
    reason = "safe goal arrival";
    switch scenario.metric
        case "narrowHallway"
            passed = max(abs(path.el_deg)) <= 1+1e-9 && ...
                result.diagnostic.turnCountAfterSmoothing <= 2;
            reason = metricReason(passed, ...
                "centered with stable heading","hallway jitter or excess turns");
        case "pistonSlalom"
            passed = result.diagnostic.turnCountAfterSmoothing <= 18;
            reason = metricReason(passed, ...
                "smooth timed weave","excessive slalom heading changes");
        case "crossingTraffic"
            maneuvered = result.diagnostic.waitTime_s > 0 || ...
                max(abs(path.el_deg)) >= 1;
            passed = maneuvered;
            reason = metricReason(passed, ...
                "yielded or used a safe crossing gap", ...
                "no observable crossing-traffic response");
        case "timedGate"
            crossingTime_s = path.time_s(nearestIndex(path.az_deg,0));
            openAtCrossing = crossingTime_s >= 20 && crossingTime_s <= 26;
            passed = openAtCrossing && result.diagnostic.waitTime_s > 0;
            reason = metricReason(passed, ...
                "waited for an open gate interval", ...
                "gate crossing occurred outside its open interval");
        case "sweepingArm"
            maneuvered = result.diagnostic.waitTime_s > 0 || ...
                max(abs(path.el_deg)) >= 1;
            passed = maneuvered;
            reason = metricReason(passed, ...
                "timed passage around sweeping arm", ...
                "no observable sweeping-arm response");
        case "movingTunnel"
            centerEl = 3*sin(2*pi*path.time_s/28);
            passed = all(abs(path.el_deg-centerEl) <= 3.1+1e-9);
            reason = metricReason(passed, ...
                "remained inside moving corridor", ...
                "left the moving feasible region");
        case "shrinkingFunnel"
            aligned = abs(path.el_deg(path.az_deg >= 8));
            passed = ~isempty(aligned) && max(aligned) <= 2+1e-9;
            reason = metricReason(passed, ...
                "aligned before narrow exit","late or unstable funnel alignment");
        case "offsetKeyholes"
            centers = [3,-3,2];
            walls = [-8,0,8];
            errors = zeros(size(walls));
            for wallIndex = 1:numel(walls)
                index = nearestIndex(path.az_deg,walls(wallIndex));
                errors(wallIndex) = abs(path.el_deg(index)-centers(wallIndex));
            end
            passed = all(errors <= 2.2) && ...
                result.diagnostic.turnCountAfterSmoothing <= 14;
            reason = metricReason(passed, ...
                "threaded offset openings smoothly", ...
                "missed a keyhole or zigzagged excessively");
        case "oncomingTraffic"
            avoidance = path.el_deg(abs(path.el_deg) >= 0.75);
            signChanges = countSignChanges(avoidance);
            passed = ~isempty(avoidance) && signChanges == 0;
            reason = metricReason(passed, ...
                "used one consistent avoidance side", ...
                "switched avoidance sides in the narrow lane");
        case "twoWayTrap"
            passed = min(path.el_deg) < -3;
            reason = metricReason(passed, ...
                "rejected the closing branch","entered the future-closed branch");
        case "uShapedTrap"
            passed = max(abs(path.el_deg)) > 7;
            reason = metricReason(passed, ...
                "escaped the U-shaped local minimum", ...
                "did not execute the required U-shaped detour");
        case "timedShortcut"
            passed = result.diagnostic.waitTime_s > 0 && ...
                result.diagnostic.duration_s < 42;
            reason = metricReason(passed, ...
                "waited for faster shortcut", ...
                "failed shortcut timing or took the long route");
        case "doubleGate"
            firstTime_s = firstCrossingTime(path,-5);
            secondTime_s = firstCrossingTime(path,5);
            firstOpen = (firstTime_s >= 9 && firstTime_s <= 15) || ...
                (firstTime_s >= 28 && firstTime_s <= 34);
            secondOpen = secondTime_s >= 18 && secondTime_s <= 26;
            passed = firstOpen && secondOpen && firstTime_s < secondTime_s;
            reason = metricReason(passed, ...
                "crossed both gates in sequence", ...
                "missed a gate timing window");
        case "popupObstacle"
            maneuvered = result.diagnostic.waitTime_s > 0 || ...
                max(abs(path.el_deg)) > 2.5;
            passed = maneuvered;
            reason = metricReason(passed, ...
                "reacted stably to pop-up obstacle", ...
                "no observable pop-up response");
        case "goalInterception"
            timeIndex = path.planningTimeIndex(end);
            goal = scenario.goalAzEl_deg(timeIndex,:);
            passed = hypot(path.az_deg(end)-goal(1), ...
                path.el_deg(end)-goal(2)) <= ...
                max(scenario.options.gridStep_deg)+1e-9;
            reason = metricReason(passed, ...
                "intercepted moving goal", ...
                "arrival did not match moving goal");
        case "rotatingSlot"
            maneuvered = result.diagnostic.waitTime_s > 0 || ...
                max(abs(path.el_deg)) >= 1;
            passed = maneuvered;
            reason = metricReason(passed, ...
                "timed entry through rotating slot", ...
                "no observable rotating-slot response");
        case "clearanceSqueeze"
            passed = max(abs(path.el_deg)) <= 1e-9 && ...
                result.diagnostic.turnCountAfterSmoothing == 0;
            reason = metricReason(passed, ...
                "held exact centerline without jitter", ...
                "clearance squeeze was not centered");
    end
end


function index = nearestIndex(values,target)
    [~,index] = min(abs(values-target));
end


function time_s = firstCrossingTime(path,azimuth_deg)
    index = find(path.az_deg >= azimuth_deg,1,'first');
    if isempty(index)
        time_s = Inf;
    else
        time_s = path.time_s(index);
    end
end


function count = countSignChanges(values)
    if isempty(values)
        count = 0;
        return
    end
    signs = sign(values);
    count = nnz(diff(signs) ~= 0);
end


function reason = metricReason(passed,passText,failText)
    if passed
        reason = string(passText);
    else
        reason = string(failText);
    end
end


function label = passLabel(passed)
    if passed
        label = 'PASS';
    else
        label = 'FAIL';
    end
end
