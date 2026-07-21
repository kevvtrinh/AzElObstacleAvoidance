function report = runAzimuthWraparoundGauntlet(throwOnFailure)
%RUNAZIMUTHWRAPAROUNDGAUNTLET Run the seven azimuth seam checks.
%   REPORT = RUNAZIMUTHWRAPAROUNDGAUNTLET executes deterministic planner
%   and angle-math checks for the -180/180 degree seam. REPORT.ENTRIES is
%   suitable for both command-line reporting and generic GIF rendering.

    if nargin < 1
        throwOnFailure = true;
    end
    validateattributes(throwOnFailure,{'logical','numeric'}, ...
        {'scalar','binary'},mfilename,'throwOnFailure');

    entries = repmat(entryTemplate,7,1);
    entries(1) = plannerCrossingEntry( ...
        "Short seam crossing",[179,30],[-179,30],2);
    entries(2) = plannerCrossingEntry( ...
        "Reverse seam crossing",[-179,30],[179,30],-2);
    entries(3) = interpolationEntry;
    entries(4) = slewRateEntry;
    entries(5) = accelerationEntry;
    entries(6) = equivalentEndpointsEntry;
    entries(7) = multiCrossingEntry;

    fprintf('=== Azimuth wraparound gauntlet ===\n');
    for entryIndex = 1:numel(entries)
        fprintf('%s %-28s %s\n',passLabel(entries(entryIndex).passed), ...
            entries(entryIndex).name,entries(entryIndex).observed);
    end

    passed = [entries.passed];
    report = struct;
    report.entries = entries;
    report.numPassed = nnz(passed);
    report.numTests = numel(entries);
    report.passed = all(passed);
    if report.passed
        fprintf('ALL %d AZIMUTH WRAPAROUND CHECKS PASSED\n',report.numTests);
    elseif throwOnFailure
        failedNames = strjoin([entries(~passed).name],', ');
        error('runAzimuthWraparoundGauntlet:CheckFailure', ...
            'Azimuth wraparound failure(s): %s',failedNames);
    end
end


function entry = plannerCrossingEntry(name,startAzEl_deg,goalAzEl_deg, ...
        expectedNetChange_deg)
    entry = entryTemplate;
    entry.name = name;
    entry.example = string(sprintf('Start [%g, %g], goal [%g, %g]', ...
        startAzEl_deg,goalAzEl_deg));
    entry.expected = "Travel 2 deg across the seam, not 358 deg";
    try
        data = emptyData((0:1:5)');
        result = planAzElQLearning( ...
            data,startAzEl_deg,goalAzEl_deg,periodicPlannerOptions);
        pathAzimuth_deg = result.path.az_deg;
        if isempty(pathAzimuth_deg)
            travel_deg = NaN;
            netChange_deg = NaN;
            maxStep_deg = NaN;
            arrivalError_deg = Inf;
        else
            travel_deg = sum(abs(diff(pathAzimuth_deg)));
            netChange_deg = pathAzimuth_deg(end)-pathAzimuth_deg(1);
            maxStep_deg = maxOrZero(abs(diff(pathAzimuth_deg)));
            arrivalError_deg = abs(shortestAzimuthDeltaDeg( ...
                pathAzimuth_deg(end),goalAzEl_deg(1)));
        end
        tolerance = 1e-9;
        entry.passed = result.success && result.diagnostic.collisionFree && ...
            abs(travel_deg-2) <= tolerance && ...
            abs(netChange_deg-expectedNetChange_deg) <= tolerance && ...
            maxStep_deg <= result.options.azRate_deg_s*result.grid.dt_s+ ...
            tolerance && arrivalError_deg <= tolerance;
        entry.values = struct( ...
            'travel_deg',travel_deg, ...
            'netChange_deg',netChange_deg, ...
            'duration_s',result.diagnostic.duration_s, ...
            'maxStep_deg',maxStep_deg, ...
            'arrivalError_deg',arrivalError_deg, ...
            'pathAzimuth_deg',pathAzimuth_deg, ...
            'pathWrappedAzimuth_deg',result.path.azWrapped_deg);
        entry.observed = string(sprintf( ...
            'travel %.3f deg, duration %.3f s, max step %.3f deg', ...
            travel_deg,result.diagnostic.duration_s,maxStep_deg));
        entry.result = result;
    catch exception
        entry.observed = "ERROR: " + string(exception.message);
        entry.values = struct('errorIdentifier',string(exception.identifier));
    end
end


function entry = interpolationEntry
    entry = entryTemplate;
    entry.name = "Seam interpolation";
    entry.example = "Interpolate from 179 deg to -179 deg";
    entry.expected = "Midpoint is near 180 deg, not 0 deg";
    fractions = [0,0.5,1];
    samples_deg = interpolateAzimuthDeg(179,-179,fractions);
    midpoint_deg = samples_deg(2);
    arcLength_deg = sum(abs(diff(samples_deg)));
    entry.passed = abs(midpoint_deg-180) <= 1e-12 && ...
        abs(arcLength_deg-2) <= 1e-12;
    entry.values = struct('fractions',fractions, ...
        'samples_deg',samples_deg,'midpoint_deg',midpoint_deg, ...
        'arcLength_deg',arcLength_deg);
    entry.observed = string(sprintf( ...
        'samples [%g, %g, %g] deg; midpoint %.3f deg', ...
        samples_deg,midpoint_deg));
end


function entry = slewRateEntry
    entry = entryTemplate;
    entry.name = "Seam slew rate";
    entry.example = "Sequence 178, 179, -180, -179 deg at 1 s intervals";
    entry.expected = "Constant 1 deg/s with no seam velocity spike";
    wrapped_deg = [178,179,-180,-179];
    dt_s = 1;
    continuous_deg = unwrapAzimuthDeg(wrapped_deg,wrapped_deg(1));
    velocity_deg_s = diff(continuous_deg)/dt_s;
    rawVelocity_deg_s = diff(wrapped_deg)/dt_s;
    maxVelocityError_deg_s = max(abs(velocity_deg_s-1));
    entry.passed = maxVelocityError_deg_s <= 1e-12;
    entry.values = struct('wrapped_deg',wrapped_deg, ...
        'continuous_deg',continuous_deg, ...
        'velocity_deg_s',velocity_deg_s, ...
        'rawVelocity_deg_s',rawVelocity_deg_s, ...
        'maxVelocityError_deg_s',maxVelocityError_deg_s);
    entry.observed = string(sprintf( ...
        'seam-aware max rate %.3f deg/s; raw wrapped max %.3f deg/s', ...
        max(abs(velocity_deg_s)),max(abs(rawVelocity_deg_s))));
end


function entry = accelerationEntry
    entry = entryTemplate;
    entry.name = "Seam acceleration";
    entry.example = "Sequence 178, 179, -180, -179 deg at fixed timestep";
    entry.expected = "Acceleration and jerk remain nearly zero";
    wrapped_deg = [178,179,-180,-179];
    dt_s = 1;
    continuous_deg = unwrapAzimuthDeg(wrapped_deg,wrapped_deg(1));
    velocity_deg_s = diff(continuous_deg)/dt_s;
    acceleration_deg_s2 = diff(velocity_deg_s)/dt_s;
    jerk_deg_s3 = diff(acceleration_deg_s2)/dt_s;
    maxAcceleration_deg_s2 = maxOrZero(abs(acceleration_deg_s2));
    maxJerk_deg_s3 = maxOrZero(abs(jerk_deg_s3));
    entry.passed = maxAcceleration_deg_s2 <= 1e-12 && ...
        maxJerk_deg_s3 <= 1e-12;
    entry.values = struct('continuous_deg',continuous_deg, ...
        'velocity_deg_s',velocity_deg_s, ...
        'acceleration_deg_s2',acceleration_deg_s2, ...
        'jerk_deg_s3',jerk_deg_s3, ...
        'maxAcceleration_deg_s2',maxAcceleration_deg_s2, ...
        'maxJerk_deg_s3',maxJerk_deg_s3);
    entry.observed = string(sprintf( ...
        'max |acceleration| %.3g deg/s^2; max |jerk| %.3g deg/s^3', ...
        maxAcceleration_deg_s2,maxJerk_deg_s3));
end


function entry = equivalentEndpointsEntry
    entry = entryTemplate;
    entry.name = "Equivalent endpoints";
    entry.example = "Goal 180 deg versus -180 deg";
    entry.expected = "Planner treats both endpoint representations identically";
    try
        data = emptyData((0:1:12)');
        options = periodicPlannerOptions;
        positiveResult = planAzElQLearning(data,[170,30],[180,30],options);
        negativeResult = planAzElQLearning(data,[170,30],[-180,30],options);
        endpointDelta_deg = shortestAzimuthDeltaDeg(180,-180);
        pathsIdentical = isequal(positiveResult.path.az_deg, ...
            negativeResult.path.az_deg) && ...
            isequal(positiveResult.path.el_deg,negativeResult.path.el_deg);
        durationsEqual = abs(positiveResult.diagnostic.duration_s- ...
            negativeResult.diagnostic.duration_s) <= 1e-12;
        entry.passed = positiveResult.success && negativeResult.success && ...
            endpointDelta_deg == 0 && pathsIdentical && durationsEqual;
        entry.values = struct('endpointDelta_deg',endpointDelta_deg, ...
            'pathsIdentical',pathsIdentical, ...
            'positiveDuration_s',positiveResult.diagnostic.duration_s, ...
            'negativeDuration_s',negativeResult.diagnostic.duration_s, ...
            'positivePathAzimuth_deg',positiveResult.path.az_deg, ...
            'negativePathAzimuth_deg',negativeResult.path.az_deg);
        entry.observed = string(sprintf( ...
            'angular difference %.3f deg; paths identical %d; durations %.3f/%.3f s', ...
            endpointDelta_deg,pathsIdentical, ...
            positiveResult.diagnostic.duration_s, ...
            negativeResult.diagnostic.duration_s));
        entry.result = struct('positive180',positiveResult, ...
            'negative180',negativeResult);
    catch exception
        entry.observed = "ERROR: " + string(exception.message);
        entry.values = struct('errorIdentifier',string(exception.identifier));
    end
end


function entry = multiCrossingEntry
    entry = entryTemplate;
    entry.name = "Multi-crossing path";
    entry.example = "Target repeatedly crosses the -180/180 deg seam";
    entry.expected = "Continuous unwrapped tracking without 358 deg jumps";
    wrapped_deg = [178,179,-180,-179,-180,179,178,179, ...
        -180,-179,-180,179,178];
    expectedContinuous_deg = [178,179,180,181,180,179,178,179, ...
        180,181,180,179,178];
    continuous_deg = unwrapAzimuthDeg(wrapped_deg,wrapped_deg(1));
    continuousStep_deg = diff(continuous_deg);
    rawStep_deg = diff(wrapped_deg);
    seamCrossings = nnz(abs(rawStep_deg) > 180);
    equivalenceError_deg = max(abs(shortestAzimuthDeltaDeg( ...
        wrapped_deg,continuous_deg)));
    maxContinuousStep_deg = maxOrZero(abs(continuousStep_deg));
    sequenceError_deg = max(abs(continuous_deg-expectedContinuous_deg));
    entry.passed = sequenceError_deg <= 1e-12 && ...
        seamCrossings >= 4 && maxContinuousStep_deg <= 1+1e-12 && ...
        equivalenceError_deg <= 1e-12;
    entry.values = struct('wrapped_deg',wrapped_deg, ...
        'continuous_deg',continuous_deg, ...
        'continuousStep_deg',continuousStep_deg, ...
        'seamCrossings',seamCrossings, ...
        'maxContinuousStep_deg',maxContinuousStep_deg, ...
        'maxRawStep_deg',maxOrZero(abs(rawStep_deg)), ...
        'sequenceError_deg',sequenceError_deg, ...
        'equivalenceError_deg',equivalenceError_deg);
    entry.observed = string(sprintf( ...
        '%d seam crossings; max continuous step %.3f deg versus raw %.3f deg', ...
        seamCrossings,maxContinuousStep_deg,maxOrZero(abs(rawStep_deg))));
end


function entry = entryTemplate
    entry = struct('name',"",'example',"",'expected',"", ...
        'observed',"",'passed',false,'values',struct,'result',struct);
end


function options = periodicPlannerOptions
    options = struct;
    options.azLim_deg = [-180,180];
    options.elLim_deg = [29,31];
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 1;
    options.elRate_deg_s = 1;
    options.azimuthTopology = "periodic";
    options.goalAzimuthIsWrapped = true;
    options.episodes = 1;
    options.minimumEpisodesPerLearner = 1;
    options.earlyStopSuccessStreak = 1;
    options.guidedExplorationProbability = 1;
    options.useParallel = false;
    options.randomSeed = 73;
    options.smoothPath = false;
    % These seam tests intentionally exercise a one-degree-per-second
    % constant-rate command with no time available for endpoint ramps.
    options.trajectoryRequireEndpointRest = false;
    options.verbose = false;
end


function data = emptyData(time_s)
    time_s = time_s(:);
    data = struct;
    data.targetName = 'Azimuth wraparound gauntlet';
    data.time_s = time_s;
    data.az_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.el_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.status = repmat("NotVisible",numel(time_s),1);
end


function value = maxOrZero(values)
    if isempty(values)
        value = 0;
    else
        value = max(values);
    end
end


function label = passLabel(passed)
    if passed
        label = 'PASS';
    else
        label = 'FAIL';
    end
end
