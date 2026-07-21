function [blocked,grid,stats] = buildAzElOccupancy(azElData,options)
%BUILDAZELOCCUPANCY Rasterize moving az/el polygons without bounding boxes.
%   BLOCKED is numEl-by-numAz-by-numTimes. Polygon contours are filled with
%   INPOLYGON, and NaNs separate multiple contours in the same time frame.

    % ---------------------------------------------------------------------
    % Validate the obstacle timeline and grid definition
    % ---------------------------------------------------------------------
    requiredFields = {'time_s','az_deg','el_deg'};
    for k = 1:numel(requiredFields)
        if ~isfield(azElData,requiredFields{k})
            error('azElData.%s is required.',requiredFields{k});
        end
    end

    time_s = azElData.time_s(:);
    validateattributes(time_s,{'numeric'}, ...
        {'column','real','finite','nonempty','increasing'});
    if numel(time_s) < 2
        error('azElData.time_s requires at least two samples.');
    end
    dt_s = median(diff(time_s));
    if max(abs(diff(time_s)-dt_s)) > 1e-6*max(1,dt_s)
        error('Q-learning currently requires uniformly spaced azElData.time_s samples.');
    end

    if isscalar(options.gridStep_deg)
        options.gridStep_deg = [options.gridStep_deg,options.gridStep_deg];
    end
    validateattributes(options.gridStep_deg,{'numeric'}, ...
        {'row','numel',2,'positive','finite'});
    validateattributes(options.azLim_deg,{'numeric'}, ...
        {'row','numel',2,'increasing','finite'});
    validateattributes(options.elLim_deg,{'numeric'}, ...
        {'row','numel',2,'increasing','finite'});

    periodicAzimuth = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    azGrid_deg = makeGrid( ...
        options.azLim_deg,options.gridStep_deg(1),periodicAzimuth);
    elGrid_deg = makeGrid(options.elLim_deg,options.gridStep_deg(2),false);
    [AZ_deg,EL_deg] = meshgrid(azGrid_deg,elGrid_deg);
    numTimes = numel(time_s);
    blocked = false(numel(elGrid_deg),numel(azGrid_deg),numTimes);

    if numel(azElData.az_deg) ~= numTimes || ...
            numel(azElData.el_deg) ~= numTimes
        error('azElData az/el polygon histories must match azElData.time_s.');
    end

    % ---------------------------------------------------------------------
    % Rasterize static and per-frame dynamic geometry
    % ---------------------------------------------------------------------
    staticBlocked = false(size(AZ_deg));
    for polygonIndex = 1:numel(options.staticPolygons)
        polygon = options.staticPolygons{polygonIndex};
        if isempty(polygon)
            continue
        end
        validateattributes(polygon,{'numeric'}, ...
            {'2d','ncols',2,'real','finite'});
        staticBlocked = staticBlocked | rasterizeContoursWithClearance( ...
            polygon(:,1),polygon(:,2),AZ_deg,EL_deg, ...
            options.clearance_deg);
    end
    for timeIndex = 1:numTimes
        [polygonAz_deg,polygonEl_deg] = getFrame(azElData,timeIndex);
        blocked(:,:,timeIndex) = staticBlocked | ...
            rasterizeContoursWithClearance(polygonAz_deg,polygonEl_deg, ...
            AZ_deg,EL_deg,options.clearance_deg);
    end

    % Temporal padding unions neighboring frames into each occupancy slice.
    % This intentionally over-approximates timing uncertainty at grid level;
    % the analytic auditor later applies a continuous-time counterpart.
    paddingSteps = max(0,ceil(options.temporalPadding_s/dt_s-1e-12));
    if paddingSteps > 0
        originalBlocked = blocked;
        for offset = 1:paddingSteps
            blocked(:,:,1+offset:end) = blocked(:,:,1+offset:end) | ...
                originalBlocked(:,:,1:end-offset);
            blocked(:,:,1:end-offset) = blocked(:,:,1:end-offset) | ...
                originalBlocked(:,:,1+offset:end);
        end
    end

    grid = struct;
    grid.az_deg = azGrid_deg;
    grid.el_deg = elGrid_deg;
    grid.time_s = time_s;
    grid.dt_s = dt_s;
    grid.azLim_deg = [azGrid_deg(1),azGrid_deg(end)];
    grid.elLim_deg = [elGrid_deg(1),elGrid_deg(end)];
    if periodicAzimuth
        grid.azLim_deg = options.azLim_deg;
        grid.azimuthTopology = "periodic";
    else
        grid.azimuthTopology = "mechanical";
    end

    stats = struct;
    stats.occupiedFraction = nnz(blocked)/numel(blocked);
    stats.numOccupiedSamples = nnz(blocked);
    stats.numSamples = numel(blocked);
end


% =========================================================================
% Grid and frame helpers
% =========================================================================
function values = makeGrid(limits,step,isPeriodic)
    if isPeriodic
        numValues = round(diff(limits)/step);
        values = limits(1)+(0:numValues-1)*step;
        return
    end
    values = limits(1):step:limits(2);
    if values(end) < limits(2)-1e-12
        values(end+1) = limits(2);
    end
end


function [az_deg,el_deg] = getFrame(data,timeIndex)
    if iscell(data.az_deg)
        az_deg = data.az_deg{timeIndex};
    else
        az_deg = data.az_deg(timeIndex,:);
    end
    if iscell(data.el_deg)
        el_deg = data.el_deg{timeIndex};
    else
        el_deg = data.el_deg(timeIndex,:);
    end
    az_deg = az_deg(:);
    el_deg = el_deg(:);
    if numel(az_deg) ~= numel(el_deg)
        error('Azimuth and elevation polygon vectors differ at frame %d.',timeIndex);
    end
end


% =========================================================================
% Seam-aware contour rasterization
% =========================================================================
function occupied = rasterizeContours(az_deg,el_deg,AZ_deg,EL_deg)
    occupied = false(size(AZ_deg));
    if isempty(az_deg)
        return
    end

    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators;true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;
    for contourIndex = 1:numel(starts)
        contourAz_deg = az_deg(starts(contourIndex):ends(contourIndex));
        contourEl_deg = el_deg(starts(contourIndex):ends(contourIndex));
        valid = isfinite(contourAz_deg) & isfinite(contourEl_deg);
        contourAz_deg = contourAz_deg(valid);
        contourEl_deg = contourEl_deg(valid);
        if numel(contourAz_deg) < 3
            continue
        end

        contourAz_deg = rad2deg(unwrap(deg2rad(contourAz_deg)));
        % Unwrap each contour locally, then test enough 360-degree copies
        % to cover the configured grid. This prevents a seam-crossing
        % polygon from turning into a false map-spanning obstacle.
        repeatCount = ceil((max(abs(AZ_deg(:)))+ ...
            max(abs(contourAz_deg)))/360)+1;
        for wrapIndex = -repeatCount:repeatCount
            occupied = occupied | inpolygon(AZ_deg+360*wrapIndex,EL_deg, ...
                contourAz_deg,contourEl_deg);
        end
    end
end


function occupied = rasterizeContoursWithClearance( ...
        az_deg,el_deg,AZ_deg,EL_deg,clearance_deg)
    occupied = rasterizeContours(az_deg,el_deg,AZ_deg,EL_deg);
    clearance_deg = max(clearance_deg(:));
    if isempty(az_deg) || clearance_deg <= 0
        return
    end

    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators(:);true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;
    for contourIndex = 1:numel(starts)
        contourAz_deg = az_deg(starts(contourIndex):ends(contourIndex));
        contourEl_deg = el_deg(starts(contourIndex):ends(contourIndex));
        valid = isfinite(contourAz_deg) & isfinite(contourEl_deg);
        contourAz_deg = rad2deg(unwrap(deg2rad(contourAz_deg(valid))));
        contourEl_deg = contourEl_deg(valid);
        if numel(contourAz_deg) < 2
            continue
        end
        repeatCount = ceil((max(abs(AZ_deg(:)))+ ...
            max(abs(contourAz_deg)))/360)+1;
        for wrapIndex = -repeatCount:repeatCount
            shiftedAz_deg = contourAz_deg-360*wrapIndex;
            if hypot(shiftedAz_deg(1)-shiftedAz_deg(end), ...
                    contourEl_deg(1)-contourEl_deg(end)) > 1e-12
                edgeStartAz = shiftedAz_deg;
                edgeStartEl = contourEl_deg;
                edgeEndAz = shiftedAz_deg([2:end,1]);
                edgeEndEl = contourEl_deg([2:end,1]);
            else
                edgeStartAz = shiftedAz_deg(1:end-1);
                edgeStartEl = contourEl_deg(1:end-1);
                edgeEndAz = shiftedAz_deg(2:end);
                edgeEndEl = contourEl_deg(2:end);
            end
            for edgeIndex = 1:numel(edgeStartAz)
                dAz = edgeEndAz(edgeIndex)-edgeStartAz(edgeIndex);
                dEl = edgeEndEl(edgeIndex)-edgeStartEl(edgeIndex);
                lengthSquared = dAz^2+dEl^2;
                if lengthSquared <= eps
                    distance = hypot(AZ_deg-edgeStartAz(edgeIndex), ...
                        EL_deg-edgeStartEl(edgeIndex));
                else
                    fraction = ((AZ_deg-edgeStartAz(edgeIndex))*dAz+ ...
                        (EL_deg-edgeStartEl(edgeIndex))*dEl)/lengthSquared;
                    fraction = min(1,max(0,fraction));
                    distance = hypot( ...
                        AZ_deg-(edgeStartAz(edgeIndex)+fraction*dAz), ...
                        EL_deg-(edgeStartEl(edgeIndex)+fraction*dEl));
                end
                occupied = occupied | distance <= clearance_deg+1e-10;
            end
        end
    end
end
