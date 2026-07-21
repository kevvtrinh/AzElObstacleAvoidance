function [az_deg,el_deg] = interpolateAzElObstacleFrame(data,time_s,options)
%INTERPOLATEAZELOBSTACLEFRAME Interpolate obstacle contours at one time.
%   Corresponding vertices use shortest-arc azimuth interpolation. If the
%   contour topology changes between frames, both endpoint geometries are
%   returned so the visualization does not hide either obstacle state.
%   OPTIONS enables seam-safe periodic copies when azimuthTopology is
%   "periodic" and supplies the visible azLim_deg.

    if nargin < 3 || isempty(options)
        options = struct;
    end

    dataTime_s = data.time_s(:);
    validateattributes(time_s,{'numeric'}, ...
        {'scalar','real','finite'},mfilename,'time_s');
    if isempty(dataTime_s)
        az_deg = zeros(0,1);
        el_deg = zeros(0,1);
        return
    end

    if time_s <= dataTime_s(1)
        [az_deg,el_deg] = getFrame(data,1);
        [az_deg,el_deg] = prepareForDisplay(az_deg,el_deg,options);
        return
    elseif time_s >= dataTime_s(end)
        [az_deg,el_deg] = getFrame(data,numel(dataTime_s));
        [az_deg,el_deg] = prepareForDisplay(az_deg,el_deg,options);
        return
    end

    firstIndex = find(dataTime_s <= time_s,1,'last');
    if abs(time_s-dataTime_s(firstIndex)) <= ...
            1e-12*max(1,abs(time_s))
        [az_deg,el_deg] = getFrame(data,firstIndex);
        [az_deg,el_deg] = prepareForDisplay(az_deg,el_deg,options);
        return
    end
    secondIndex = firstIndex+1;
    fraction = (time_s-dataTime_s(firstIndex))/ ...
        (dataTime_s(secondIndex)-dataTime_s(firstIndex));
    [firstAz_deg,firstEl_deg] = getFrame(data,firstIndex);
    [secondAz_deg,secondEl_deg] = getFrame(data,secondIndex);

    firstSeparators = isnan(firstAz_deg) | isnan(firstEl_deg);
    secondSeparators = isnan(secondAz_deg) | isnan(secondEl_deg);
    sameTopology = isequal(size(firstAz_deg),size(secondAz_deg)) && ...
        isequal(firstSeparators,secondSeparators);
    if sameTopology
        az_deg = NaN(size(firstAz_deg));
        el_deg = NaN(size(firstEl_deg));
        finite = ~firstSeparators & ~secondSeparators & ...
            isfinite(firstAz_deg) & isfinite(firstEl_deg) & ...
            isfinite(secondAz_deg) & isfinite(secondEl_deg);
        az_deg(finite) = firstAz_deg(finite)+fraction.* ...
            shortestAzimuthDeltaDeg(firstAz_deg(finite),secondAz_deg(finite));
        el_deg(finite) = firstEl_deg(finite)+fraction.* ...
            (secondEl_deg(finite)-firstEl_deg(finite));
        [az_deg,el_deg] = prepareForDisplay(az_deg,el_deg,options);
        return
    end

    [az_deg,el_deg] = combineFrames( ...
        firstAz_deg,firstEl_deg,secondAz_deg,secondEl_deg);
    [az_deg,el_deg] = prepareForDisplay(az_deg,el_deg,options);
end


function [az_deg,el_deg] = getFrame(data,index)
    if iscell(data.az_deg)
        az_deg = data.az_deg{index};
    else
        az_deg = data.az_deg(index,:);
    end
    if iscell(data.el_deg)
        el_deg = data.el_deg{index};
    else
        el_deg = data.el_deg(index,:);
    end
    az_deg = az_deg(:);
    el_deg = el_deg(:);
end


function [az_deg,el_deg] = combineFrames( ...
        firstAz_deg,firstEl_deg,secondAz_deg,secondEl_deg)
    firstValid = any(isfinite(firstAz_deg) & isfinite(firstEl_deg));
    secondValid = any(isfinite(secondAz_deg) & isfinite(secondEl_deg));
    if firstValid && secondValid
        az_deg = [firstAz_deg;NaN;secondAz_deg];
        el_deg = [firstEl_deg;NaN;secondEl_deg];
    elseif firstValid
        az_deg = firstAz_deg;
        el_deg = firstEl_deg;
    elseif secondValid
        az_deg = secondAz_deg;
        el_deg = secondEl_deg;
    else
        az_deg = zeros(0,1);
        el_deg = zeros(0,1);
    end
end


function [displayAz_deg,displayEl_deg] = ...
        prepareForDisplay(az_deg,el_deg,options)
    periodic = isfield(options,'azimuthTopology') && ...
        strcmpi(string(options.azimuthTopology),"periodic");
    if ~periodic || isempty(az_deg)
        displayAz_deg = az_deg;
        displayEl_deg = el_deg;
        return
    end
    if isfield(options,'azLim_deg') && numel(options.azLim_deg) == 2
        azLim_deg = options.azLim_deg(:)';
    else
        azLim_deg = [-180,180];
    end

    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators(:);true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;
    displayAz_deg = zeros(0,1);
    displayEl_deg = zeros(0,1);
    for contourIndex = 1:numel(starts)
        indices = starts(contourIndex):ends(contourIndex);
        valid = isfinite(az_deg(indices)) & isfinite(el_deg(indices));
        contourAz_deg = az_deg(indices(valid));
        contourEl_deg = el_deg(indices(valid));
        if isempty(contourAz_deg)
            continue
        end
        contourAz_deg = unwrapAzimuthDeg(contourAz_deg);
        firstCopy = ceil((azLim_deg(1)-max(contourAz_deg))/360);
        lastCopy = floor((azLim_deg(2)-min(contourAz_deg))/360);
        for copyIndex = firstCopy:lastCopy
            if ~isempty(displayAz_deg)
                displayAz_deg(end+1,1) = NaN; %#ok<AGROW>
                displayEl_deg(end+1,1) = NaN; %#ok<AGROW>
            end
            displayAz_deg = [displayAz_deg; ...
                contourAz_deg+360*copyIndex]; %#ok<AGROW>
            displayEl_deg = [displayEl_deg;contourEl_deg]; %#ok<AGROW>
        end
    end
end
