function azElData = createQLearningExampleAzElData(time_s)
%CREATEQLEARNINGEXAMPLEAZELDATA Make irregular masks for the Q-learning demo.
%   The first contour is a deforming country-like outline. A second wavy
%   exclusion curtain temporarily spans the elevation range, forcing the policy
%   to wait until a safe interval opens.

    time_s = time_s(:);
    numTimes = numel(time_s);

    % Slender, asymmetric outline so the rasterized mask is visibly not a
    % bounding box. Coordinates are local [az el] offsets in degrees.
    countryShape = [ ...
        -4.5, 17.0;
        -1.0, 14.0;
         0.0, 10.0;
        -2.0,  6.0;
        -1.0,  2.0;
         3.5, -1.0;
         5.0, -5.0;
         3.0, -9.0;
         0.5,-13.5;
        -3.5,-17.0;
        -6.0,-13.0;
        -5.0, -8.0;
        -3.0, -4.0;
        -5.0,  0.0;
        -4.0,  5.0;
        -6.0, 10.5;
        -4.5, 17.0];

    azCells = cell(numTimes,1);
    elCells = cell(numTimes,1);
    status = strings(numTimes,1);

    for timeIndex = 1:numTimes
        t = time_s(timeIndex)-time_s(1);
        scaleAz = 1.0+0.25*sin(2*pi*t/45);
        scaleEl = 1.0+0.15*cos(2*pi*t/38);
        rotation_deg = 8*sin(2*pi*t/55);
        rotation = [cosd(rotation_deg),-sind(rotation_deg); ...
                    sind(rotation_deg), cosd(rotation_deg)];
        shape = (rotation*[scaleAz*countryShape(:,1), ...
            scaleEl*countryShape(:,2)]')';
        shape(:,1) = shape(:,1)+10+8*sin(2*pi*t/70);
        shape(:,2) = shape(:,2)+48+4*cos(2*pi*t/60);

        % This time-limited wavy strip divides the grid and demonstrates
        % the learned wait action. It is removed after 28 seconds.
        if t <= 28
            curtain = [ ...
                -5.0,  5.0;
                -3.5, 20.0;
                -5.5, 35.0;
                -3.0, 50.0;
                -5.0, 67.0;
                -4.0, 85.0;
                 4.0, 85.0;
                 5.0, 68.0;
                 3.0, 52.0;
                 5.5, 36.0;
                 3.5, 20.0;
                 5.0,  5.0;
                -5.0,  5.0];
            azCells{timeIndex} = [shape(:,1);NaN;curtain(:,1)];
            elCells{timeIndex} = [shape(:,2);NaN;curtain(:,2)];
        else
            azCells{timeIndex} = shape(:,1);
            elCells{timeIndex} = shape(:,2);
        end

        status(timeIndex) = "Visible";
    end

    azElData = struct;
    azElData.targetName = 'Irregular moving keep-out masks';
    azElData.time_s = time_s;
    azElData.az_deg = azCells;
    azElData.el_deg = elCells;
    azElData.status = status;
end
