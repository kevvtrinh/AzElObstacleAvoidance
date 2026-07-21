function path = selectAzElDisplayPath(result)
%SELECTAZELDISPLAYPATH Select the executable path safe to present.
%   PATH = SELECTAZELDISPLAYPATH(RESULT) never substitutes an unaudited raw
%   route for a requested smooth command. The stable facade delegates to
%   AZEL.VISUALIZATION.SELECTAZELDISPLAYPATH.

    path = azel.visualization.selectAzElDisplayPath(result);
end
