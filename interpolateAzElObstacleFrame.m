function [az_deg,el_deg] = interpolateAzElObstacleFrame(data,time_s,options)
%INTERPOLATEAZELOBSTACLEFRAME Interpolate moving contours at command time.
%   [AZ,EL] = INTERPOLATEAZELOBSTACLEFRAME(DATA,TIME,OPTIONS) preserves
%   periodic seam geometry. The stable facade delegates to
%   AZEL.GEOMETRY.INTERPOLATEAZELOBSTACLEFRAME.

    if nargin < 3
        options = struct;
    end
    [az_deg,el_deg] = azel.geometry.interpolateAzElObstacleFrame( ...
        data,time_s,options);
end
