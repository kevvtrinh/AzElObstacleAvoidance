function [blocked,grid,stats] = buildAzElOccupancy(azElData,options)
%BUILDAZELOCCUPANCY Rasterize time-varying az/el keep-out polygons.
%   [BLOCKED,GRID,STATS] = BUILDAZELOCCUPANCY(DATA,OPTIONS) applies
%   clearance, temporal padding, and periodic seam handling. The stable
%   facade delegates to AZEL.MAPPING.BUILDAZELOCCUPANCY.

    [blocked,grid,stats] = ...
        azel.mapping.buildAzElOccupancy(azElData,options);
end
