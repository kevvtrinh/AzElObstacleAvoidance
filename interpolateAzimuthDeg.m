function azimuth_deg = interpolateAzimuthDeg(from_deg,to_deg,fraction)
%INTERPOLATEAZIMUTHDEG Public facade for shortest-arc interpolation.
%   The implementation lives in AZEL.GEOMETRY.INTERPOLATEAZIMUTHDEG.

    azimuth_deg = azel.geometry.interpolateAzimuthDeg( ...
        from_deg,to_deg,fraction);
end
