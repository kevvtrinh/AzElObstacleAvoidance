function azimuth_deg = interpolateAzimuthDeg(from_deg,to_deg,fraction)
%INTERPOLATEAZIMUTHDEG Interpolate along the shortest wrapped angular arc.

    validateattributes(fraction,{'numeric'},{'real','finite'});
    azimuth_deg = from_deg+fraction.* ...
        shortestAzimuthDeltaDeg(from_deg,to_deg);
end
