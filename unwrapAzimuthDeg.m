function unwrapped_deg = unwrapAzimuthDeg(wrapped_deg,reference_deg)
%UNWRAPAZIMUTHDEG Public facade for continuous azimuth unwrapping.
%   The implementation lives in AZEL.GEOMETRY.UNWRAPAZIMUTHDEG.

    if nargin < 2
        reference_deg = [];
    end
    unwrapped_deg = ...
        azel.geometry.unwrapAzimuthDeg(wrapped_deg,reference_deg);
end
