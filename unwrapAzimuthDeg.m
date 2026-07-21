function unwrapped_deg = unwrapAzimuthDeg(wrapped_deg,reference_deg)
%UNWRAPAZIMUTHDEG Convert a wrapped degree sequence to a continuous branch.

    validateattributes(wrapped_deg,{'numeric'},{'real','finite'});
    unwrapped_deg = rad2deg(unwrap(deg2rad(wrapped_deg)));
    if nargin >= 2 && ~isempty(reference_deg)
        validateattributes(reference_deg,{'numeric'}, ...
            {'scalar','real','finite'});
        firstValue = unwrapped_deg(1);
        unwrapped_deg = unwrapped_deg+360*round( ...
            (reference_deg-firstValue)/360);
    end
end
