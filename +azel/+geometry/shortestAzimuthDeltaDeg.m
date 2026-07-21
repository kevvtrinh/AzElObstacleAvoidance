function delta_deg = shortestAzimuthDeltaDeg(from_deg,to_deg)
%SHORTESTAZIMUTHDELTADEG Signed shortest angular change in degrees.

    validateattributes(from_deg,{'numeric'},{'real','finite'});
    validateattributes(to_deg,{'numeric'},{'real','finite'});
    if ~isequal(size(from_deg),size(to_deg)) && ...
            ~isscalar(from_deg) && ~isscalar(to_deg)
        error('shortestAzimuthDeltaDeg:SizeMismatch', ...
            'Inputs must have matching sizes or one input must be scalar.');
    end
    delta_deg = mod(to_deg-from_deg+180,360)-180;
end
