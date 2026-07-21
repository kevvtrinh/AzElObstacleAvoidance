function result = planAzElQLearning(azElData,startAzEl_deg,goalAzEl_deg,options)
%PLANAZELQLEARNING Plan and certify time-dependent az/el motion.
%   RESULT = PLANAZELQLEARNING(DATA,START,GOAL,OPTIONS) searches a
%   time-expanded azimuth/elevation grid, generates a smooth command, and
%   independently audits it. RESULT.routeSuccess describes the discrete
%   route; when smoothing is requested, RESULT.success additionally requires
%   an executable audited trajectory.
%
%   This stable facade delegates to AZEL.PLANNING.PLANAZELQLEARNING.

    if nargin < 4
        options = struct;
    end
    result = azel.planning.planAzElQLearning( ...
        azElData,startAzEl_deg,goalAzEl_deg,options);
end
