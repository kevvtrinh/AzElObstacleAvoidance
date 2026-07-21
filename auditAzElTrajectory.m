function audit = auditAzElTrajectory(scenario,trajectory,checkGoalHold)
%AUDITAZELTRAJECTORY Independently certify an analytic C2 command.
%   AUDIT = AUDITAZELTRAJECTORY(SCENARIO,TRAJECTORY,CHECKGOALHOLD) verifies
%   collision clearance, limits, kinematics, continuity, and optional hold.
%   The stable facade delegates to AZEL.AUDIT.AUDITAZELTRAJECTORY.

    if nargin < 3
        checkGoalHold = true;
    end
    audit = azel.audit.auditAzElTrajectory( ...
        scenario,trajectory,checkGoalHold);
end
