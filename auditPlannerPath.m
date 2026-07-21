function audit = auditPlannerPath(scenario,result)
%AUDITPLANNERPATH Independently certify a planner result.
%   AUDIT = AUDITPLANNERPATH(SCENARIO,RESULT) checks raw timed polylines or
%   dispatches analytic commands to the trajectory auditor. The stable
%   facade delegates to AZEL.AUDIT.AUDITPLANNERPATH.

    audit = azel.audit.auditPlannerPath(scenario,result);
end
