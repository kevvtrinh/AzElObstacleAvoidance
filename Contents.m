% AzElObstacleAvoidance
% Version 1.1
%
% Public planning API
%   planAzElQLearning          - Plan and certify an az/el command.
%   buildAzElOccupancy         - Rasterize time-varying obstacle masks.
%   smoothAzElTrajectory       - Convert a timed route into a C2 command.
%   auditPlannerPath           - Certify a raw or analytic planner result.
%   auditAzElTrajectory        - Certify an analytic Bezier trajectory.
%
% Azimuth and obstacle geometry
%   shortestAzimuthDeltaDeg    - Signed shortest angular difference.
%   interpolateAzimuthDeg      - Shortest-arc angle interpolation.
%   unwrapAzimuthDeg           - Convert wrapped samples to one branch.
%   interpolateAzElObstacleFrame - Interpolate moving obstacle contours.
%
% Visualization
%   selectAzElDisplayPath      - Select only an executable display path.
%   plotAzElQLearningResult    - Plot masks and a planner result.
%   animateAzElQLearningPlan   - Animate masks and the command path.
%   createGauntletGif          - Render the compact demonstration GIF.
%   createGauntletSuiteGif     - Render all verified gauntlets in one GIF.
%
% Verification and examples
%   runGauntlet                - Run static, unit, scenario, seam, and smoke checks.
%   runPlannerGauntletSuite    - Run the 18-scenario planner suite.
%   runAzimuthWraparoundGauntlet - Run periodic-azimuth checks.
%   runQLearningAzElExample    - Execute the documented example.
%
% Namespaced implementations are grouped under the +azel package. Keep the
% repository root on the MATLAB path; do not add package folders directly.
