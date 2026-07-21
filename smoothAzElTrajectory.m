function [trajectory,info] = smoothAzElTrajectory(path,data,options)
%SMOOTHAZELTRAJECTORY Build a fixed-time, collision-checked C2 command.
%   [TRAJECTORY,INFO] = SMOOTHAZELTRAJECTORY(PATH,DATA,OPTIONS) rounds
%   velocity discontinuities with quintic Bezier blends. The stable facade
%   delegates to AZEL.TRAJECTORY.SMOOTHAZELTRAJECTORY.

    [trajectory,info] = ...
        azel.trajectory.smoothAzElTrajectory(path,data,options);
end
