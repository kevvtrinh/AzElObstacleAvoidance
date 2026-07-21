function figureHandle = animateAzElQLearningPlan(azElData,result,options)
%ANIMATEAZELQLEARNINGPLAN Public facade for planner animation.
%   The implementation lives in AZEL.VISUALIZATION.ANIMATEAZELQLEARNINGPLAN.

    if nargin < 3
        options = struct;
    end
    figureHandle = azel.visualization.animateAzElQLearningPlan( ...
        azElData,result,options);
end
