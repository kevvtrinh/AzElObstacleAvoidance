function figureHandle = plotAzElQLearningResult(azElData,result,sweepStride)
%PLOTAZELQLEARNINGRESULT Public facade for result plotting.
%   The implementation lives in AZEL.VISUALIZATION.PLOTAZELQLEARNINGRESULT.

    if nargin < 3
        sweepStride = [];
    end
    figureHandle = azel.visualization.plotAzElQLearningResult( ...
        azElData,result,sweepStride);
end
