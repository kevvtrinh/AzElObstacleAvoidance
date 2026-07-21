function figureHandle = animateAzElQLearningPlan(azElData,result,options)
%ANIMATEAZELQLEARNINGPLAN Animate the mask against the learned path.
%   OPTIONS.frameStride controls skipped display frames. OPTIONS.gifFile
%   enables GIF output; OPTIONS.gifDelay_s controls GIF playback speed
%   independently of the live animation pause.

    if nargin < 3 || isempty(options)
        options = struct;
    end
    options = applyDefaults(options,struct( ...
        'frameStride',1, ...
        'pause_s',0.02, ...
        'gifFile','', ...
        'gifDelay_s',0.04));

    if isempty(result.path.time_s)
        error('The result contains no successful or partial path to animate.');
    end

    figureHandle = figure('Color','w','Name','Animated Q-learning az/el plan');
    ax = axes(figureHandle);
    hold(ax,'on');
    grid(ax,'on');
    box(ax,'on');
    xlim(ax,result.grid.azLim_deg);
    ylim(ax,result.grid.elLim_deg);
    xlabel(ax,'Mechanical azimuth (deg)');
    ylabel(ax,'Sensor elevation (deg)');

    maskHandle = patch(ax,NaN,NaN,[0.90,0.20,0.15], ...
        'FaceAlpha',0.30,'EdgeColor',[0.70,0.08,0.05], ...
        'LineWidth',1.0,'DisplayName','Mask at current time');
    plot(ax,result.path.az_deg,result.path.el_deg,'-', ...
        'Color',[0.65,0.65,0.65],'LineWidth',1.4, ...
        'DisplayName','Remaining plan');
    executedHandle = plot(ax,NaN,NaN,'b-', ...
        'LineWidth',2.5,'DisplayName','Executed path');
    currentHandle = scatter(ax,result.path.az_deg(1),result.path.el_deg(1), ...
        70,[0.05,0.40,0.95],'filled','DisplayName','Current pointing');
    scatter(ax,result.path.az_deg(1),result.path.el_deg(1),65, ...
        [0.10,0.65,0.20],'filled','DisplayName','Start');
    scatter(ax,result.path.az_deg(end),result.path.el_deg(end),70, ...
        [0.75,0.10,0.75],'filled','DisplayName','Arrival');

    for polygonIndex = 1:numel(result.options.staticPolygons)
        polygon = result.options.staticPolygons{polygonIndex};
        if ~isempty(polygon)
            patch(ax,polygon(:,1),polygon(:,2),[0.35,0.35,0.35], ...
                'FaceAlpha',0.15,'EdgeColor',[0.25,0.25,0.25], ...
                'HandleVisibility','off');
        end
    end
    legend(ax,'Location','best');

    frameIndices = unique([1:options.frameStride:numel(result.path.time_s), ...
        numel(result.path.time_s)]);
    writeGif = strlength(string(options.gifFile)) > 0;
    for outputFrame = 1:numel(frameIndices)
        pathIndex = frameIndices(outputFrame);
        maskIndex = result.path.timeIndex(pathIndex);
        [maskAz_deg,maskEl_deg] = getFrame(azElData,maskIndex);

        set(maskHandle,'XData',maskAz_deg,'YData',maskEl_deg);
        set(executedHandle,'XData',result.path.az_deg(1:pathIndex), ...
            'YData',result.path.el_deg(1:pathIndex));
        set(currentHandle,'XData',result.path.az_deg(pathIndex), ...
            'YData',result.path.el_deg(pathIndex));
        if result.path.isWaiting(pathIndex)
            stateText = 'WAITING';
        else
            stateText = 'SLEWING';
        end
        title(ax,sprintf('t = %.2f s   %s', ...
            result.path.time_s(pathIndex),stateText));

        if writeGif
            drawnow;
            [imageData,colorMap] = rgb2ind(frame2im(getframe(figureHandle)),256);
            if outputFrame == 1
                imwrite(imageData,colorMap,options.gifFile,'gif', ...
                    'LoopCount',Inf,'DelayTime',options.gifDelay_s);
            else
                imwrite(imageData,colorMap,options.gifFile,'gif', ...
                    'WriteMode','append','DelayTime',options.gifDelay_s);
            end
        else
            drawnow limitrate;
        end

        if options.pause_s > 0
            pause(options.pause_s);
        end
    end
end


function options = applyDefaults(options,defaults)
    names = fieldnames(defaults);
    for k = 1:numel(names)
        if ~isfield(options,names{k}) || isempty(options.(names{k}))
            options.(names{k}) = defaults.(names{k});
        end
    end
end


function [az_deg,el_deg] = getFrame(data,timeIndex)
    if iscell(data.az_deg)
        az_deg = data.az_deg{timeIndex};
    else
        az_deg = data.az_deg(timeIndex,:);
    end
    if iscell(data.el_deg)
        el_deg = data.el_deg{timeIndex};
    else
        el_deg = data.el_deg(timeIndex,:);
    end
    az_deg = az_deg(:);
    el_deg = el_deg(:);
end
