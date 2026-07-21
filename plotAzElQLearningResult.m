function figureHandle = plotAzElQLearningResult(azElData,result,sweepStride)
%PLOTAZELQLEARNINGRESULT Plot the full mask sweep and learned path.

    if nargin < 3 || isempty(sweepStride)
        sweepStride = max(1,ceil(numel(azElData.time_s)/120));
    end

    figureHandle = figure('Color','w','Name','Q-learning az/el result');
    ax = axes(figureHandle);
    hold(ax,'on');
    grid(ax,'on');
    box(ax,'on');

    frameIndices = unique([1:sweepStride:numel(azElData.time_s), ...
        numel(azElData.time_s)]);
    maskLegend = gobjects(1);
    for frameNumber = 1:numel(frameIndices)
        [az_deg,el_deg] = getFrame(azElData,frameIndices(frameNumber));
        if isempty(az_deg)
            continue
        end
        handle = patch(ax,az_deg,el_deg,[0.90,0.25,0.20], ...
            'FaceAlpha',0.018,'EdgeColor',[0.80,0.50,0.48], ...
            'EdgeAlpha',0.12,'HandleVisibility','off');
        if ~isgraphics(maskLegend)
            maskLegend = handle;
        end
    end

    for polygonIndex = 1:numel(result.options.staticPolygons)
        polygon = result.options.staticPolygons{polygonIndex};
        if ~isempty(polygon)
            patch(ax,polygon(:,1),polygon(:,2),[0.35,0.35,0.35], ...
                'FaceAlpha',0.15,'EdgeColor',[0.25,0.25,0.25], ...
                'HandleVisibility','off');
        end
    end

    if isempty(result.path.time_s)
        title(ax,sprintf('Q-learning: %s',result.status));
        xlim(ax,result.grid.azLim_deg);
        ylim(ax,result.grid.elLim_deg);
        xlabel(ax,'Mechanical azimuth (deg)');
        ylabel(ax,'Sensor elevation (deg)');
        return
    end

    plannedPath = plot(ax,result.path.az_deg,result.path.el_deg,'b-', ...
        'LineWidth',2.2,'DisplayName','Learned path');
    waitSamples = result.path.isWaiting;
    if any(waitSamples)
        waitHandle = scatter(ax,result.path.az_deg(waitSamples), ...
            result.path.el_deg(waitSamples),18,[0.05,0.45,0.95],'filled', ...
            'DisplayName','Wait');
    else
        waitHandle = gobjects(1);
    end
    startHandle = scatter(ax,result.path.az_deg(1),result.path.el_deg(1), ...
        70,[0.10,0.65,0.20],'filled','DisplayName','Start');
    goalHandle = scatter(ax,result.path.az_deg(end),result.path.el_deg(end), ...
        75,[0.75,0.10,0.75],'filled','DisplayName','Arrival');

    xlim(ax,result.grid.azLim_deg);
    ylim(ax,result.grid.elLim_deg);
    xlabel(ax,'Mechanical azimuth (deg)');
    ylabel(ax,'Sensor elevation (deg)');
    title(ax,sprintf('Q-learning %s: arrival %.2f s, wait %.2f s', ...
        result.status,result.diagnostic.arrivalTime_s, ...
        result.diagnostic.waitTime_s));

    legendHandles = [plannedPath,startHandle,goalHandle];
    if isgraphics(waitHandle)
        legendHandles = [legendHandles,waitHandle];
    end
    legend(ax,legendHandles,'Location','best');
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
