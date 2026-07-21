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
        time_s = azElData.time_s(frameIndices(frameNumber));
        [az_deg,el_deg] = azel.geometry.interpolateAzElObstacleFrame( ...
            azElData,time_s,result.options);
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

    path = azel.visualization.selectAzElDisplayPath(result);
    if isempty(path.time_s)
        title(ax,sprintf('Q-learning: %s',result.status));
        xlim(ax,result.grid.azLim_deg);
        ylim(ax,result.grid.elLim_deg);
        xlabel(ax,'Mechanical azimuth (deg)');
        ylabel(ax,'Sensor elevation (deg)');
        return
    end

    isSmoothCommand = isfield(result,'trajectory') && ...
        isfield(result.trajectory,'success') && result.trajectory.success;
    if isSmoothCommand
        pathLabel = 'Smooth command trajectory';
    else
        pathLabel = 'Planner diagnostic path';
    end
    plannedPath = plot(ax,path.az_deg,path.el_deg,'b-', ...
        'LineWidth',2.2,'DisplayName',pathLabel);
    waitSamples = path.isWaiting;
    if any(waitSamples)
        waitHandle = scatter(ax,path.az_deg(waitSamples), ...
            path.el_deg(waitSamples),18,[0.05,0.45,0.95],'filled', ...
            'DisplayName','Wait');
    else
        waitHandle = gobjects(1);
    end
    startHandle = scatter(ax,path.az_deg(1),path.el_deg(1), ...
        70,[0.10,0.65,0.20],'filled','DisplayName','Start');
    goalHandle = scatter(ax,path.az_deg(end),path.el_deg(end), ...
        75,[0.75,0.10,0.75],'filled','DisplayName','Arrival');

    xlim(ax,result.grid.azLim_deg);
    ylim(ax,result.grid.elLim_deg);
    xlabel(ax,'Mechanical azimuth (deg)');
    ylabel(ax,'Sensor elevation (deg)');
    title(ax,sprintf('Q-learning %s: arrival %.2f s, wait %.2f s', ...
        result.status,result.diagnostic.arrivalTime_s, ...
        displayWaitDuration(result,path)));

    legendHandles = [plannedPath,startHandle,goalHandle];
    if isgraphics(waitHandle)
        legendHandles = [legendHandles,waitHandle];
    end
    legend(ax,legendHandles,'Location','best');
end


function duration_s = displayWaitDuration(result,path)
    duration_s = result.diagnostic.waitTime_s;
    if ~isfield(path,'segments') || isempty(path.segments)
        return
    end
    segments = path.segments;
    isWait = arrayfun(@(segment) string(segment.kind) == "wait",segments);
    duration_s = sum([segments(isWait).endTime_s]- ...
        [segments(isWait).startTime_s]);
end
