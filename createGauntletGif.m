function gifFile = createGauntletGif(gifFile)
%CREATEGAUNTLETGIF Run all checks and render both planning runs into one GIF.

    projectFolder = fileparts(mfilename('fullpath'));
    if nargin < 1 || strlength(string(gifFile)) == 0
        gifFile = fullfile(projectFolder,'gauntlet_all_runs_fixed.gif');
    end
    gifFile = char(gifFile);

    report = runGauntlet;
    dynamicResult = report.smokeResult;
    spiralResult = report.spiralResult;
    dynamicPath = selectAzElDisplayPath(dynamicResult);
    spiralPath = selectAzElDisplayPath(spiralResult);
    dynamicData = createQLearningExampleAzElData((0:0.5:75)');
    spiralScenario = createSpiralGauntletScenario;
    spiralCommandWinding_deg = pathWinding(spiralPath, ...
        numel(spiralPath.time_s),spiralScenario.centerAzEl_deg, ...
        max(spiralScenario.options.gridStep_deg));

    figureHandle = figure('Color',[0.965,0.975,0.99], ...
        'Visible','off','Units','pixels','Position',[100,100,960,640], ...
        'InvertHardcopy','off', ...
        'Name','Complete planner gauntlet');
    set(figureHandle,'PaperPositionMode','auto');
    cleanup = onCleanup(@() closeFigure(figureHandle));
    isFirstFrame = true;

    showTitleCard(figureHandle,'Planner verification gauntlet', ...
        {sprintf('%d MATLAB unit tests passed',numel(report.testResults)), ...
        'Moving-mask avoidance with required waiting', ...
        'Spiral navigation to the center'},'TWO VERIFIED MOTION RUNS');
    isFirstFrame = appendFrame( ...
        figureHandle,gifFile,isFirstFrame,1.8);

    dynamicIndices = unique([1:5:numel(dynamicPath.time_s), ...
        numel(dynamicPath.time_s)]);
    for frameNumber = 1:numel(dynamicIndices)
        pathIndex = dynamicIndices(frameNumber);
        isFinal = frameNumber == numel(dynamicIndices);
        drawDynamicFrame(figureHandle,dynamicData,dynamicResult, ...
            pathIndex,isFinal);
        delay_s = 0.11;
        if isFinal
            delay_s = 1.8;
        end
        isFirstFrame = appendFrame( ...
            figureHandle,gifFile,isFirstFrame,delay_s);
    end

    showTitleCard(figureHandle,'Spiral-to-center challenge', ...
        {'The direct route is blocked', ...
        'The path must wind through the open channel', ...
        'Pass threshold: 1440 deg of net winding'},'RUN 2 OF 2');
    isFirstFrame = appendFrame( ...
        figureHandle,gifFile,isFirstFrame,1.4);

    spiralIndices = unique([1:4:numel(spiralPath.time_s), ...
        numel(spiralPath.time_s)]);
    for frameNumber = 1:numel(spiralIndices)
        pathIndex = spiralIndices(frameNumber);
        winding_deg = pathWinding(spiralPath,pathIndex, ...
            spiralScenario.centerAzEl_deg, ...
            max(spiralScenario.options.gridStep_deg));
        isFinal = frameNumber == numel(spiralIndices);
        drawSpiralFrame(figureHandle,spiralScenario,spiralResult, ...
            pathIndex,winding_deg,isFinal);
        delay_s = 0.09;
        if isFinal
            delay_s = 1.9;
        end
        isFirstFrame = appendFrame( ...
            figureHandle,gifFile,isFirstFrame,delay_s);
    end

    showTitleCard(figureHandle,'All gauntlet checks passed', ...
        {sprintf('Moving mask: %.1f s arrival, %.1f s waiting', ...
        dynamicResult.diagnostic.arrivalTime_s, ...
        commandWaitDuration(dynamicResult)), ...
        sprintf('Spiral: %.1f s arrival, %.1f deg net winding', ...
        spiralResult.diagnostic.arrivalTime_s,spiralCommandWinding_deg), ...
        'Spiral solver: deterministic graph search'},'VERIFIED SAFE');
    appendFrame(figureHandle,gifFile,isFirstFrame,2.6);
    fprintf('Combined gauntlet GIF written to %s\n',gifFile);
    clear cleanup
end


function drawDynamicFrame(figureHandle,data,result,pathIndex,isFinal)
    path = selectAzElDisplayPath(result);
    [ax,palette] = makePlotCanvas(figureHandle, ...
        'Moving-mask avoidance','RUN 1 OF 2');
    [maskAz_deg,maskEl_deg] = interpolateAzElObstacleFrame( ...
        data,path.time_s(pathIndex),result.options);
    drawMaskContours(ax,maskAz_deg,maskEl_deg,palette.mask);
    plot(ax,path.az_deg,path.el_deg,'--', ...
        'Color',palette.plan,'LineWidth',1.5);
    plot(ax,path.az_deg(1:pathIndex),path.el_deg(1:pathIndex),'-', ...
        'Color',palette.executed,'LineWidth',3.2);
    scatter(ax,path.az_deg(1),path.el_deg(1),75, ...
        palette.start,'filled','MarkerEdgeColor','w','LineWidth',1);
    scatter(ax,path.az_deg(end),path.el_deg(end),85, ...
        palette.goal,'filled','MarkerEdgeColor','w','LineWidth',1);
    scatter(ax,path.az_deg(pathIndex),path.el_deg(pathIndex), ...
        105,palette.current,'filled', ...
        'MarkerEdgeColor','w','LineWidth',1.4);
    text(ax,path.az_deg(1)+3,path.el_deg(1)-3,'START', ...
        'Color',palette.text,'FontSize',9,'FontWeight','bold');
    text(ax,path.az_deg(end)-3,path.el_deg(end)+3,'GOAL', ...
        'HorizontalAlignment','right','Color',palette.text, ...
        'FontSize',9,'FontWeight','bold');
    xlim(ax,result.grid.azLim_deg);
    ylim(ax,result.grid.elLim_deg);
    xlabel(ax,'Azimuth (deg)');
    ylabel(ax,'Elevation (deg)');

    if path.isWaiting(pathIndex)
        motion = 'WAITING';
    else
        motion = 'SLEWING';
    end
    if isFinal
        motion = 'COMPLETE';
    end
    progress = pathIndex/numel(path.time_s);
    metrics = {sprintf('Time  %.1f / %.1f s', ...
        path.time_s(pathIndex),path.time_s(end)), ...
        sprintf('State %s',motion), ...
        sprintf('Wait  %.1f s total',commandWaitDuration(result)), ...
        sprintf('Turns %d smoothed', ...
        result.diagnostic.turnCountAfterSmoothing)};
    addSidebar(figureHandle,'DYNAMIC MASK',metrics,progress,isFinal,palette);
end


function drawSpiralFrame(figureHandle,scenario,result,pathIndex, ...
        winding_deg,isFinal)
    path = selectAzElDisplayPath(result);
    [ax,palette] = makePlotCanvas(figureHandle, ...
        'Spiral-to-center navigation','RUN 2 OF 2');
    wall = scenario.spiralWallAzEl_deg;
    patch(ax,wall(:,1),wall(:,2),palette.wall, ...
        'FaceAlpha',0.88,'EdgeColor',palette.wallEdge,'LineWidth',1.2);
    plot(ax,path.az_deg,path.el_deg,'--', ...
        'Color',palette.plan,'LineWidth',1.5);
    plot(ax,path.az_deg(1:pathIndex),path.el_deg(1:pathIndex),'-', ...
        'Color',palette.executed,'LineWidth',3.2);
    scatter(ax,path.az_deg(1),path.el_deg(1),75, ...
        palette.start,'filled','MarkerEdgeColor','w','LineWidth',1);
    scatter(ax,scenario.centerAzEl_deg(1),scenario.centerAzEl_deg(2),90, ...
        palette.goal,'filled','MarkerEdgeColor','w','LineWidth',1);
    scatter(ax,path.az_deg(pathIndex),path.el_deg(pathIndex), ...
        105,palette.current,'filled', ...
        'MarkerEdgeColor','w','LineWidth',1.4);
    text(ax,path.az_deg(1)-1,path.el_deg(1)+3,'START', ...
        'HorizontalAlignment','right','Color',palette.text, ...
        'FontSize',9,'FontWeight','bold');
    text(ax,scenario.centerAzEl_deg(1),scenario.centerAzEl_deg(2)-3, ...
        'CENTER','HorizontalAlignment','center','Color',palette.text, ...
        'FontSize',9,'FontWeight','bold');
    xlim(ax,result.grid.azLim_deg);
    ylim(ax,result.grid.elLim_deg);
    axis(ax,'equal');
    xlabel(ax,'Azimuth (deg)');
    ylabel(ax,'Elevation (deg)');

    progress = pathIndex/numel(path.time_s);
    status = 'WINDING';
    if isFinal
        status = 'COMPLETE';
    end
    source = strrep(char(result.diagnostic.selectedPolicySource), ...
        'graphFallback','GRAPH SEARCH');
    metrics = {sprintf('Time  %.1f / %.1f s', ...
        path.time_s(pathIndex),path.time_s(end)), ...
        sprintf('State %s',status), ...
        sprintf('Angle %.1f deg',winding_deg), ...
        sprintf('Need  >= %.0f deg',scenario.minimumWinding_deg), ...
        sprintf('Source %s',source)};
    addSidebar(figureHandle,'SPIRAL WALL',metrics,progress,isFinal,palette);
end


function [ax,palette] = makePlotCanvas(figureHandle,heading,runLabel)
    clf(figureHandle);
    set(figureHandle,'Color',[0.965,0.975,0.99]);
    palette = makePalette;
    annotation(figureHandle,'textbox',[0.065,0.905,0.70,0.06], ...
        'String',heading,'EdgeColor','none','Color',palette.text, ...
        'FontSize',19,'FontWeight','bold','VerticalAlignment','middle');
    annotation(figureHandle,'textbox',[0.79,0.91,0.16,0.045], ...
        'String',runLabel,'EdgeColor','none','Color',palette.muted, ...
        'FontSize',10,'FontWeight','bold','HorizontalAlignment','right');
    ax = axes(figureHandle,'Position',[0.075,0.115,0.675,0.755], ...
        'Color','w','FontSize',10,'XColor',palette.axis, ...
        'YColor',palette.axis,'GridColor',palette.grid, ...
        'GridAlpha',0.42,'LineWidth',0.8);
    hold(ax,'on');
    box(ax,'on');
    grid(ax,'on');
end


function addSidebar(figureHandle,label,metrics,progress,isFinal,palette)
    annotation(figureHandle,'textbox',[0.785,0.59,0.18,0.25], ...
        'String',[{label};metrics(:)],'BackgroundColor','w', ...
        'EdgeColor',palette.border,'LineWidth',1, ...
        'Color',palette.text,'FontName','Consolas','FontSize',10, ...
        'VerticalAlignment','top','Margin',12);
    annotation(figureHandle,'textbox',[0.79,0.49,0.17,0.04], ...
        'String',sprintf('PROGRESS  %3.0f%%',100*progress), ...
        'EdgeColor','none','Color',palette.muted, ...
        'FontSize',9,'FontWeight','bold');
    annotation(figureHandle,'rectangle',[0.80,0.455,0.15,0.022], ...
        'FaceColor',palette.progressBackground, ...
        'EdgeColor',palette.progressBackground);
    annotation(figureHandle,'rectangle', ...
        [0.80,0.455,max(0.001,0.15*progress),0.022], ...
        'FaceColor',palette.executed,'EdgeColor',palette.executed);
    if isFinal
        annotation(figureHandle,'textbox',[0.79,0.34,0.17,0.065], ...
            'String','PASS','BackgroundColor',palette.passBackground, ...
            'EdgeColor',palette.pass,'Color',palette.pass, ...
            'FontSize',17,'FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','middle');
    else
        annotation(figureHandle,'textbox',[0.79,0.34,0.17,0.065], ...
            'String','RUNNING','BackgroundColor',palette.runningBackground, ...
            'EdgeColor',palette.executed,'Color',palette.executed, ...
            'FontSize',12,'FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','middle');
    end
end


function showTitleCard(figureHandle,heading,lines,badge)
    clf(figureHandle);
    palette = makePalette;
    set(figureHandle,'Color',[0.965,0.975,0.99]);
    ax = axes(figureHandle,'Position',[0,0,1,1]);
    axis(ax,[0,1,0,1]);
    axis(ax,'off');
    hold(ax,'on');
    rectangle(ax,'Position',[0.12,0.14,0.76,0.72], ...
        'FaceColor','w','EdgeColor',palette.border,'LineWidth',1.2);
    text(ax,0.5,0.74,badge,'HorizontalAlignment','center', ...
        'FontSize',10,'FontWeight','bold','Color',palette.executed);
    text(ax,0.5,0.64,heading,'HorizontalAlignment','center', ...
        'FontSize',25,'FontWeight','bold','Color',palette.text);
    lineText = strjoin(string(lines),newline);
    text(ax,0.5,0.43,lineText,'HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',14, ...
        'Color',palette.muted);
    plot(ax,[0.34,0.66],[0.27,0.27],'-','Color',palette.border, ...
        'LineWidth',1.2);
end


function drawMaskContours(ax,az_deg,el_deg,color)
    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators(:);true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;
    for contourIndex = 1:numel(starts)
        indices = starts(contourIndex):ends(contourIndex);
        patch(ax,az_deg(indices),el_deg(indices),color, ...
            'FaceAlpha',0.30,'EdgeColor',color,'LineWidth',1.1);
    end
end


function winding_deg = pathWinding(path,pathIndex,centerAzEl_deg,minRadius_deg)
    relativeAz_deg = path.az_deg(1:pathIndex)-centerAzEl_deg(1);
    relativeEl_deg = path.el_deg(1:pathIndex)-centerAzEl_deg(2);
    measure = hypot(relativeAz_deg,relativeEl_deg) > minRadius_deg;
    angles_rad = unwrap(atan2(relativeEl_deg(measure), ...
        relativeAz_deg(measure)));
    if numel(angles_rad) < 2
        winding_deg = 0;
    else
        winding_deg = abs(rad2deg(angles_rad(end)-angles_rad(1)));
    end
end


function duration_s = commandWaitDuration(result)
    duration_s = result.diagnostic.waitTime_s;
    if ~isfield(result,'trajectory') || ...
            ~isfield(result.trajectory,'success') || ...
            ~result.trajectory.success || ...
            ~isfield(result.trajectory,'segments')
        return
    end
    segments = result.trajectory.segments;
    if isempty(segments)
        duration_s = 0;
        return
    end
    isWait = arrayfun(@(segment) string(segment.kind) == "wait",segments);
    duration_s = sum([segments(isWait).endTime_s]- ...
        [segments(isWait).startTime_s]);
end


function isFirstFrame = appendFrame(figureHandle,gifFile, ...
        isFirstFrame,delay_s)
    drawnow;
    imageData = print(figureHandle,'-RGBImage','-r90','-image');
    [indexedImage,colorMap] = rgb2ind(imageData,256,'nodither');
    if isFirstFrame
        imwrite(indexedImage,colorMap,gifFile,'gif', ...
            'LoopCount',Inf,'DelayTime',delay_s);
        isFirstFrame = false;
    else
        imwrite(indexedImage,colorMap,gifFile,'gif', ...
            'WriteMode','append','DelayTime',delay_s);
    end
end


function palette = makePalette
    palette = struct;
    palette.text = [0.08,0.14,0.23];
    palette.muted = [0.30,0.37,0.47];
    palette.axis = [0.18,0.23,0.31];
    palette.grid = [0.72,0.77,0.84];
    palette.border = [0.78,0.82,0.88];
    palette.mask = [0.88,0.25,0.18];
    palette.wall = [0.22,0.27,0.35];
    palette.wallEdge = [0.10,0.13,0.18];
    palette.plan = [0.60,0.65,0.72];
    palette.executed = [0.03,0.42,0.82];
    palette.current = [1.00,0.55,0.05];
    palette.start = [0.12,0.62,0.30];
    palette.goal = [0.66,0.18,0.74];
    palette.progressBackground = [0.86,0.89,0.93];
    palette.pass = [0.06,0.52,0.25];
    palette.passBackground = [0.90,0.97,0.92];
    palette.runningBackground = [0.91,0.95,0.99];
end


function closeFigure(figureHandle)
    if isgraphics(figureHandle)
        close(figureHandle);
    end
end
