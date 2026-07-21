function gifFile = createGauntletSuiteGif(gifFile,report)
%CREATEGAUNTLETSUITEGIF Render every named gauntlet into one fixed-size GIF.
%   GIFFILE = CREATEGAUNTLETSUITEGIF writes a 1200-by-720 animation that
%   contains the 18 planner scenarios, the spiral-to-center run, and the
%   seven azimuth-wraparound checks. Supplying REPORT avoids rerunning the
%   gauntlet; REPORT must be an output from RUNGAUNTLET.

    projectFolder = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(gifFile) || ...
            all(strlength(string(gifFile)) == 0)
        gifFile = fullfile(projectFolder,'gauntlet_complete_suite.gif');
    end
    if nargin < 2 || isempty(report)
        report = runGauntlet;
    end
    requireReportFields(report);
    requireSuccessfulReport(report);
    gifFile = char(gifFile);

    canvasPixels = [720,1200];
    figureHandle = figure('Color',[0.965,0.975,0.99], ...
        'Visible','off','Units','pixels', ...
        'Position',[40,40,canvasPixels(2),canvasPixels(1)], ...
        'InvertHardcopy','off','Name','Complete planner gauntlet');
    set(figureHandle,'PaperPositionMode','auto');
    cleanup = onCleanup(@() closeFigure(figureHandle));
    isFirstFrame = true;
    frameCount = 0;

    showTitleCard(figureHandle,'Complete obstacle-avoidance gauntlet', ...
        {sprintf('%d planner scenarios', ...
        report.scenarioSuite.numScenarios), ...
        sprintf('%.2f-turn spiral to the center', ...
        report.spiralScenario.spiralCycles), ...
        sprintf('%d seam checks; analytic C2 command motion', ...
        report.wraparound.numTests)},'ALL RUNS IN ONE FIXED CANVAS');
    [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
        isFirstFrame,1.8,canvasPixels,frameCount);

    entries = report.scenarioSuite.entries;
    for scenarioIndex = 1:numel(entries)
        entry = entries(scenarioIndex);
        showScenarioTitle(figureHandle,entry,scenarioIndex,numel(entries));
        [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
            isFirstFrame,0.75,canvasPixels,frameCount);

        result = entry.result;
        displayPath = selectAzElDisplayPath(result);
        if ~isempty(displayPath.time_s)
            pathIndices = sampleIndices(numel(displayPath.time_s),12);
            for frameIndex = 1:numel(pathIndices)
                pathIndex = pathIndices(frameIndex);
                isFinal = frameIndex == numel(pathIndices);
                drawScenarioFrame(figureHandle,entry,scenarioIndex, ...
                    numel(entries),pathIndex,[],isFinal);
                delay_s = 0.11;
                if isFinal
                    delay_s = 0.85;
                end
                [isFirstFrame,frameCount] = appendFrame( ...
                    figureHandle,gifFile,isFirstFrame,delay_s, ...
                    canvasPixels,frameCount);
            end
        else
            dataIndices = sampleIndices(numel(entry.scenario.data.time_s),3);
            for frameIndex = 1:numel(dataIndices)
                dataIndex = dataIndices(frameIndex);
                isFinal = frameIndex == numel(dataIndices);
                drawScenarioFrame(figureHandle,entry,scenarioIndex, ...
                    numel(entries),0,dataIndex,isFinal);
                delay_s = 0.18;
                if isFinal
                    delay_s = 1.0;
                end
                [isFirstFrame,frameCount] = appendFrame( ...
                    figureHandle,gifFile,isFirstFrame,delay_s, ...
                    canvasPixels,frameCount);
            end
        end
    end

    showTitleCard(figureHandle,'Spiral to center', ...
        {sprintf('Wall: %.2f complete turns', ...
        report.spiralScenario.spiralCycles), ...
        sprintf('Required path winding: at least %.0f deg', ...
        report.spiralScenario.minimumWinding_deg), ...
        'The blue clearance halo must remain outside the wall'}, ...
        'SPIRAL GAUNTLET');
    [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
        isFirstFrame,1.0,canvasPixels,frameCount);

    spiralPath = selectAzElDisplayPath(report.spiralResult);
    spiralIndices = sampleIndices(numel(spiralPath.time_s),36);
    for frameIndex = 1:numel(spiralIndices)
        pathIndex = spiralIndices(frameIndex);
        isFinal = frameIndex == numel(spiralIndices);
        drawSpiralFrame(figureHandle,report,pathIndex,isFinal);
        delay_s = 0.09;
        if isFinal
            delay_s = 1.3;
        end
        [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
            isFirstFrame,delay_s,canvasPixels,frameCount);
    end

    showTitleCard(figureHandle,'Azimuth wraparound checks', ...
        {'Shortest routes cross the +180 / -180 seam', ...
        'Interpolation, rate, acceleration, and jerk remain continuous', ...
        'Equivalent endpoints and repeated crossings do not jump'}, ...
        'SEVEN DETERMINISTIC CHECKS');
    [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
        isFirstFrame,1.0,canvasPixels,frameCount);

    wrapEntries = report.wraparound.entries;
    for entryIndex = 1:numel(wrapEntries)
        showWraparoundCard(figureHandle,wrapEntries(entryIndex), ...
            entryIndex,numel(wrapEntries));
        [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
            isFirstFrame,0.9,canvasPixels,frameCount);
    end

    minimumClearance_deg = getAuditClearance(report.spiralAudit);
    showTitleCard(figureHandle,'Every named gauntlet passed', ...
        {sprintf('%d / %d planner scenarios', ...
        report.scenarioSuite.numPassed,report.scenarioSuite.numScenarios), ...
        sprintf('Spiral: %.1f deg net winding, %.3f deg minimum clearance', ...
        report.spiralWinding_deg,minimumClearance_deg), ...
        sprintf('%d / %d azimuth wraparound checks', ...
        report.wraparound.numPassed,report.wraparound.numTests)}, ...
        sprintf('%d / %d VERIFIED',report.numNamedChecksPassed, ...
        report.numNamedChecks));
    [~,frameCount] = appendFrame(figureHandle,gifFile, ...
        isFirstFrame,2.8,canvasPixels,frameCount);

    validateGif(gifFile,canvasPixels,frameCount);
    fprintf('Complete gauntlet GIF written to %s (%d frames)\n', ...
        gifFile,frameCount);
    clear cleanup
end


function requireReportFields(report)
    required = {'scenarioSuite','spiralScenario','spiralResult', ...
        'spiralAudit','spiralWinding_deg','wraparound', ...
        'numNamedChecks','numNamedChecksPassed'};
    missing = required(~isfield(report,required));
    if ~isempty(missing)
        error('createGauntletSuiteGif:IncompleteReport', ...
            'REPORT is missing: %s.',strjoin(missing,', '));
    end
end


function requireSuccessfulReport(report)
    scenariosPassed = isfield(report.scenarioSuite,'passed') && ...
        report.scenarioSuite.passed && ...
        all([report.scenarioSuite.entries.passed]);
    wrapPassed = isfield(report.wraparound,'passed') && ...
        report.wraparound.passed && all([report.wraparound.entries.passed]);
    spiralPassed = report.spiralResult.success && ...
        report.spiralAudit.collisionFree;
    countsAgree = report.numNamedChecksPassed == report.numNamedChecks;
    if ~isfield(report,'passed') || ~report.passed || ...
            ~scenariosPassed || ~wrapPassed || ~spiralPassed || ~countsAgree
        error('createGauntletSuiteGif:GauntletFailed', ...
            'The GIF can only claim completion for a fully passing report.');
    end
end


function showScenarioTitle(figureHandle,entry,index,total)
    auditText = auditSummary(entry);
    showTitleCard(figureHandle,char(entry.name), ...
        {char(entry.setup),sprintf('Verified: %s',entry.reason),auditText}, ...
        sprintf('PLANNER GAUNTLET %02d OF %02d',index,total));
end


function drawScenarioFrame(figureHandle,entry,index,total,pathIndex, ...
        suppliedDataIndex,isFinal)
    [ax,palette] = makePlotCanvas(figureHandle,char(entry.name), ...
        sprintf('SCENARIO %02d / %02d',index,total));
    scenario = entry.scenario;
    result = entry.result;
    path = selectAzElDisplayPath(result);

    if pathIndex > 0
        dataIndex = pathDataIndex(path,pathIndex);
        currentAzEl_deg = [path.az_deg(pathIndex),path.el_deg(pathIndex)];
        currentTime_s = path.time_s(pathIndex);
    else
        dataIndex = suppliedDataIndex;
        currentAzEl_deg = scenario.startAzEl_deg;
        currentTime_s = scenario.data.time_s(dataIndex);
    end

    drawStaticPolygons(ax,scenario.options.staticPolygons,palette);
    drawDynamicFramePolygons(ax,scenario.data,currentTime_s, ...
        scenario.options,palette);

    if ~isempty(path.time_s)
        plot(ax,path.az_deg,path.el_deg,':', ...
            'Color',palette.plan,'LineWidth',1.5);
        plot(ax,path.az_deg(1:pathIndex),path.el_deg(1:pathIndex),'-', ...
            'Color',palette.executed,'LineWidth',2.6);
    end

    goalAzEl_deg = goalAtTime(scenario,currentTime_s);
    scatter(ax,scenario.startAzEl_deg(1),scenario.startAzEl_deg(2), ...
        68,palette.start,'filled','MarkerEdgeColor','w','LineWidth',1);
    scatter(ax,goalAzEl_deg(1),goalAzEl_deg(2),80,palette.goal, ...
        'filled','MarkerEdgeColor','w','LineWidth',1);
    drawCurrentPosition(ax,currentAzEl_deg, ...
        scenario.options.clearance_deg,palette);
    text(ax,scenario.startAzEl_deg(1)+0.7,scenario.startAzEl_deg(2)-0.8, ...
        'START','Color',palette.text,'FontSize',8,'FontWeight','bold');
    text(ax,goalAzEl_deg(1)-0.7,goalAzEl_deg(2)+0.8,'GOAL', ...
        'HorizontalAlignment','right','Color',palette.text, ...
        'FontSize',8,'FontWeight','bold');

    xlim(ax,scenario.options.azLim_deg);
    ylim(ax,scenario.options.elLim_deg);
    axis(ax,'equal');
    xlabel(ax,'Azimuth (deg)');
    ylabel(ax,'Elevation (deg)');

    if ~entry.expectedSuccess
        if isempty(path.time_s)
            progress = dataIndex/numel(scenario.data.time_s);
        else
            progress = pathIndex/numel(path.time_s);
        end
        state = 'SEARCHING';
        if isFinal
            state = 'NO ROUTE';
        end
        arrivalText = 'Expected result: unreachable';
        sourceText = sprintf('Source %s',formatSource( ...
            result.diagnostic.selectedPolicySource));
    elseif isempty(path.time_s)
        progress = dataIndex/numel(scenario.data.time_s);
        state = 'SEARCHING';
        if isFinal
            state = 'NO ROUTE';
        end
        arrivalText = 'Expected result: unreachable';
        sourceText = 'Exhaustive search completed';
    else
        progress = pathIndex/numel(path.time_s);
        if path.isWaiting(pathIndex)
            state = 'WAITING';
        else
            state = 'SLEWING';
        end
        if isFinal
            state = 'COMPLETE';
        end
        arrivalText = sprintf('Arrival %.1f s', ...
            result.diagnostic.arrivalTime_s);
        sourceText = sprintf('Source %s',formatSource( ...
            result.diagnostic.selectedPolicySource));
    end
    if entry.expectedSuccess
        commandText = 'C2 command: verified';
        sidebarLabel = 'TIME-ALIGNED C2 MOTION';
    else
        commandText = 'No command: unreachable verified';
        sidebarLabel = 'EXHAUSTIVE NO-ROUTE CHECK';
    end
    metrics = {sprintf('Time %.1f s',currentTime_s), ...
        sprintf('State %s',state),arrivalText,sourceText, ...
        auditSummary(entry),commandText};
    addSidebar(figureHandle,sidebarLabel,metrics, ...
        progress,isFinal && entry.passed,palette,~entry.expectedSuccess);
end


function drawSpiralFrame(figureHandle,report,pathIndex,isFinal)
    scenario = report.spiralScenario;
    result = report.spiralResult;
    path = selectAzElDisplayPath(result);
    [ax,palette] = makePlotCanvas(figureHandle, ...
        'Spiral-to-center navigation','SPIRAL: 5.25 TURNS');

    wall = scenario.spiralWallAzEl_deg;
    patch(ax,wall(:,1),wall(:,2),palette.wall, ...
        'FaceAlpha',0.88,'EdgeColor',palette.wallEdge,'LineWidth',1.0);
    plot(ax,path.az_deg,path.el_deg,':', ...
        'Color',palette.plan,'LineWidth',1.4);
    plot(ax,path.az_deg(1:pathIndex),path.el_deg(1:pathIndex),'-', ...
        'Color',palette.executed,'LineWidth',2.2);
    scatter(ax,path.az_deg(1),path.el_deg(1),68, ...
        palette.start,'filled','MarkerEdgeColor','w','LineWidth',1);
    scatter(ax,scenario.centerAzEl_deg(1),scenario.centerAzEl_deg(2), ...
        80,palette.goal,'filled','MarkerEdgeColor','w','LineWidth',1);
    currentAzEl_deg = [path.az_deg(pathIndex),path.el_deg(pathIndex)];
    drawCurrentPosition(ax,currentAzEl_deg, ...
        scenario.options.clearance_deg,palette);
    text(ax,path.az_deg(1)-0.8,path.el_deg(1)+1.2, ...
        'START','HorizontalAlignment','right','Color',palette.text, ...
        'FontSize',8,'FontWeight','bold');
    text(ax,scenario.centerAzEl_deg(1),scenario.centerAzEl_deg(2)-1.5, ...
        'CENTER','HorizontalAlignment','center','Color',palette.text, ...
        'FontSize',8,'FontWeight','bold');
    xlim(ax,result.grid.azLim_deg);
    ylim(ax,result.grid.elLim_deg);
    axis(ax,'equal');
    xlabel(ax,'Azimuth (deg)');
    ylabel(ax,'Elevation (deg)');

    winding_deg = netPathWinding(path,pathIndex, ...
        scenario.centerAzEl_deg,max(scenario.options.gridStep_deg));
    progress = pathIndex/numel(path.time_s);
    state = 'WINDING';
    if isFinal
        state = 'CENTER REACHED';
    end
    metrics = {sprintf('Time %.1f / %.1f s', ...
        path.time_s(pathIndex),path.time_s(end)), ...
        sprintf('State %s',state), ...
        sprintf('Net winding %.1f deg',winding_deg), ...
        sprintf('Required >= %.0f deg',scenario.minimumWinding_deg), ...
        sprintf('Min clearance %.3f deg', ...
        getAuditClearance(report.spiralAudit)), ...
        sprintf('Source %s',formatSource( ...
        result.diagnostic.selectedPolicySource))};
    addSidebar(figureHandle,'CLEARANCE-AUDITED C2 PATH',metrics, ...
        progress,isFinal,palette,false);
end


function showWraparoundCard(figureHandle,entry,index,total)
    clf(figureHandle);
    palette = makePalette;
    set(figureHandle,'Color',[0.965,0.975,0.99]);
    ax = axes(figureHandle,'Position',[0,0,1,1]);
    axis(ax,[0,1,0,1]);
    axis(ax,'off');
    hold(ax,'on');

    rectangle(ax,'Position',[0.10,0.11,0.80,0.78], ...
        'FaceColor','w','EdgeColor',palette.border,'LineWidth',1.2);
    text(ax,0.14,0.82,sprintf('AZIMUTH CHECK %d OF %d',index,total), ...
        'FontSize',10,'FontWeight','bold','Color',palette.executed);
    text(ax,0.14,0.70,char(entry.name),'FontSize',24, ...
        'FontWeight','bold','Color',palette.text);
    text(ax,0.14,0.56,['Expected: ' char(entry.expected)], ...
        'FontSize',13,'Color',palette.muted,'Interpreter','none');
    text(ax,0.14,0.45,['Observed: ' char(entry.observed)], ...
        'FontSize',13,'Color',palette.text,'Interpreter','none');

    plot(ax,[0.20,0.80],[0.31,0.31],'-','Color',palette.border, ...
        'LineWidth',2);
    plot(ax,[0.49,0.51],[0.31,0.31],'-','Color',palette.executed, ...
        'LineWidth',7);
    text(ax,0.50,0.265,'+180 / -180 seam', ...
        'HorizontalAlignment','center','FontSize',10, ...
        'Color',palette.muted);
    if entry.passed
        status = 'PASS';
        statusColor = palette.pass;
        statusBackground = palette.passBackground;
    else
        status = 'FAIL';
        statusColor = palette.fail;
        statusBackground = palette.failBackground;
    end
    text(ax,0.79,0.18,status,'HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',18,'FontWeight','bold', ...
        'Color',statusColor,'BackgroundColor',statusBackground, ...
        'EdgeColor',statusColor,'Margin',10);
end


function [ax,palette] = makePlotCanvas(figureHandle,heading,runLabel)
    clf(figureHandle);
    set(figureHandle,'Color',[0.965,0.975,0.99]);
    palette = makePalette;
    annotation(figureHandle,'textbox',[0.055,0.91,0.70,0.06], ...
        'String',heading,'EdgeColor','none','Color',palette.text, ...
        'FontSize',19,'FontWeight','bold','VerticalAlignment','middle');
    annotation(figureHandle,'textbox',[0.78,0.915,0.18,0.045], ...
        'String',runLabel,'EdgeColor','none','Color',palette.muted, ...
        'FontSize',9,'FontWeight','bold','HorizontalAlignment','right');
    ax = axes(figureHandle,'Position',[0.060,0.105,0.690,0.785], ...
        'Color','w','FontSize',10,'XColor',palette.axis, ...
        'YColor',palette.axis,'GridColor',palette.grid, ...
        'GridAlpha',0.42,'LineWidth',0.8);
    hold(ax,'on');
    box(ax,'on');
    grid(ax,'on');
end


function addSidebar(figureHandle,label,metrics,progress,isFinal,palette, ...
        expectedFailure)
    annotation(figureHandle,'textbox',[0.775,0.53,0.20,0.34], ...
        'String',[{label};metrics(:)],'BackgroundColor','w', ...
        'EdgeColor',palette.border,'LineWidth',1, ...
        'Color',palette.text,'FontName','Consolas','FontSize',9, ...
        'VerticalAlignment','top','Margin',12);
    annotation(figureHandle,'textbox',[0.785,0.445,0.18,0.04], ...
        'String',sprintf('PROGRESS %3.0f%%',100*progress), ...
        'EdgeColor','none','Color',palette.muted, ...
        'FontSize',9,'FontWeight','bold');
    annotation(figureHandle,'rectangle',[0.79,0.410,0.17,0.022], ...
        'FaceColor',palette.progressBackground, ...
        'EdgeColor',palette.progressBackground);
    annotation(figureHandle,'rectangle', ...
        [0.79,0.410,max(0.001,0.17*progress),0.022], ...
        'FaceColor',palette.executed,'EdgeColor',palette.executed);
    if isFinal
        if expectedFailure
            labelText = 'PASS - NO ROUTE';
            fontSize = 12;
        else
            labelText = 'PASS';
            fontSize = 17;
        end
        annotation(figureHandle,'textbox',[0.79,0.29,0.17,0.065], ...
            'String',labelText,'BackgroundColor',palette.passBackground, ...
            'EdgeColor',palette.pass,'Color',palette.pass, ...
            'FontSize',fontSize,'FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','middle');
    else
        annotation(figureHandle,'textbox',[0.79,0.29,0.17,0.065], ...
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
    rectangle(ax,'Position',[0.10,0.12,0.80,0.76], ...
        'FaceColor','w','EdgeColor',palette.border,'LineWidth',1.2);
    text(ax,0.5,0.77,badge,'HorizontalAlignment','center', ...
        'FontSize',10,'FontWeight','bold','Color',palette.executed, ...
        'Interpreter','none');
    text(ax,0.5,0.65,heading,'HorizontalAlignment','center', ...
        'FontSize',25,'FontWeight','bold','Color',palette.text, ...
        'Interpreter','none');
    lineText = strjoin(string(lines),newline);
    text(ax,0.5,0.43,lineText,'HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',13, ...
        'Color',palette.muted,'Interpreter','none');
    plot(ax,[0.32,0.68],[0.25,0.25],'-','Color',palette.border, ...
        'LineWidth',1.2);
end


function drawStaticPolygons(ax,polygons,palette)
    for polygonIndex = 1:numel(polygons)
        polygon = polygons{polygonIndex};
        if isempty(polygon)
            continue
        end
        patch(ax,polygon(:,1),polygon(:,2),palette.wall, ...
            'FaceAlpha',0.86,'EdgeColor',palette.wallEdge,'LineWidth',1.0);
    end
end


function drawDynamicFramePolygons(ax,data,time_s,options,palette)
    [az_deg,el_deg] = interpolateAzElObstacleFrame(data,time_s,options);
    if isempty(az_deg)
        return
    end
    separators = isnan(az_deg) | isnan(el_deg);
    changes = diff([true;separators(:);true]);
    starts = find(changes == -1);
    ends = find(changes == 1)-1;
    for contourIndex = 1:numel(starts)
        indices = starts(contourIndex):ends(contourIndex);
        patch(ax,az_deg(indices),el_deg(indices),palette.mask, ...
            'FaceAlpha',0.34,'EdgeColor',palette.mask,'LineWidth',1.0);
    end
end


function drawCurrentPosition(ax,azEl_deg,clearance_deg,palette)
    if clearance_deg > 0
        angle_rad = linspace(0,2*pi,50);
        patch(ax,azEl_deg(1)+clearance_deg*cos(angle_rad), ...
            azEl_deg(2)+clearance_deg*sin(angle_rad), ...
            palette.halo,'FaceAlpha',0.20,'EdgeColor',palette.haloEdge, ...
            'LineStyle','--','LineWidth',0.8);
    end
    scatter(ax,azEl_deg(1),azEl_deg(2),80,palette.current,'filled', ...
        'MarkerEdgeColor','w','LineWidth',1.2);
end


function goalAzEl_deg = goalAtTime(scenario,time_s)
    if size(scenario.goalAzEl_deg,1) == 1
        goalAzEl_deg = scenario.goalAzEl_deg;
        return
    end
    dataTime_s = scenario.data.time_s(:);
    if time_s <= dataTime_s(1)
        goalAzEl_deg = scenario.goalAzEl_deg(1,:);
        return
    elseif time_s >= dataTime_s(end)
        goalAzEl_deg = scenario.goalAzEl_deg(end,:);
        return
    end
    firstIndex = find(dataTime_s <= time_s,1,'last');
    secondIndex = firstIndex+1;
    fraction = (time_s-dataTime_s(firstIndex))/ ...
        (dataTime_s(secondIndex)-dataTime_s(firstIndex));
    firstGoal = scenario.goalAzEl_deg(firstIndex,:);
    secondGoal = scenario.goalAzEl_deg(secondIndex,:);
    goalAzEl_deg = firstGoal+fraction.*[ ...
        shortestAzimuthDeltaDeg(firstGoal(1),secondGoal(1)), ...
        secondGoal(2)-firstGoal(2)];
    if isfield(scenario.options,'azimuthTopology') && ...
            strcmpi(string(scenario.options.azimuthTopology),"periodic")
        azimuthMinimum_deg = scenario.options.azLim_deg(1);
        goalAzEl_deg(1) = mod(goalAzEl_deg(1)-azimuthMinimum_deg,360)+ ...
            azimuthMinimum_deg;
    end
end


function dataIndex = pathDataIndex(path,pathIndex)
    if isfield(path,'timeIndex') && numel(path.timeIndex) >= pathIndex
        dataIndex = path.timeIndex(pathIndex);
    elseif isfield(path,'planningTimeIndex') && ...
            numel(path.planningTimeIndex) >= pathIndex
        dataIndex = path.planningTimeIndex(pathIndex);
    else
        dataIndex = pathIndex;
    end
end


function textValue = auditSummary(entry)
    if isfield(entry,'audit') && ~isempty(entry.audit)
        clearance_deg = getAuditClearance(entry.audit);
        if isfinite(clearance_deg)
            textValue = sprintf('Audit: safe, min clearance %.3f deg', ...
                clearance_deg);
        else
            textValue = 'Audit: safe, no obstacle contact';
        end
    elseif isfield(entry.result,'diagnostic') && ...
            isfield(entry.result.diagnostic,'collisionFree') && ...
            entry.result.diagnostic.collisionFree
        textValue = 'Audit: planner collision check passed';
    else
        textValue = 'Audit: safe unreachable result';
    end
end


function clearance_deg = getAuditClearance(audit)
    if isfield(audit,'certifiedMinimumClearance_deg')
        clearance_deg = audit.certifiedMinimumClearance_deg;
    elseif isfield(audit,'minimumClearance_deg')
        clearance_deg = audit.minimumClearance_deg;
    elseif isfield(audit,'minClearance_deg')
        clearance_deg = audit.minClearance_deg;
    else
        clearance_deg = NaN;
    end
end


function textValue = formatSource(source)
    textValue = char(string(source));
    textValue = strrep(textValue,'graphFallback','graph search');
    textValue = strrep(textValue,'qLearning','Q-learning');
end


function winding_deg = netPathWinding(path,pathIndex,centerAzEl_deg, ...
        minRadius_deg)
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


function indices = sampleIndices(numValues,numSamples)
    if numValues <= 0
        indices = zeros(1,0);
        return
    end
    indices = unique(round(linspace(1,numValues,min(numValues,numSamples))));
end


function [isFirstFrame,frameCount] = appendFrame(figureHandle,gifFile, ...
        isFirstFrame,delay_s,canvasPixels,frameCount)
    drawnow;
    imageData = print(figureHandle,'-RGBImage','-r96','-image');
    imageData = normalizeFrameSize(imageData,canvasPixels);
    [indexedImage,colorMap] = rgb2ind(imageData,256,'nodither');
    if isFirstFrame
        imwrite(indexedImage,colorMap,gifFile,'gif', ...
            'LoopCount',Inf,'DelayTime',delay_s);
        isFirstFrame = false;
    else
        imwrite(indexedImage,colorMap,gifFile,'gif', ...
            'WriteMode','append','DelayTime',delay_s);
    end
    frameCount = frameCount+1;
end


function imageData = normalizeFrameSize(imageData,canvasPixels)
    currentSize = size(imageData);
    if currentSize(1) == canvasPixels(1) && ...
            currentSize(2) == canvasPixels(2)
        return
    end
    rowIndices = round(linspace(1,currentSize(1),canvasPixels(1)));
    columnIndices = round(linspace(1,currentSize(2),canvasPixels(2)));
    imageData = imageData(rowIndices,columnIndices,:);
end


function validateGif(gifFile,canvasPixels,expectedFrames)
    information = imfinfo(gifFile);
    heights = [information.Height];
    widths = [information.Width];
    if numel(information) ~= expectedFrames || ...
            any(heights ~= canvasPixels(1)) || ...
            any(widths ~= canvasPixels(2))
        error('createGauntletSuiteGif:InvalidCanvas', ...
            'The GIF is not a fixed %d-by-%d canvas with %d frames.', ...
            canvasPixels(2),canvasPixels(1),expectedFrames);
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
    palette.halo = [0.20,0.68,1.00];
    palette.haloEdge = [0.02,0.38,0.72];
    palette.progressBackground = [0.86,0.89,0.93];
    palette.pass = [0.06,0.52,0.25];
    palette.passBackground = [0.90,0.97,0.92];
    palette.fail = [0.72,0.12,0.12];
    palette.failBackground = [0.99,0.91,0.91];
    palette.runningBackground = [0.91,0.95,0.99];
end


function closeFigure(figureHandle)
    if isgraphics(figureHandle)
        close(figureHandle);
    end
end
