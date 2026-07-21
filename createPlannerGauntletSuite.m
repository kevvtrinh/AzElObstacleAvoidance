function suite = createPlannerGauntletSuite
%CREATEPLANNERGAUNTLETSUITE Build deterministic planner stress scenarios.

    suite = repmat(makeBaseScenario("",'',"",(0:1:1)'),18,1);

    %% 1. Narrow hallway
    scenario = makeBaseScenario("Narrow hallway", ...
        'Long corridor only slightly wider than the path clearance.', ...
        "narrowHallway",(0:1:42)');
    scenario.options.staticPolygons = { ...
        rectanglePolygon(-20,20,2.1,12), ...
        rectanglePolygon(-20,20,-12,-2.1)};
    suite(1) = scenario;

    %% 2. Piston slalom
    scenario = makeBaseScenario("Piston slalom", ...
        'Alternating pillars move vertically across the route.', ...
        "pistonSlalom",(0:1:48)');
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@pistonPolygons,'Piston slalom');
    suite(2) = scenario;

    %% 3. Crossing traffic
    scenario = makeBaseScenario("Crossing traffic", ...
        'Obstacles repeatedly cross perpendicular to the route.', ...
        "crossingTraffic",(0:1:48)');
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@crossingTrafficPolygons,'Crossing traffic');
    suite(3) = scenario;

    %% 4. Timed gate
    scenario = makeBaseScenario("Timed gate", ...
        'A doorway periodically opens and closes.', ...
        "timedGate",(0:1:48)');
    scenario.options.staticPolygons = verticalWallWithGap(0,0,2.2);
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@timedGatePolygons,'Timed gate');
    suite(4) = scenario;

    %% 5. Sweeping arm
    scenario = makeBaseScenario("Sweeping arm", ...
        'A long obstacle rotates across the direct route.', ...
        "sweepingArm",(0:1:45)');
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@sweepingArmPolygons,'Sweeping arm');
    suite(5) = scenario;

    %% 6. Moving tunnel
    scenario = makeBaseScenario("Moving tunnel", ...
        'Two moving walls form a shifting feasible corridor.', ...
        "movingTunnel",(0:1:48)');
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@movingTunnelPolygons,'Moving tunnel');
    suite(6) = scenario;

    %% 7. Shrinking funnel
    scenario = makeBaseScenario("Shrinking funnel", ...
        'A wide entrance narrows to a tightly aligned exit.', ...
        "shrinkingFunnel",(0:1:40)');
    scenario.startAzEl_deg = [-18,6];
    scenario.options.staticPolygons = shrinkingFunnelPolygons;
    suite(7) = scenario;

    %% 8. Offset keyholes
    scenario = makeBaseScenario("Offset keyholes", ...
        'Three narrow openings are deliberately misaligned.', ...
        "offsetKeyholes",(0:1:55)');
    scenario.options.staticPolygons = [ ...
        verticalWallWithGap(-8,3,2), ...
        verticalWallWithGap(0,-3,2), ...
        verticalWallWithGap(8,2,2)];
    suite(8) = scenario;

    %% 9. Oncoming traffic
    scenario = makeBaseScenario("Oncoming traffic", ...
        'An obstacle travels toward the path through a narrow lane.', ...
        "oncomingTraffic",(0:1:46)');
    scenario.options.staticPolygons = { ...
        rectanglePolygon(-20,20,4.2,12), ...
        rectanglePolygon(-20,20,-12,-4.2)};
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@oncomingTrafficPolygons,'Oncoming traffic');
    suite(9) = scenario;

    %% 10. Two-way trap
    scenario = makeBaseScenario("Two-way trap", ...
        'Both detours look open, but the upper branch later closes.', ...
        "twoWayTrap",(0:1:50)');
    scenario.options.staticPolygons = {rectanglePolygon(-5,5,-3,3)};
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@twoWayTrapPolygons,'Two-way trap');
    suite(10) = scenario;

    %% 11. U-shaped trap
    scenario = makeBaseScenario("U-shaped trap", ...
        'The goal is close in Euclidean distance but behind a U-shaped wall.', ...
        "uShapedTrap",(0:1:58)');
    scenario.startAzEl_deg = [-10,0];
    scenario.goalAzEl_deg = [5,0];
    scenario.options.staticPolygons = { ...
        rectanglePolygon(-1,1,-7,7), ...
        rectanglePolygon(-1,10,5,7), ...
        rectanglePolygon(-1,10,-7,-5)};
    suite(11) = scenario;

    %% 12. Timed shortcut
    scenario = makeBaseScenario("Timed shortcut", ...
        'A short doorway opens after a wait; a longer detour is always open.', ...
        "timedShortcut",(0:1:45)');
    scenario.options.staticPolygons = { ...
        rectanglePolygon(-1,1,2.1,8), ...
        rectanglePolygon(-1,1,-8,-2.1)};
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@timedShortcutPolygons,'Timed shortcut');
    suite(12) = scenario;

    %% 13. Double gate
    scenario = makeBaseScenario("Double gate", ...
        'Two timed doors must be crossed in the correct sequence.', ...
        "doubleGate",(0:1:50)');
    scenario.options.staticPolygons = [ ...
        verticalWallWithGap(-5,0,2.2),verticalWallWithGap(5,0,2.2)];
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@doubleGatePolygons,'Double gate');
    suite(13) = scenario;

    %% 14. Pop-up obstacle
    scenario = makeBaseScenario("Pop-up obstacle", ...
        'An obstacle appears on the nominal path with advance warning.', ...
        "popupObstacle",(0:1:42)');
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@popupObstaclePolygons,'Pop-up obstacle');
    suite(14) = scenario;

    %% 15. Goal interception
    scenario = makeBaseScenario("Goal interception", ...
        'The goal moves across the far side of the map.', ...
        "goalInterception",(0:1:44)');
    time_s = scenario.data.time_s;
    scenario.goalAzEl_deg = [14+zeros(size(time_s)), ...
        6*sin(2*pi*time_s/24)];
    suite(15) = scenario;

    %% 16. Rotating narrow slot
    scenario = makeBaseScenario("Rotating narrow slot", ...
        'A narrow central opening rotates while the path approaches.', ...
        "rotatingSlot",(0:1:48)');
    scenario.data = makeDynamicData( ...
        scenario.data.time_s,@rotatingSlotPolygons,'Rotating narrow slot');
    suite(16) = scenario;

    %% 17. Clearance squeeze
    scenario = makeBaseScenario("Clearance squeeze", ...
        'The only feasible passage is the perfectly centered grid row.', ...
        "clearanceSqueeze",(0:1:42)');
    scenario.options.staticPolygons = { ...
        rectanglePolygon(-20,20,0.75,12), ...
        rectanglePolygon(-20,20,-12,-0.75)};
    suite(17) = scenario;

    %% 18. Impossible maze
    scenario = makeBaseScenario("Impossible maze", ...
        'A solid wall separates start and goal for the entire horizon.', ...
        "impossibleMaze",(0:1:35)');
    scenario.expectedSuccess = false;
    scenario.options.staticPolygons = {rectanglePolygon(-1,1,-12,12)};
    suite(18) = scenario;
end


function scenario = makeBaseScenario(name,setup,metric,time_s)
    scenario = struct;
    scenario.name = string(name);
    scenario.slug = metric;
    scenario.setup = string(setup);
    scenario.metric = string(metric);
    scenario.data = emptyData(time_s,name);
    scenario.startAzEl_deg = [-18,0];
    scenario.goalAzEl_deg = [18,0];
    scenario.expectedSuccess = true;
    scenario.options = baseOptions(time_s);
end


function options = baseOptions(time_s)
    options = struct;
    options.azLim_deg = [-20,20];
    options.elLim_deg = [-12,12];
    options.gridStep_deg = [1,1];
    options.azRate_deg_s = 2;
    options.elRate_deg_s = 2;
    options.startTime_s = time_s(1);
    options.planningHorizon_s = time_s(end)-time_s(1);
    options.goalHold_s = 0;
    options.clearance_deg = 0;
    options.temporalPadding_s = 0;
    options.staticPolygons = {};
    options.preventCornerCutting = true;
    options.episodes = 20;
    options.minimumEpisodesPerLearner = 20;
    options.earlyStopSuccessStreak = 10;
    options.useParallel = false;
    options.randomSeed = 31;
    options.guidanceWeight = 3;
    options.progressRewardWeight = 2;
    options.turnPenalty = 0.4;
    options.smoothPath = true;
    options.smoothingMaxLookahead = 120;
    options.verbose = false;
end


function data = emptyData(time_s,name)
    time_s = time_s(:);
    data = struct;
    data.targetName = char(name);
    data.time_s = time_s;
    data.az_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.el_deg = repmat({zeros(0,1)},numel(time_s),1);
    data.status = repmat("NotVisible",numel(time_s),1);
end


function data = makeDynamicData(time_s,generator,name)
    data = emptyData(time_s,name);
    for timeIndex = 1:numel(time_s)
        polygons = generator(time_s(timeIndex));
        [data.az_deg{timeIndex},data.el_deg{timeIndex}] = ...
            packPolygons(polygons);
        if ~isempty(polygons)
            data.status(timeIndex) = "Visible";
        end
    end
end


function [az_deg,el_deg] = packPolygons(polygons)
    if isempty(polygons)
        az_deg = zeros(0,1);
        el_deg = zeros(0,1);
        return
    end
    lengths = cellfun(@(polygon) size(polygon,1),polygons);
    totalLength = sum(lengths)+numel(polygons)-1;
    az_deg = NaN(totalLength,1);
    el_deg = NaN(totalLength,1);
    firstIndex = 1;
    for polygonIndex = 1:numel(polygons)
        polygon = polygons{polygonIndex};
        lastIndex = firstIndex+size(polygon,1)-1;
        az_deg(firstIndex:lastIndex) = polygon(:,1);
        el_deg(firstIndex:lastIndex) = polygon(:,2);
        firstIndex = lastIndex+2;
    end
end


function polygon = rectanglePolygon(minAz,maxAz,minEl,maxEl)
    polygon = [minAz,minEl;maxAz,minEl;maxAz,maxEl; ...
        minAz,maxEl;minAz,minEl];
end


function polygons = verticalWallWithGap(centerAz,gapCenterEl,gapHalfHeight)
    halfWidth = 0.65;
    polygons = { ...
        rectanglePolygon(centerAz-halfWidth,centerAz+halfWidth, ...
        -12,gapCenterEl-gapHalfHeight), ...
        rectanglePolygon(centerAz-halfWidth,centerAz+halfWidth, ...
        gapCenterEl+gapHalfHeight,12)};
end


function polygon = rotatedRectangle(center,lengthValue,widthValue,angle_deg)
    local = 0.5*[ ...
        -lengthValue,-widthValue;lengthValue,-widthValue; ...
        lengthValue,widthValue;-lengthValue,widthValue; ...
        -lengthValue,-widthValue];
    rotation = [cosd(angle_deg),-sind(angle_deg); ...
        sind(angle_deg),cosd(angle_deg)];
    polygon = local*rotation'+center;
end


function polygons = pistonPolygons(time_s)
    centersAz = [-10,-4,4,10];
    polygons = cell(1,numel(centersAz));
    for index = 1:numel(centersAz)
        phase = (index-1)*pi;
        centerEl = 5.5*sin(2*pi*time_s/14+phase);
        polygons{index} = rectanglePolygon( ...
            centersAz(index)-0.8,centersAz(index)+0.8, ...
            centerEl-4,centerEl+4);
    end
end


function polygons = crossingTrafficPolygons(time_s)
    polygons = {};
    firstEl = time_s-12;
    secondEl = time_s-24;
    if abs(firstEl) <= 11
        polygons{end+1} = rectanglePolygon( ...
            -7,-5,firstEl-1.8,firstEl+1.8);
    end
    if abs(secondEl) <= 11
        polygons{end+1} = rectanglePolygon( ...
            5,7,secondEl-1.8,secondEl+1.8);
    end
end


function polygons = timedGatePolygons(time_s)
    gateOpen = time_s >= 20 && time_s <= 26;
    if ~gateOpen
        polygons = {rectanglePolygon(-0.8,0.8,-2.2,2.2)};
    else
        polygons = {};
    end
end


function polygons = sweepingArmPolygons(time_s)
    angle_deg = 62*sin(2*pi*time_s/24);
    polygons = {rotatedRectangle([0,0],18,1.1,angle_deg)};
end


function polygons = movingTunnelPolygons(time_s)
    centerEl = 3*sin(2*pi*time_s/28);
    halfGap = 3.1;
    polygons = { ...
        rectanglePolygon(-20,20,centerEl+halfGap,12), ...
        rectanglePolygon(-20,20,-12,centerEl-halfGap)};
end


function polygons = shrinkingFunnelPolygons
    upper = [-20,12;20,12;20,1.6;-20,8;-20,12];
    lower = [-20,-12;-20,-8;20,-1.6;20,-12;-20,-12];
    polygons = {upper,lower};
end


function polygons = oncomingTrafficPolygons(time_s)
    centerAz = 15-0.75*time_s;
    if centerAz < -19
        polygons = {};
    else
        polygons = {rectanglePolygon( ...
            centerAz-1.2,centerAz+1.2,-1.3,1.3)};
    end
end


function polygons = twoWayTrapPolygons(time_s)
    if time_s >= 8
        polygons = {rectanglePolygon(1,8,3,12)};
    else
        polygons = {};
    end
end


function polygons = timedShortcutPolygons(time_s)
    if time_s < 20
        polygons = {rectanglePolygon(-1,1,-2.1,2.1)};
    else
        polygons = {};
    end
end


function polygons = doubleGatePolygons(time_s)
    firstOpen = (time_s >= 9 && time_s <= 15) || ...
        (time_s >= 28 && time_s <= 34);
    secondOpen = time_s >= 18 && time_s <= 26;
    polygons = {};
    if ~firstOpen
        polygons{end+1} = rectanglePolygon(-5.7,-4.3,-2.2,2.2);
    end
    if ~secondOpen
        polygons{end+1} = rectanglePolygon(4.3,5.7,-2.2,2.2);
    end
end


function polygons = popupObstaclePolygons(time_s)
    if time_s >= 7 && time_s <= 18
        polygons = {rectanglePolygon(-2,3,-2.5,2.5)};
    else
        polygons = {};
    end
end


function polygons = rotatingSlotPolygons(time_s)
    angle_deg = 70*sin(2*pi*time_s/30);
    direction = [cosd(angle_deg),sind(angle_deg)];
    polygons = { ...
        rotatedRectangle(-7*direction,10,1.2,angle_deg), ...
        rotatedRectangle(7*direction,10,1.2,angle_deg)};
end
