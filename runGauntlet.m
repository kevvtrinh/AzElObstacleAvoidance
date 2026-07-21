function report = runGauntlet
%RUNGAUNTLET Run every static, unit, scenario, spiral, and seam check.
%   REPORT = RUNGAUNTLET fails immediately if CHECKCODE reports an issue,
%   a unit test fails, a scenario misses its metric, an independent path
%   audit fails, or an azimuth-wraparound check fails.

    projectFolder = fileparts(mfilename('fullpath'));
    previousFolder = pwd;
    previousFigureVisibility = get(groot,'DefaultFigureVisible');
    cleanup = onCleanup(@() restoreEnvironment( ...
        previousFolder,previousFigureVisibility));
    cd(projectFolder);
    set(groot,'DefaultFigureVisible','off');

    fprintf('=== Q-learning az/el planner gauntlet ===\n');
    files = dir(fullfile(projectFolder,'*.m'));
    analysis = cell(numel(files),1);
    issueCount = 0;
    for fileIndex = 1:numel(files)
        analysis{fileIndex} = checkcode( ...
            fullfile(projectFolder,files(fileIndex).name),'-id');
        issueCount = issueCount+numel(analysis{fileIndex});
    end
    if issueCount > 0
        fprintf('Static analysis found %d issue(s):\n',issueCount);
        for fileIndex = 1:numel(files)
            messages = analysis{fileIndex};
            for messageIndex = 1:numel(messages)
                fprintf('  %s:%d %s [%s]\n',files(fileIndex).name, ...
                    messages(messageIndex).line, ...
                    messages(messageIndex).message, ...
                    messages(messageIndex).id);
            end
        end
        error('runGauntlet:StaticAnalysisFailed', ...
            'Static analysis must be clean before runtime tests are run.');
    end
    fprintf('PASS static analysis (%d files)\n',numel(files));

    suite = testsuite(fullfile(projectFolder, ...
        'testQLearningAzElPlanner.m'));
    testResults = run(suite);
    assertSuccess(testResults);
    fprintf('PASS unit tests (%d tests)\n',numel(testResults));

    scenarioSuite = runPlannerGauntletSuite([],true);
    fprintf('PASS planner scenario suite (%d/%d)\n', ...
        scenarioSuite.numPassed,scenarioSuite.numScenarios);

    wraparound = runAzimuthWraparoundGauntlet(true);
    fprintf('PASS azimuth wraparound suite (%d/%d)\n', ...
        wraparound.numPassed,wraparound.numTests);

    [smokeResult,smokeScenario] = runHeadlessSmokeTest;
    smokeAudit = auditPlannerPath(smokeScenario,smokeResult);
    if ~smokeResult.success || ~smokeResult.diagnostic.collisionFree || ...
            ~smokeAudit.collisionFree
        error('runGauntlet:SmokeTestFailed', ...
            'The headless end-to-end example did not produce a safe path.');
    end
    fprintf(['PASS headless smoke test (arrival %.1f s, wait %.1f s, ' ...
        '%d turns)\n'],smokeResult.diagnostic.arrivalTime_s, ...
        smokeResult.diagnostic.waitTime_s, ...
        smokeResult.diagnostic.turnCountAfterSmoothing);

    [spiralResult,spiralWinding_deg,spiralScenario,spiralAudit] = ...
        runSpiralSmokeTest;
    fprintf(['PASS spiral-center smoke test (arrival %.1f s, ' ...
        'angular travel %.1f deg, source %s)\n'], ...
        spiralResult.diagnostic.arrivalTime_s,spiralWinding_deg, ...
        spiralResult.diagnostic.selectedPolicySource);

    report = struct;
    report.passed = scenarioSuite.passed && wraparound.passed && ...
        smokeAudit.collisionFree && spiralAudit.collisionFree;
    report.staticAnalysisIssueCount = issueCount;
    report.testResults = testResults;
    report.smokeDiagnostic = smokeResult.diagnostic;
    report.smokeResult = smokeResult;
    report.smokeAudit = smokeAudit;
    report.scenarioSuite = scenarioSuite;
    report.wraparound = wraparound;
    report.spiralDiagnostic = spiralResult.diagnostic;
    report.spiralResult = spiralResult;
    report.spiralScenario = spiralScenario;
    report.spiralAudit = spiralAudit;
    report.spiralWinding_deg = spiralWinding_deg;
    report.numNamedChecks = scenarioSuite.numScenarios+1+ ...
        wraparound.numTests;
    report.numNamedChecksPassed = scenarioSuite.numPassed+ ...
        double(spiralAudit.collisionFree)+wraparound.numPassed;
    fprintf('ALL %d NAMED GAUNTLET CHECKS PASSED\n', ...
        report.numNamedChecks);
    clear cleanup
end


function [result,scenario] = runHeadlessSmokeTest
    time_s = (0:0.5:75)';
    data = createQLearningExampleAzElData(time_s);
    options = struct;
    options.azLim_deg = [-100,100];
    options.elLim_deg = [5,85];
    options.gridStep_deg = [2,2];
    options.azRate_deg_s = 12;
    options.elRate_deg_s = 12;
    options.startTime_s = 0;
    options.planningHorizon_s = 75;
    options.goalHold_s = 2;
    options.clearance_deg = 1.5;
    options.temporalPadding_s = 0.5;
    options.episodes = 160;
    options.minimumEpisodesPerLearner = 40;
    options.earlyStopSuccessStreak = 15;
    options.useParallel = false;
    options.randomSeed = 7;
    options.smoothPath = true;
    options.verbose = false;
    result = planAzElQLearning(data,[-80,20],[80,70],options);
    scenario = struct('data',data,'startAzEl_deg',[-80,20], ...
        'goalAzEl_deg',[80,70],'options',options);
end


function [result,winding_deg,scenario,audit] = runSpiralSmokeTest
    scenario = createSpiralGauntletScenario;
    result = planAzElQLearning(scenario.data, ...
        scenario.startAzEl_deg,scenario.goalAzEl_deg,scenario.options);
    if ~result.success || ~result.diagnostic.collisionFree
        error('runGauntlet:SpiralPlanningFailed', ...
            'The planner did not safely reach the center of the spiral.');
    end

    audit = auditPlannerPath(scenario,result);
    if ~audit.collisionFree
        error('runGauntlet:SpiralIndependentAuditFailed', ...
            'The spiral path failed its independent audit: %s.', ...
            audit.message);
    end

    finalDistance_deg = hypot( ...
        result.path.az_deg(end)-scenario.centerAzEl_deg(1), ...
        result.path.el_deg(end)-scenario.centerAzEl_deg(2));
    goalTolerance_deg = max(scenario.options.gridStep_deg)/2+1e-10;
    if finalDistance_deg > goalTolerance_deg
        error('runGauntlet:SpiralCenterMissed', ...
            'The spiral path stopped %.3g deg from the center.', ...
            finalDistance_deg);
    end

    relativeAz_deg = result.path.az_deg-scenario.centerAzEl_deg(1);
    relativeEl_deg = result.path.el_deg-scenario.centerAzEl_deg(2);
    radius_deg = hypot(relativeAz_deg,relativeEl_deg);
    measure = radius_deg > max(scenario.options.gridStep_deg);
    pathAngle_rad = unwrap(atan2(relativeEl_deg(measure), ...
        relativeAz_deg(measure)));
    winding_deg = abs(rad2deg(pathAngle_rad(end)-pathAngle_rad(1)));
    if winding_deg < scenario.minimumWinding_deg
        error('runGauntlet:InsufficientSpiralWinding', ...
            ['The path reached the center but accumulated only %.1f deg ' ...
            'of angular travel; at least %.1f deg is required.'], ...
            winding_deg,scenario.minimumWinding_deg);
    end
end


function restoreEnvironment(folder,figureVisibility)
    cd(folder);
    set(groot,'DefaultFigureVisible',figureVisibility);
end
