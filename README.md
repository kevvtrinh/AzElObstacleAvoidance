# Fast Q-learning azimuth/elevation obstacle-avoidance planner

This package implements time-dependent tabular Q-learning for moving keep-out
polygons in azimuth/elevation space. It supports waiting, moving goals, static
and dynamic obstacles, clearance inflation, cable-angle branches, periodic
azimuth topology, collision-checked smoothing, deterministic graph-search
fallback, C2-continuous quintic command trajectories, explicit rate,
acceleration, and jerk limits, diagnostics, plots, animations, and a
deterministic planner gauntlet.

![Complete planner gauntlet](gauntlet_complete_suite.gif)

## Quick start

Clone or download the repository, keep its folder structure intact, make the
repository root the MATLAB current folder, and run:

```matlab
runQLearningAzElExample
```

The example contains a deforming irregular obstacle and a temporary exclusion
curtain. The policy must wait for the curtain to clear before reaching the
destination.

Run strict static analysis, all unit tests, all 18 planner scenarios, all seven
azimuth-wraparound checks, the dynamic-mask smoke test, and the 5.25-turn
spiral-to-center challenge with:

```matlab
report = runGauntlet;
```

Run the complete deterministic 18-scenario planner suite with:

```matlab
suiteReport = runPlannerGauntletSuite;
```

`runPlannerGauntletSuite` throws if any scenario fails. To collect a report
without throwing, use:

```matlab
suiteReport = runPlannerGauntletSuite([],false);
```

Run only the MATLAB unit tests with:

```matlab
results = runtests('testQLearningAzElPlanner.m');
assertSuccess(results);
```

Create one fixed-canvas GIF containing the complete gauntlet suite with:

```matlab
gifFile = createGauntletSuiteGif;
```

The output is `gauntlet_complete_suite.gif`. `createGauntletGif` is retained as
the legacy two-run moving-mask/spiral renderer.

## The 18-scenario planner suite

`createPlannerGauntletSuite` builds the same fixed-seed scenarios on every run.
`runPlannerGauntletSuite` checks expected success or failure, an independent
continuous audit of the analytic command curve, rate/acceleration/jerk limits,
C2 continuity, and the scenario-specific behavior below.

| Scenario | Setup | Scenario-specific check |
| --- | --- | --- |
| Narrow hallway | Long corridor only slightly wider than the safe path | Centered travel with no more than two smoothed turns |
| Piston slalom | Alternating pillars move vertically across the route | Timed weave with a bounded smoothed turn count |
| Crossing traffic | Obstacles cross perpendicular to the route | Observable yielding or lateral use of a safe gap |
| Timed gate | A doorway opens only from 20 s through 26 s | Waits and crosses during the open interval |
| Sweeping arm | A long obstacle rotates across the direct route | Waits or maneuvers laterally for a safe passage |
| Moving tunnel | Two moving walls form a shifting corridor | Remains inside the time-varying feasible region |
| Shrinking funnel | A wide entrance narrows to a tight exit | Aligns before reaching the narrow section |
| Offset keyholes | Three narrow openings have different centerlines | Crosses all openings with bounded zigzagging |
| Oncoming traffic | An obstacle approaches through a narrow lane | Uses one consistent avoidance side |
| Two-way trap | Both detours initially look open; one later closes | Rejects the future-closed branch |
| U-shaped trap | The nearby goal is behind a U-shaped wall | Executes the required detour instead of stalling at a local minimum |
| Timed shortcut | A short doorway opens after a wait; a long route stays open | Waits for the shortcut and arrives within the time bound |
| Double gate | Two timed doors must be crossed in sequence | Crosses each gate in its own open window and in the correct order |
| Pop-up obstacle | An obstacle appears later on the nominal path | Produces an observable, stable time-dependent response |
| Goal interception | The goal moves across the far side of the map | Ends on the moving goal at the arrival time |
| Rotating narrow slot | A narrow opening rotates during the approach | Waits or aligns laterally for safe entry |
| Clearance squeeze | Only the exact center grid row is feasible | Holds the centerline with zero smoothed turns |
| Impossible maze | A solid wall separates start and goal | Correctly reports the route as unreachable |

The impossible-maze case is a pass only when the planner returns failure. It
guards against false success and endless oscillation.

## Spiral-to-center gauntlet

`createSpiralGauntletScenario` creates a 5.25-cycle Archimedean spiral ribbon.
The start is outside the outer tip and the goal is at `[0,0]`. The 1-degree
planner grid uses 0.35 degrees of clearance and a 520-second horizon.

The gauntlet requires all of the following:

- successful center arrival;
- a collision-free path under continuous segment checks; and
- at least 1440 degrees of **net unwrapped winding**, equivalent to four full
  revolutions.

Net winding is measured from the difference between the first and last
unwrapped polar angles. Back-and-forth angular oscillation therefore cannot
artificially satisfy the four-cycle requirement.

## State and actions

Each Q-learning state is:

```text
(azimuth grid cell, elevation grid cell, time index)
```

Time is part of the state, so the policy can distinguish "move now" from "wait
until this obstacle passes." The actions are wait plus the eight neighboring
grid moves.

Before selecting an action, the planner rejects transitions that:

- leave the configured azimuth/elevation domain;
- exceed the per-sample azimuth or elevation slew rate;
- enter a moving mask or static keep-out polygon;
- cut diagonally between blocked corner cells;
- cross a static polygon edge or violate its continuous clearance; or
- intersect the swept region of a moving polygon between consecutive frames.

This keeps unsafe exploration out of the training loop instead of relying on a
collision penalty to teach safety.

## Continuous and swept collision safety

Grid-cell occupancy is only the first safety layer. The planner also checks the
line segment for each move against the actual static polygon edges. A thin wall
between two otherwise free cell centers is therefore still detected, and
`clearance_deg` is enforced using segment-to-polygon distance.

For moving obstacles, corresponding polygon frames are joined into conservative
swept polygons. Endpoint polygons and the quadrilaterals swept by corresponding
edges are included, then buffered by cell and clearance margins before an
action is accepted. For the strongest swept-motion guarantee, keep each moving
contour's vertex count and ordering consistent from one frame to the next.

The waypoint shortcut pass uses the same segment-level safety model. It is then
converted into fixed-schedule quintic Bezier fillets. Each fillet matches
position, velocity, and acceleration at its joins; no global retiming is used,
so a gate crossing is not silently moved into a different obstacle interval.
The analytic curve is recursively certified against interpolated moving
polygons, rather than treating dense plotted samples as proof of safety.
The same audit honors `temporalPadding_s`, so smoothing cannot enter an
adjacent-time obstacle reservation that the discrete planner avoided.

Rate, acceleration, and jerk are certified from the Bezier derivative control
points. If no curve can satisfy collision and actuator limits at the existing
schedule, the planner tries the remaining ranked routes and graph-search
fallback. If none produces a certified command, `result.success` and
`result.trajectory.success` are false while `result.routeSuccess` records
whether a raw diagnostic route reached the goal.
When `goalHold_s` is positive, the trajectory must also reach zero velocity
and acceleration before that stationary hold begins. Set
`trajectoryRequireEndpointRest=true` when a command must start and end at rest;
this enforces zero velocity and acceleration at both endpoints. It defaults to
`false` because fixed obstacle timing can make a rest-to-rest ramp infeasible
when the first or final raw leg already uses all available slew rate.
Regression tests cover moving obstacles that cross between frames, thin static
obstacles between grid points, coarse 90-degree corners, and dynamic slalom
corners.

## Use existing `azElData`

```matlab
startAzEl_deg = [-80,30];
goalAzEl_deg = [120,80];

options.azLim_deg = [-180,180];
options.elLim_deg = [0,90];
options.gridStep_deg = [2,2];
options.azRate_deg_s = 10;
options.elRate_deg_s = 10;
options.startTime_s = azElData.time_s(1);
options.planningHorizon_s = 120;
options.goalHold_s = 0;
options.clearance_deg = 1;
options.temporalPadding_s = 0.5;

options.azimuthTopology = "mechanical";
options.goalAzimuthIsWrapped = false;
options.cableReserve_deg = 0;

options.episodes = 4000;
options.useParallel = true;
options.numLearners = 0;       % Use all available workers
options.randomSeed = 7;
options.maxQTableMB = 512;

options.turnPenalty = 0.75;
options.smoothPath = true;
options.smoothingMaxLookahead = 250;

options.generateSmoothTrajectory = true;
options.trajectoryRequireEndpointRest = false; % Set true for rest-to-rest commands
options.trajectorySampleTime_s = 0.1;
options.maxAzAcceleration_deg_s2 = 40;
options.maxElAcceleration_deg_s2 = 40;
options.maxAzJerk_deg_s3 = 200;
options.maxElJerk_deg_s3 = 200;
options.verbose = true;

result = planAzElQLearning( ...
    azElData,startAzEl_deg,goalAzEl_deg,options);

plotAzElQLearningResult(azElData,result);
animateAzElQLearningPlan(azElData,result);
```

`result.path` retains the time-aligned planner polyline for reproducibility and
scenario metrics. `result.trajectory` is the executable dense command history:

```matlab
commandTime_s = result.trajectory.time_s;
commandAz_deg = result.trajectory.az_deg;
commandEl_deg = result.trajectory.el_deg;

assert(result.success);
assert(result.trajectory.success);
assert(result.diagnostic.trajectoryCollisionFree);
assert(result.diagnostic.trajectoryKinematicallyFeasible);
```

The trajectory also contains axis rate, acceleration, and jerk histories plus
the exact piecewise-quintic Bezier segments used by the independent auditor.
If acceleration or jerk limits are omitted, their defaults are respectively
four and twenty times the corresponding axis slew-rate limit. The plotting,
animation, and gauntlet GIF functions automatically prefer this smooth command
trajectory.

The expected data fields are:

```matlab
azElData.targetName
azElData.time_s       % N-by-1, uniformly spaced
azElData.az_deg       % N-by-1 cell array of polygon azimuth vertices
azElData.el_deg       % N-by-1 cell array of polygon elevation vertices
azElData.status       % optional
```

NaNs may separate multiple contours in one frame. The occupancy builder fills
the supplied polygons with `inpolygon`; it does not replace them with bounding
boxes. Polygons that cross the `-180/180` seam are unwrapped and rasterized on
the appropriate seam branches.

For time-accurate animation, corresponding obstacle vertices are interpolated
between data frames. Keep contour ordering and vertex ordering stable whenever
an obstacle persists. If that topology changes, the renderer shows both
endpoint geometries rather than visually inventing a correspondence.

An `N`-by-2 goal history aligned with `azElData.time_s` is supported for moving
goal interception.

## Mechanical and periodic azimuth topology

The default topology is `"mechanical"`. Its azimuth limits are physical cable
limits, and the two ends of the grid are not adjacent. For a wrapped target
command on a multi-turn mechanical range, enable equivalent goal branches:

```matlab
options.azimuthTopology = "mechanical";
options.azLim_deg = [-540,540];
options.cableReserve_deg = 10;
options.goalAzimuthIsWrapped = true;

result = planAzElQLearning(azElData,[370,30],[10,60],options);
```

The planner considers `10 + 360*k` inside the effective cable limits.

Use periodic topology when azimuth is genuinely circular and crossing the seam
is physically allowed:

```matlab
options.azimuthTopology = "periodic";
options.azLim_deg = [-180,180];  % Must span exactly 360 degrees
options.gridStep_deg = [1,1];   % Azimuth step must divide 360
options.cableReserve_deg = 0;   % Cable reserve is incompatible with periodic mode
options.goalAzimuthIsWrapped = true;
```

Periodic mode makes the first and last azimuth cells neighbors and omits the
duplicated upper seam endpoint. `result.path.az_deg` is a continuous unwrapped
mechanical history; `result.path.azWrapped_deg` contains the equivalent values
inside the configured 360-degree plotting interval. The same distinction is
available in `result.trajectory.az_deg` and
`result.trajectory.azWrapped_deg`.

The helpers below use shortest-arc degree semantics:

```matlab
delta_deg = shortestAzimuthDeltaDeg(from_deg,to_deg);
sample_deg = interpolateAzimuthDeg(from_deg,to_deg,fraction);
continuous_deg = unwrapAzimuthDeg(wrapped_deg,reference_deg);
```

Seven dedicated unit tests verify:

1. short forward crossing from 179 degrees to -179 degrees;
2. the reverse crossing;
3. interpolation through 180 degrees instead of 0 degrees;
4. constant seam-crossing slew rate;
5. zero artificial acceleration and jerk;
6. equivalence of 180 degrees and -180 degrees; and
7. continuity through repeated seam crossings.

## Parallel training and deterministic fallback

With `useParallel=true`, episodes are divided among independent learners with
different deterministic seeds. Each large Q-table remains on its worker; only
candidate paths and compact statistics return to the client. Final selection
prefers successful paths, then earlier arrival, fewer turns, and shorter path
length.

If the Parallel Computing Toolbox or a multi-worker pool is unavailable, the
planner emits a warning, runs serially, and records the reason in
`result.diagnostic.parallelFallbackReason`.

Training may stop early after the configured minimum episode count and success
streak. Requested and completed episodes, success rate, and stop reasons are
recorded in `result.diagnostic`.

If all learned policies fail, the planner evaluates a deterministic
time-expanded graph-search fallback with the same collision checks. A
successful learned policy remains preferred; learner index 0 and
`selectedPolicySource == "graphFallback"` identify the fallback.

Set `useParallel=false` for reproducible serial debugging and gauntlet runs.

## Speed and memory

The single-precision Q-table requires approximately:

```text
numAz * numEl * numTimeSamples * 9 actions * 4 bytes per learner
```

The largest controls are `planningHorizon_s`, `gridStep_deg`, `episodes`,
`numLearners`, and `maxQTableMB`. Start real-data experiments with a short time
window and a 2-degree grid, then refine the grid only after the policy succeeds
reliably.

Each grid step must be reachable in one time sample:

```matlab
options.gridStep_deg(1) <= options.azRate_deg_s * dt_s
options.gridStep_deg(2) <= options.elRate_deg_s * dt_s
```

Otherwise only the wait action may remain valid.

## Reward controls

- `stepPenalty` favors earlier arrival.
- `waitPenalty` discourages unnecessary waiting.
- `progressRewardWeight` rewards reduction in goal distance.
- `guidanceWeight` guides early exploration toward the goal.
- `turnPenalty` discourages direction changes.
- `goalReward` rewards a valid terminal arrival.

## Code organization

User-facing functions remain at the repository root for backward compatibility.
Their implementations are grouped in MATLAB namespaces under `+azel`:

| Namespace | Responsibility |
| --- | --- |
| `azel.planning` | Planning pipeline, Q-learning, graph fallback, and result assembly |
| `azel.mapping` | Occupancy-grid construction and clearance inflation |
| `azel.trajectory` | Fixed-time C2 Bezier command generation and kinematic bounds |
| `azel.audit` | Independent polyline and analytic-trajectory certification |
| `azel.geometry` | Seam-aware angles and time-interpolated obstacle geometry |
| `azel.visualization` | Safe display-path selection, plotting, and animation |

For example, existing code can continue calling `planAzElQLearning(...)`; the
facade delegates to `azel.planning.planAzElQLearning(...)`. Only the repository
root belongs on the MATLAB path; do not add `+azel` directories directly.

See [ARCHITECTURE.md](ARCHITECTURE.md) for module boundaries, data contracts,
and extension guidance. MATLAB's `help Contents` and `help azel` commands also
provide a categorized function index.

## Main files

- `planAzElQLearning.m` - stable public facade for the planning pipeline
- `buildAzElOccupancy.m`, `smoothAzElTrajectory.m`, and the audit/geometry/
  visualization entry points - stable compatibility facades
- `+azel/` - namespaced implementation modules, each with a `Contents.m` index
- `Contents.m` - categorized public API index
- `shortestAzimuthDeltaDeg.m`, `interpolateAzimuthDeg.m`, and
  `unwrapAzimuthDeg.m` - public wrapped-angle facades
- `createPlannerGauntletSuite.m` - deterministic definitions for all 18 planner
  scenarios
- `runPlannerGauntletSuite.m` - scenario execution and behavior assertions
- `ARCHITECTURE.md` - dependency rules, struct contracts, and maintenance guide
- `createSpiralGauntletScenario.m` - 5.25-cycle spiral-to-center challenge
- `runAzimuthWraparoundGauntlet.m` - seven named seam and wrapped-angle checks
- `runGauntlet.m` - complete static-analysis, unit-test, scenario, seam,
  dynamic-mask, and spiral entry point
- `createGauntletSuiteGif.m` - complete-suite fixed-canvas GIF renderer
- `createGauntletGif.m` - legacy two-run GIF renderer
- `testQLearningAzElPlanner.m` - core, continuous-trajectory, safety, and seven
  wraparound regressions
- `plotAzElQLearningResult.m` - complete mask sweep and final path
- `animateAzElQLearningPlan.m` - current mask, executed path, and optional GIF
- `createQLearningExampleAzElData.m` - self-contained moving-mask example
- `runQLearningAzElExample.m` - runnable demonstration

Acceleration and jerk are intentionally not added to the tabular Q state,
which would multiply the state space. Instead, the selected time-aligned route
is converted into an analytic C2 command trajectory and independently
certified. This keeps the learner compact while still making actuator limits a
required property of the command output.
