# Architecture

The repository exposes a small, stable MATLAB API at the root and keeps the
implementation in the `azel` package. This separation lets existing scripts
continue to work while making ownership and dependencies explicit.

## Module layout

```text
repository root
├── planAzElQLearning.m            public compatibility facade
├── buildAzElOccupancy.m           public compatibility facade
├── smoothAzElTrajectory.m         public compatibility facade
├── audit*.m / angle helpers       public compatibility facades
├── plotting and animation facades
├── gauntlets, tests, and examples
└── +azel
    ├── +planning                  route planning and result assembly
    ├── +mapping                   occupancy-grid construction
    ├── +trajectory                executable C2 command generation
    ├── +audit                     independent safety certification
    ├── +geometry                  angular and obstacle geometry
    └── +visualization             result presentation
```

Keep only the repository root on the MATLAB path. MATLAB discovers package
directories automatically; adding a `+azel` directory itself to the path can
make name resolution ambiguous.

## Planning pipeline

1. `azel.planning.planAzElQLearning` validates options and selects the planning
   time window.
2. `azel.mapping.buildAzElOccupancy` rasterizes static and moving keep-out
   contours, including clearance and temporal padding.
3. The planner builds time-dependent safe transitions and trains independent
   Q-learning candidates. A deterministic time-expanded graph search remains
   the route-completeness fallback.
4. Candidate polylines are shortcut only when continuous route checks remain
   safe.
5. `azel.trajectory.smoothAzElTrajectory` replaces velocity discontinuities
   with fixed-time quintic Bezier blends.
6. `azel.audit.auditAzElTrajectory` independently certifies collision
   clearance, mechanical limits, rate, acceleration, jerk, C2 continuity, and
   any requested goal hold.
7. The public result is assembled with both route and executable-command
   diagnostics.

## Stable contracts

### Result

- `result.routeSuccess` and `result.routeReachedGoal` describe the discrete
  route.
- When smooth commands are requested, `result.success` is true only when an
  analytic command was generated and its independent audit passed.
- `result.path` is the timed discrete/shortcut route.
- `result.trajectory` is the sampled command plus its authoritative analytic
  `segments` array.
- `result.trajectoryAudit` contains the command certification report.

### Analytic segment

Every trajectory segment contains:

- `startTime_s` and `endTime_s` absolute time bounds;
- `controlPoints_deg`, a 6-by-2 quintic Bezier control polygon ordered as
  `[azimuth, elevation]`;
- `kind`, such as `line`, `wait`, or a transition type.

### Audit

Audit structs expose collision, hold, kinematic, mechanical-bound, and C2
flags; observed and certified minimum clearance; kinematic extrema; and the
offending segment/time/obstacle type when certification fails.

## Dependency rules

- Package implementations use qualified cross-module calls such as
  `azel.geometry.shortestAzimuthDeltaDeg`.
- Root facades may call package implementations, but package implementations
  do not call root facades.
- The Q-learning loop and graph fallback do not depend on plotting or GIF
  code.
- The two auditors remain independent of planner decisions. Similar polygon
  helpers are intentionally not merged unless their conservative behavior is
  proven equivalent.
- Visualization uses `selectAzElDisplayPath`, which suppresses an unaudited raw
  route when a smooth executable command was requested but could not be
  certified.

## Making changes

Place new implementation code in the package that owns its behavior. Add a
root facade only when the function is intended as a stable public entry point.
Document package additions in the relevant `Contents.m` file.

Run `runGauntlet` before committing. It recursively applies `checkcode` to root
and package files, executes all unit tests, the 18-scenario planner suite, seven
azimuth-wraparound checks, the end-to-end smoke test, and the spiral challenge.
