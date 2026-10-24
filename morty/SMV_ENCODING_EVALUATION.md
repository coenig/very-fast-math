# Task 7 SMV Encoding Experiments

Date: 2026-10-24. Linux, nuXmv 2.2.0, MathSAT bounded falsification.

## Recommendation

The strongest prototype combines a directly represented integer lane coordinate,
division-free lane-jump constraints, acceleration projection, and specialization of
provably stationary vehicles. It reduced the sum of six matched-query median times
from 25.80 s to 3.83 s, a 6.74x improvement on this small fixture set.

The combined encoding is now implemented in the production generator and enabled
by `#TEMPLATE.UCD_TASK7_COMPACT` in [envmodel_config.tpl.json](envmodel_config.tpl.json).
Bounding-box priorities, all 16 fallbacks, shortcuts, physics, and solver algorithms
and bounds remain unchanged. The original rewrite experiment is retained in
[evaluate_smv_encodings.py](evaluate_smv_encodings.py) for comparisons.

## Production Baseline

Run normally after rebuilding the native library:

```bash
cmake --build build --target runTests --parallel 8
.venv/bin/python -m morty.morty --live --force_head
```

Omit `--force_head` for headless operation. Set `UCD_TASK7_COMPACT` to `false` and
restart to restore the original encoding. The setting defaults to `false` when
absent; other master templates and classical non-UCD mode remain unchanged.

Despite its task-7 name, the option does not require five vehicles, two lanes, a
particular experiment number, section length, scaling, or speed/acceleration limits.
Lane bounds, linear jump constraints, and velocity transitions use the configured
values. It requires concrete ego-less UCD with simple lane changes, positive integer
technical-lane and jump counts, and distinct scaled jump-velocity thresholds. The
acceleration variables are projected out: specifications that observe acceleration
must use the legacy mode. Performance outside the measured profile is not established.

Parked-state elimination is conservative and optional within compact mode. It needs
an explicit stationary-velocity invariant, a fixed single section, an in-section
initial position, and a positive scaled stationary lane-hold threshold. Otherwise
that vehicle retains dynamic state. Negative initial positions are not frozen. The initial
previous position, collision exemption at counter zero, and transition obligations
are preserved.

Configuration choices use native vfmacro conditionals. Only per-query parked
constants use the existing `-pre cpp` stage: the native input writer supplies them
before the model includes, avoiding Python model rewrites or per-cycle template
regeneration. When compact mode is off, vfmacro emits none of these new preprocessor
directives. Classical non-UCD output was compared with the pre-change templates and
matched apart from comments and whitespace.

Production validation passed 33 focused native tests, 38 Python regressions, and 11
native transition-relation checks covering varied lane/jump and acceleration settings.
All 36 matched sequential solves agreed on bounded feasibility and SAT depths; six
bidirectional projected-witness replays passed. The sum of the six query medians was
25.849 s for legacy and 3.861 s for production, about 6.70x faster. The known blind
fixtures remain without a witness through bound 30.

Complete production-path trials used the real native input writer, no experimental
rewrite, all 16 priorities, shortcuts, and strict first-failure accounting:

| Seed | Cycles | MC Jobs | nuXmv Time | Cycle Time | Minimum Sampled Clearance | Completion |
|---|---:|---:|---:|---:|---:|---|
| 0 | 19 | 24 | 8.00 s | 12.55 s | 1.134 m | Model and continuous goal |
| 1 | 19 | 20 | 7.15 s | 11.41 s | 1.620 m | Model only |
| 2 | 20 | 21 | 5.47 s | 9.59 s | 1.353 m | Model and continuous goal |

All three had zero blind steps and crashes. Seed 1 stopped about 12.5 cm before the
continuous forward-position threshold under the existing model-completion rule.
Seed 0 used priority 4 once; no fallback configurations were removed. Median cycle
times were 0.377 / 0.497 / 0.363 s, so the 100 ms target remains unmet. These runs
follow different trajectories from the prototype and are not a rare-event safety
certification or evidence of improved clearance. Startup is excluded from the table;
cycle timing includes the evaluator instrumentation.

Production artifacts are in `tmp/task7_production_20261024` and the
`task7_production_final_priority_seed{0,1,2}` directories under
`tmp/ucd_live_task7_eval/selection_eval`. The remaining sections describe the original
prototype experiment.

## Scope and Method

Only the current five-car task-7 scenario was tested: two moving cars, three parked
cars, two physical lanes, 12 technical lane bits representing 23 normalized lateral
positions, one straight section of length 40000 in model units, and BMC bound 30.
No general road-geometry optimization was tested.

The evaluator captured the exact generated main SMV input after native observation
conversion, including the original goal and INIT constraints. Fixtures cover:

- Initial full goals for seeds 0 and 1, both satisfiable.
- A failed shortcut for seed 0 and a successful shortcut for seed 1.
- The initially blind seed 5 at both largest and smallest bounding-box priorities.

Every variant received identical observations, goals, and model parameters. Timings
are nuXmv process wall time, including preprocessing, model construction, solving,
and trace output; they exclude Python simulation and model generation. All solves
ran sequentially, with three repetitions and rotated variant order. The per-query
wall limit was 30 s; no measured run timed out. There were 144 initial-matrix runs
and 90 combination-control runs, including repeated baseline and combined controls.

All arithmetic introduced by these variants is linear or piecewise linear.
No variable-times-variable product was introduced. Lookup alternatives were measured
instead of assuming arithmetic or lookup tables would necessarily be faster.

## Standalone Changes

The total column sums the median time of each of the six fixtures; it is not a
population-average planning-cycle cost.

| Encoding | Total Median Time | Relative Speed |
|---|---:|---:|
| Original model | 25.798 s | 1.00x |
| Compact jumps, linear arithmetic | 17.288 s | 1.49x |
| Compact jumps, seven-entry velocity lookup | 16.361 s | 1.58x |
| Direct lane coordinate, integer plus bounds | 19.197 s | 1.34x |
| Direct lane coordinate, finite type `0..22` | 7.320 s | 3.52x |
| Project out acceleration | 22.243 s | 1.16x |
| Specialize parked state | 15.308 s | 1.69x |
| Combined: integer lane, linear jumps, projection, parked state | 3.825 s | 6.74x |

These are not uniformly beneficial changes. Compact linear jumps slowed full seed 0
from 5.38 s to 8.65 s and increased the blind-query times, while improving full seed 1
from 15.67 s to 2.79 s. Acceleration projection alone barely changed full seed 1.

### Combined Controls

The independent control matrix reproduced the combined result and exposed encoding
interactions: the best standalone lane type was not the best type in combination.

| Encoding | Total Median Time | Relative Speed |
|---|---:|---:|
| Fresh original control | 25.889 s | 1.00x |
| Combined | 3.841 s | 6.74x |
| Combined with finite lane type | 7.831 s | 3.31x |
| Combined with lookup jump limits | 4.286 s | 6.04x |
| Combined retaining moving-car acceleration | 7.800 s | 3.32x |

Matched original/combined query medians from the initial matrix:

| Query | Original | Combined | Result / Witness Depth |
|---|---:|---:|---|
| Full goal, seed 0 | 5.381 s | 1.305 s | SAT, 17 |
| Failed shortcut, seed 0 | 2.148 s | 0.863 s | No witness through 30 |
| Full goal, seed 1 | 15.665 s | 0.957 s | SAT, 16 |
| Successful shortcut, seed 1 | 0.316 s | 0.096 s | SAT, 1 |
| Blind initial state, priority 0 | 1.139 s | 0.304 s | No witness through 30 |
| Blind initial state, priority 15 | 1.149 s | 0.301 s | No witness through 30 |

The small shortcut measurement does not establish a 100 ms control-cycle capability.
Full goals and failed queries still exceed that budget.

## What Was Changed

1. **Lane jumps.** The original direction relations enumerate 23 current positions
   and seven destinations per direction. For the current parameters, a nonzero
   jump of absolute distance `distance` is permitted exactly when `distance <= 7`
   and `next(v) + 3 * distance <= 47`. The unchanged-lane alternative, endpoint
   clamping, current-velocity lane-hold guard, and next-velocity dependency remain.
   The lookup control selects the same permitted distance using seven speed thresholds.
2. **Lane representation.** Replace 12 state bits with a coordinate spanning 0..22,
   retaining all 23 legal positions. Original lane bits and predicates become DEFINEs,
   preserving their names and values for collision checks and trace consumers.
3. **Acceleration projection.** Replace saturated `next(v) = clamp(v + a)` with
   `-3 <= next(v) - v <= 3`, retaining velocity bounds and directional constraints.
   Remove the free acceleration state and mock ego acceleration alias. This preserves
   the possible velocity sequences, not the identity of every acceleration witness.
4. **Parked specialization.** Only cars constrained to zero velocity, with explicit
   initial positions inside the fixed section and a stationary lane-hold constraint,
   are specialized. Their position, velocity, acceleration, and lane state become
   definitions. Previous position remains 0 initially and equals the parked position
   afterward. Original assignment obligations and collision constraints are retained;
   invalid initial overlaps are not silently removed. Negative parked positions are
   left dynamic because the original boundary behavior matters there.

The transformer rejects unsupported parameter profiles and multiple sections. It is
a guarded rewrite of generated packages for experiments, not a general SMV optimizer.

## Validation

- Native nuXmv transition miters found no difference between the original lane
  relation and either compact version over the permitted lane and velocity domains.
- A native miter confirmed acceleration projection against all seven original
  acceleration choices, including saturated velocity endpoints.
- All 234 timed runs agreed with the original bounded feasibility result. Satisfiable
  fixtures also retained their original witness depths.
- There were 66 successful bidirectional witness checks. First-repetition optimized
  traces were pinned into the original model, and original traces into each variant,
  requiring the full witness depth to remain valid. The projection includes vehicle
  positions, velocities, lane bits, and section/traversal state.
- The 38 Python regression tests pass, including declaration matching, projected
  trace handling, parked-state guards, and transformation-cache invalidation.

Local miters and sampled witness checks are not a universal whole-model equivalence
proof. Equivalent state encodings can also change which witness the solver chooses,
which changes the continuous closed-loop trajectory.

## Closed-Loop Checks

Fresh baseline and combined runs used the actual native planner, priority selector,
simulator, unchanged pure-pursuit control, all 16 fallback configurations, and enabled
shortcuts. Each run stopped on the first blind/crashed step, with no exclusion of
short failures. Generation/startup is excluded below. Cycle times include geometric
clearance instrumentation and Python/native processing.

| Seed | Encoding | Cycles | MC Jobs | nuXmv Time | Cycle Time | Minimum Sampled Clearance |
|---|---|---:|---:|---:|---:|---:|
| 0 | Original | 19 | 20 | 23.95 s | 27.98 s | 1.411 m |
| 0 | Combined, cached rewrite | 22 | 23 | 14.08 s | 19.56 s | 1.469 m |
| 1 | Original | 18 | 24 | 31.86 s | 36.20 s | 0.889 m |
| 1 | Combined, cached rewrite | 20 | 25 | 8.25 s | 15.89 s | 1.510 m |

All four runs had zero blind steps and crashes. Both optimized runs reached both
model completion and the continuous Python goal. Original seed 1 reached only model
completion. These totals therefore are not identical-trajectory/termination comparisons.
The baseline used nine accepted shortcuts on seed 1; these optimized runs accepted
only full plans. The matched shortcut fixture still verifies shortcut feasibility.

The first working prototype rewrote every package before every query, adding enough
Python overhead to give 32.05 s / 27.82 s cycle totals. Caching by configuration and
parked INIT/INVAR inputs preserved those trajectories and reduced the totals above.
Measured rewrite overhead after caching was 0.64 s / 3.00 s across the respective
entire runs. The production generator described above avoids this runtime rewrite.

This does not fix the known blind initial states, certify rare-event safety, establish
a clearance improvement, or validate the denser ten-car task-7 setup. Held-out task-7
states, difficult low-priority recoveries, and more closed-loop seeds remain necessary.

## Reproduction

Use the existing native build and Python environment, including the evaluator's
Shapely dependency. Start with fresh output names. The collector's selected seeds and
query categories target the current five-car configuration.

```bash
.venv/bin/python -m morty.evaluate_smv_encodings --self-test
.venv/bin/python -m morty.evaluate_smv_encodings --collect tmp/my_task7_smv
.venv/bin/python -m morty.evaluate_smv_encodings --prepare tmp/my_task7_smv
.venv/bin/python -m morty.evaluate_smv_encodings --benchmark tmp/my_task7_smv
.venv/bin/python -m morty.evaluate_smv_encodings --verify tmp/my_task7_smv
.venv/bin/python -m morty.evaluate_smv_encodings \
  --live-check tmp/my_task7_smv_live \
  --config tmp/my_task7_smv/benchmarks/full_seed0/combined/envmodel_config.tpl.json
```

Preparation writes all eleven encodings into `#TEMPLATE` in the experiment template.
Change `UCD_SMV_EXPERIMENT_ENCODINGS` (semicolon-separated),
`UCD_SMV_EXPERIMENT_REPEATS`, or `UCD_SMV_EXPERIMENT_TIMEOUT_SECONDS` there before
benchmarking. Each candidate package receives its own template recording its encoding.
Live checks use `UCD_EVALUATION_SMV_ENCODING`, populated from the selected fixture's
`UCD_SMV_EXPERIMENT_ENCODING`. These settings require the evaluator; the production
driver does not apply the transformations. `UCD_EVALUATION_CAPTURE_SMV` enables exact
main-SMV capture during query collection. Individual trial seeds and stopping limits
are persisted by the existing evaluator. Prototype collection and live checks explicitly
disable `UCD_TASK7_COMPACT`; stacking an experimental rewrite on the production
encoding is rejected. To evaluate the production mode itself, use
`python -m morty.evaluate_mc_selection` with the active template and no experimental
rewrite setting.

The recorded study artifacts are under `tmp/task7_smv_encodings_20261024` and its
`_combinations`, `_live_v2`, `_live_cached`, and `_live_baseline` siblings. Initial
`_live` attempts failed in the instrumentation import path before launching a solver
and are preserved separately. The original eight-encoding matrix and five-encoding
control matrix are preserved; a fresh default run tests all eleven in one matrix.
Detailed query-capture/live packages are under `tmp/ucd_live_task7_eval/selection_eval`.
Artifacts include exact SMV fixtures and hashes, per-repetition logs, witness replays,
template configurations, and JSON summaries.