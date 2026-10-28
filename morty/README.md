### Driving-only mode
Rebuild the native library after updating, then run:

```bash
cmake --build build --target runTests --parallel
python3 -m morty.morty --live
```

With `UCD_MC_SELECTION: "all"` in the template, `--live` keeps the configured model-checking algorithms, bounds, priorities, driving dynamics,
pure-pursuit formulas, and optional shortcut checks. It disables visualization, video, plots,
archives, experiment statistics, diagnostic logging, and the extra `FALSE`-specification debug
solve. Warnings and errors remain enabled. The active template instead uses the recommended
`priority` selection described below, which requires `--live`. Set `UCD_MC_SELECTION` to
`all` to use ordinary experiment or replay mode.

To watch the simulation while retaining the live planning path, explicitly opt into the viewer:

```bash
python3 -m morty.morty --live --force_head
```

`--force_head` requires `--live` and cannot be combined with `--headless`. Plain `--live`
remains fully headless. The viewer shows the road, vehicles, and vehicle IDs, updating once per
control cycle without a frame-rate delay. Physics stepping is unchanged. Trajectory overlays,
debug plots, statistics, video, archives, and the extra debug solve remain disabled. The window
has a status strip: yellow means `MC pipeline running` (full plan or numbered shortcut attempt),
while gray identifies input preparation, road rendering, result processing, and simulation steps.
The MC label includes the native input/solver/output pipeline, not just nuXmv CPU time. Status
changes are painted before blocking calls; the scene itself pauses while planning. Plain headless
`--live` does not paint or maintain these status updates. Rendering adds overhead, and generating the road
background once on the first cycle can take tens of seconds; use plain `--live` for timing
without visualization costs. A working graphical display is required (for example, X11
forwarding when running remotely).

Live runs retain the native library, launch configured models without artificial delays, reuse
prepared solver scripts and unchanged planner translations, and extract only the trajectory
prefix needed for control unless shortcut reuse requires the complete plan. The working
`debug_trace_array.txt` files are still needed to pass solver results to the controller; they
are overwritten, not archived. Future-point files are only produced when shortcuts are enabled.
Trace-variable lookups also avoid copying entire state dictionaries in both execution modes.
Parser dumps and GUI progress output are disabled, and cached debug files are not copied into
live working packages. Template metadata needed to validate the model cache is retained there.

Model generation and the initial planner translation are startup costs, not steady-state cycle
costs. Restart after changing the configuration or solver templates. Existing output-directory
checks still apply; choose `--press` or `--force` explicitly when reusing experiment paths.
`--live` rejects `--dryrun`, `--record_video`, and `--detailed_archive`.

On Linux, GNU `timeout` limits each solver process to the existing 300-second budget without a
global process-polling watchdog. This is an overhead-reduction mode, not a hard real-time
guarantee: solving the configured models can still exceed a few hundred milliseconds.

#### Run Configuration

Behavior settings belong in `#TEMPLATE` in [morty/envmodel_config.tpl.json](morty/envmodel_config.tpl.json),
not in driver command-line arguments. The active five-car task-7 template uses:

| Template Key | Recommended Value | Default When Absent |
|---|---|---|
| `UCD_MC_SELECTION` | `"priority"` | `"all"` |
| `UCD_NUM_RUNS` | `1` | `100` |
| `UCD_STEPS_PER_RUN` | `300` | `300` |
| `UCD_HEADING_ADAPTATION` | `-0.5` | `-0.5` |
| `UCD_ALLOW_BLIND_STEPS` | `0` | `1` |
| `UCD_ALLOW_CRASHED_STEPS` | `0` | `1` |
| `UCD_MIN_ITERATIONS` | `0` | `2` |

Keep `UCD_USE_SHORTCUT: true` and the full 16-entry `UCD_CONFIG_PRIOS` list. The failure
limits stop at the first blind or crashed step; `UCD_MIN_ITERATIONS: 0` retains early
failures instead of silently trying another seed to replace them. Change `UCD_NUM_RUNS`
in the template for a longer campaign. Missing new keys preserve historical defaults.
The former CLI options for these settings, including their short aliases, are rejected.
Display, live/experiment mode, replay, and output-directory management remain CLI controls.

This recommendation preserves bounding-box preference within each candidate list and all
fallbacks. It does not eliminate the known initially blind seeds or establish rare-event
safety. The denser ten-car priority trial also timed out; this recommendation is for the
current five-car setup, not a general guarantee. See the [safety study](morty/MC_SELECTION_EVALUATION.md).

#### Task 7 Measurements (All Configurations)
Measured on Linux with nuXmv 2.2.0, using `morty/master_templates/task_7_tpl.json` unchanged
apart from isolated output/cache paths: ten vehicles, two lanes, sixteen prioritized models,
BMC bound 30, shortcuts enabled, seed 0. Model generation is excluded from cycle times.
The experiment baseline was headless, without video or detailed archives; both modes included
the shared trace-lookup optimization.

| Planning cycle | Experiment mode | Live mode |
|---|---:|---:|
| First full-plan cycle | 99.13 s | 51.32 s |
| Next shortcut cycle | 27.57 s | 4.73 s |
| Following shortcut cycle | 27.07 s | 4.81 s |

All three matched cycles produced identical selected priorities, shortcut decisions, actions,
and vehicle positions, with no crashes or blind steps. Shortcut cycles were about 5.7x faster.
Removing state-dictionary copies reduced coordinate extraction from 4.49 s to about 21 ms on
the first cycle, and from about 250 ms to 7-12 ms on shortcut cycles.

A separate three-repetition measurement of the same task's prepared shortcut query took
4.48-4.55 s for the sixteen-model solver batch, 96-105 ms for result processing, and about
6 ms for coordinate extraction. All prepared models remained byte-identical during that check.

An extended live run completed 22 cycles with no crashes or blind steps before the evaluation's
360-second wall-clock limit. Median completed-cycle time was 4.81 s. Cycles 17 and 18 took
57.59 s and 113.50 s because the existing shortcut policy issued seven and ten planning
attempts. The two moving vehicles passed each other, but had not cleared the full obstacle
corridor. This is not a completed-task or multi-seed safety validation, and **the 100 ms target
is not met**. Solver latency and repeated shortcut attempts remain the dominant limitations;
no model-checking heuristics were changed.

Focused regression checks:

```bash
python3 -m unittest morty.test_live_mode
./bin/runTests --gtest_filter='MortyLiveTests.*:MCTraceTests.*:GeometryScalingTests.*'
```

#### MC Selection Policies
Set `UCD_MC_SELECTION` in the template, then run:

```bash
python3 -m morty.morty --live
```

Add `--force_head` for the optional viewer. Selection options are:

- `all`: the original all-configuration batches and shortcut-first cascade, selecting the
  highest-priority usable result in each batch. This is the default when the key is absent.
- `priority`: one query at a time, with candidate configurations in original priority order.
  It stops at the first usable result and keeps all fallbacks. This is recommended for the
  current five-car scenario.
- `adaptive`: the same one-query orchestration with cost/success-ranked configuration lists.
  It can accept a lower-priority configuration without first trying every stricter one.

Both selective policies require `--live`. They change which queries run and their order,
not individual solver algorithms, bounds, model parameters, or vehicle dynamics. `priority`
preserves configuration order within each list, not the legacy shortcut-first cascade across
all saved waypoints. Neither policy guarantees the largest feasible box across all targets.

The adaptive selector ranks configurations by observed query cost divided by estimated success
probability, separately for full goals and shortcuts. Estimates reset each run, with original
priority breaking ties. Both selective policies use the following schedule: when shortcuts
look cheaper, first probe at most two configurations
against the nearest saved waypoint. It then tries full-goal configurations, returning on the
first usable result. If those all fail, it tries every remaining eligible configuration/waypoint
pair. Shortcut eligibility retains the previous full-plan winner's priority limit.

No generated package is removed or permanently blacklisted. Every configured full goal and
every eligible saved waypoint remains available before declaring an iteration blind; there is
no total query-count cutoff. Input geometry is prepared from the previous selected trace before
clearing working counterexamples. Skipped configurations cannot supply stale winners, and saved
future plans are retained for shortcut stitching. Exhaustive fallback preserves the candidate
set, but cannot guarantee a solution to an unsatisfiable or timed-out query or future states
reached along a different trajectory.

Separate Linux/nuXmv 2.2.0 measurements used a five-vehicle task-7 setup with two lanes, sixteen
size configurations and BMC bound 30. Both policies were run serially. With shortcuts disabled,
seed 0 completed in 19 planning cycles under both policies, with exactly identical actions and
vehicle positions and zero blind/crashed steps:

| Policy | Solver jobs | Summed solver time | Summed cycle time |
|---|---:|---:|---:|
| All configurations | 304 | 286.24 s | 293.40 s |
| Adaptive | 19 | 21.72 s | 24.87 s |

That is 92.4% less sequential solver time. Model generation and cached startup are excluded.
All sixteen EnvModels, solver commands, scaling files and planner sources were byte-identical
between these runs. The selected translated planner was also identical.

Adaptive completion checks across three seeds, each with shortcuts both enabled and disabled:

| Seed | Shortcuts | Planning cycles | Solver jobs | Summed solver time |
|---|---|---:|---:|---:|
| 0 | Off | 19 | 19 | 21.72 s |
| 0 | On | 19 | 20 | 24.11 s |
| 1 | Off | 18 | 18 | 28.18 s |
| 1 | On | 19 | 25 | 32.21 s |
| 2 | Off | 20 | 20 | 20.61 s |
| 2 | On | 20 | 21 | 22.49 s |

All six runs reached both model completion and the Python success condition with zero blind
steps and crashes. Seed 1 with shortcuts exercised a change from priority 0 to priority 1.
The shortcut-enabled all-config seed-0 baseline completed only two cycles within a 300-second
evaluation limit, using 128 jobs and 292.66 s of solver time. That is a partial baseline, not
an identical-trajectory or completed-run comparison.

The original ten-vehicle master, seed 0, reached the model's `FINISHED` state in 32 cycles,
using 51 jobs and 219.02 s of solver time, with zero blind steps and crashes. The driver ended
normally under its existing completion rule, but the Python success condition remained false:
the forward vehicle was about 9.6 cm short of its continuous-position threshold. This is not
an exact continuous-goal completion claim. Its first planning cycle still took 99.60 s, and
**the 100 ms target remains unmet**. These finite checks are not a universal safety guarantee.

#### Task 7 Compact Encoding
The active five-car configuration enables `#TEMPLATE.UCD_TASK7_COMPACT`: integer lane
coordinates, linear lane-jump constraints, projected acceleration, and conservative
parked-state specialization, generated directly by the native pipeline. All 16
bounding-box priorities, shortcuts, and solver bounds are retained. Set the option
to `false` and restart for the original encoding; it defaults to off in other
templates. Classical non-UCD generation remains unchanged.

After the original 234-query encoding study, 36 production/legacy solves agreed on
bounded feasibility and six bidirectional witness replays passed. The sum of six
matched-query median nuXmv times fell from 25.85 s to 3.86 s, about 6.7x. Production
seeds 0, 1, and 2 finished without blind steps or crashes; seed 1 reached only the
model completion condition, not the continuous Python goal. Median cycles were
0.36-0.50 s: this does not meet the 100 ms target or certify rare-event safety.
Build/run instructions, supported settings, measurements, and reproduction commands
are in [morty/SMV_ENCODING_EVALUATION.md](morty/SMV_ENCODING_EVALUATION.md).

#### Safety and Configuration-Pool Evaluation
The subsequent 40-trial study found blind initial states even with all 16 configurations,
blindness after removing a needed fallback, and crashes in reduced pools despite zero
blind iterations. These results do not justify removing the fallback range or treating
priority relaxation as safety-neutral. See
[morty/MC_SELECTION_EVALUATION.md](morty/MC_SELECTION_EVALUATION.md) for paired-policy results,
physical-clearance measurements, limitations, and the reproducible evaluator commands.
