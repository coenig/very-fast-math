# MC Selection Safety Study

Date: 2026-10-24. This is a screening study, not a safety certification.

## Conclusion

Keep all 16 configurations available for now. There is evidence for both parts of the
original design: large boxes provide useful physical clearance, and smaller-box fallbacks
can prevent a blind iteration. The current evidence does not justify removing the priority
preference or selecting a minimal permanent configuration pool.

Zero blind iterations alone is not an adequate safety criterion: two reduced-pool trials
crashed while the planner continued returning plans. Conversely, three initial states were
blind even with all 16 configurations and the unchanged legacy all-config policy. Their
exact model-level cause has not been established; ordering alone cannot solve those cases.

Production selection code, the active five-car configuration, solver algorithms, bounds,
and the generated model definitions were not changed for this study.

## Method

The evaluator runs the actual live driver in isolated directories, sequentially, with
nuXmv 2.2.0. It changes only the allowed configuration indices and their query ordering.
All generated packages remain present, and indices always refer to the original priority
list. Both setups use task 7, two lanes, BMC bound 30, and enabled shortcuts:

- Five vehicles: the active configuration, seeds 0-9 for the main comparison.
- Ten vehicles: the original task-7 master, seed 0 as a denser stress case.
- Reduced-pool and shortcut-first controls: five-car seeds 0-3.

There are 40 study trials, plus three one-cycle legacy checks and separate instrumentation
smoke tests. Trials stop on the first blind or crashed step, or after 50 cycles. The wall
limit is 150 seconds per five-car trial and the largest-only ten-car trial, 300 seconds for
the four-config ten-car trial, and 330 seconds for the full-pool ten-car controls. These
evaluation limits include startup and do not change an individual solver's settings.

The evaluator sets `min_iterations=0`: early failing seeds are retained. The ordinary
driver's default exclusion of short unsuccessful runs must not be used to estimate the
blind-event rate. Timeouts and step limits are reported separately, never as successful
zero-blind completions. Counts of failed individual queries are distinct from blind cycles.

Completion follows the existing driver rule: a full-goal `FINISHED` result or the Python
success predicate. Both signals are also recorded separately. A model-complete run is not
necessarily exact completion of the continuous-position predicate.

Physical clearance is the minimum Euclidean distance between the simulator's rotated
vehicle footprints, measured initially and after every physics step with Shapely. Pairs of
two initially stationary vehicles are excluded; moving-to-moving and moving-to-parked pairs
are included. This is a sampled geometric distance, not TTC, road-edge clearance, or a
continuous-time collision proof. The simulator's own crash flag remains authoritative;
collision anticipation/response can flag a crash without sampled polygon overlap.

Instrumentation smoke tests reproduced the earlier actions and vehicle positions exactly.
Solver-time totals exclude geometry sampling and Python overhead. Totals for timed-out
trials cover recorded completed cycles, not the interrupted query.

### Policies

| Evaluator Policy | Behavior |
|---|---|
| `adaptive` | Current cost/success ranking and shortcut/full-goal fallback schedule. |
| `priority` | Same orchestration, but candidate lists are always in original priority order. |
| `cascade` | All eligible saved waypoints first, in waypoint/priority order, then full goals. |

All three stop at the first usable result. `cascade` is a sequential reference for the
original shortcut-first preference, not the legacy parallel batch implementation. Only
`adaptive` is an existing production selection option; `priority` and `cascade` are
evaluation controls.

The four-config pool uses priorities `0,5,10,15`: length/width parameter pairs `(9,9)`,
`(8,8)`, `(7,7)`, `(6,6)`. These are template parameters, not physical dimensions in meters.
Priority 0 is largest in both dimensions; priority 15 is smallest. The full hierarchy
prioritizes width before length, so intermediate priorities are not a scalar size ranking.

## Results

### Five Cars

| Pool and Policy | Seeds | Complete | Blind | Crash | Timeout |
|---|---|---:|---:|---:|---:|
| Full, adaptive | 0-9 | 6 | 3 | 0 | 1 |
| Full, priority | 0-9 | 6 | 3 | 0 | 1 |
| Full, cascade | 0-3 | 1 | 0 | 0 | 3 |
| Largest only, adaptive | 0-3 | 3 | 1 | 0 | 0 |
| Smallest only, adaptive | 0-3 | 3 | 0 | 1 | 0 |
| Four configurations, adaptive | 0-3 | 4 | 0 | 0 | 0 |

For both full-pool policies, seeds 5, 8 and 9 were blind at the first cycle after all
16 queries failed; all solver exit codes were zero. One-cycle runs with production
`--mc_selection all` reproduced each failure. These are pre-existing no-plan cases under
the current model/query setup, not evidence of physical impossibility or a proven input
domain violation. There is no prior saved trajectory at the first cycle.

Seed 4 timed out under both full-pool policies. Adaptive had selected priorities 0, 12,
and 2 during its 20 recorded cycles; priority ordering selected 0, 12, and 8 during 23
cycles. That already shows why usage statistics from easy seeds are insufficient to prune
configurations. Neither partial run certifies safe completion.

The six mutually completed seeds were 0, 1, 2, 3, 6 and 7. Their summed solver times were
141.85 seconds for adaptive and 141.20 seconds for priority ordering. This is not evidence
that relaxing priority improves cost in this cohort. Trajectories and termination can
differ: priority seed 1 stopped on model completion with the Python predicate still false.

Selected matched clearance measurements:

| Seed | Full Adaptive | Full Priority | Smallest Only |
|---|---:|---:|---:|
| 0 | 1.411 m | 1.411 m | 0.272 m |
| 1 | 0.889 m | 0.889 m | 0.100 m |
| 2 | 1.531 m | 1.531 m | 0.458 m |
| 3 | 0.759 m | 0.759 m | 0 m, crash |

The only completed cascade trial, seed 0, had 1.500 m minimum clearance and used
126.91 seconds of solver time versus adaptive's 24.04 seconds. Clearance minima from
the three timed-out cascade runs are not comparable to completed traversals.

### Ten Cars

| Pool and Policy | Result | Recorded Cycles | Solver Jobs |
|---|---|---:|---:|
| Full, adaptive | Model complete, no blind/crash | 32 | 51 |
| Full, priority | Timeout | 11 | 44 |
| Largest only, adaptive | Blind immediately | 1 | 1 |
| Four configurations, adaptive | Crash, zero blind cycles | 19 | 27 |

The full adaptive run had 0.253 m minimum clearance and used 218.53 seconds of solver
time. As in the earlier evaluation, its Python success predicate remained false at model
completion: the forward car was about 9.6 cm short of the continuous threshold. The
priority run's partial results do not establish a safety advantage or disadvantage.

## Counterexamples

### Removing Necessary Fallbacks

Five-car seed 1, largest-only, matched the full adaptive run's actions and positions
exactly for the first 12 cycles. At cycle 13, the largest-box nearest-waypoint and full-goal
queries both failed. The full pool recovered with priority 1, `(8,9)`, and completed.
Largest-only tried the remaining saved waypoints and went blind. This is a matched-state
example of a configuration preventing blindness, not just two different rollouts.

### Small Boxes Do Not Imply Safe Driving

Smallest-only five-car seed 3 crashed at cycle 5 between moving vehicle 0 and parked
vehicle 2. Every cycle had a successful planning query. This rejects the hypothesis that
maintaining low blindness alone is enough to permit aggressive envelope reduction.

### Unselected Configurations Can Affect Scheduling

In the ten-car full-pool and four-config runs, every accepted plan used priority 5.
Nevertheless, their trajectories diverged at cycle 2:

- Full pool: speculative shortcuts at priorities 0 and 4 failed; a full goal at 5 succeeded.
- Four-config pool: shortcut 0 failed and shortcut 5 succeeded immediately.

The reduced pool subsequently crashed between the two moving vehicles at cycle 19.
Its minimum sampled clearance was about 1.1 cm despite the simulator's crash flag.
This does not prove that an omitted configuration needed to win. Removing it changed the
cost-ranked query schedule and shortcut choice. Winner frequency alone misses this effect.

## Reproduce and Extend

Use the existing Python environment and native build; the evaluator additionally needs
Shapely, Linux, GNU `time`, and the existing `timeout` utility:

```bash
python -m pip install "shapely>=2,<3"
python -m morty.evaluate_mc_selection \
  --config morty/envmodel_config.tpl.json --shortcuts on \
  --policies adaptive priority --seeds 0 1 2 3 4 5 6 7 8 9 \
  --steps 50 --timeout 150 --tag my_five_car_comparison
```

Example reduced-pool stress case:

```bash
python -m morty.evaluate_mc_selection \
  --config morty/master_templates/task_7_tpl.json --shortcuts on \
  --policies adaptive --configs 0 5 10 15 --seeds 0 \
  --steps 50 --timeout 300 --tag my_four_config_trial
```

Use a new tag each time: existing trial directories are never overwritten. Output is under
`tmp/ucd_live_task7_eval/selection_eval/<tag>_<policy>_seed<seed>/`, with isolated models,
configuration, cycle metrics, per-query records, solver timing, logs, and an outcome summary.
New runs additionally record the raw model input and target for each query. Time-based
ranking can depend on machine load, so retain actual query sequences, not just seed values.
Run timing comparisons sequentially.

Study tags start with `safety_full_5cars`, `safety_largest_5cars`, `safety_smallest_5cars`,
`safety_four_5cars`, and the corresponding ten-car names. Legacy initial-state checks use
`safety_legacy_seed5`, `safety_legacy_seed8`, and `safety_legacy_seed9`.

## Next Acceptance Gates

1. Diagnose the all-config initial no-plan states separately from blindness after successful
   planning. Do not silently remove either category from safety accounting.
2. Resolve or explicitly censor unfinished trials. A timeout before a difficult interaction
   is not evidence that the interaction is safe.
3. Test proposed removals against recorded failing/recovery states and full closed-loop
   rollouts, including dense scenes and previously unseen seeds. Preserve a held-out set.
4. Assess collisions and low-clearance events alongside blindness, selected envelope sizes,
   and runtime. Neither box size nor a returned plan alone proves physical safety.
5. Specify a numerical acceptance rate and its denominator. For example, zero failures in
   299 independent representative runs gives a one-sided 95% binomial upper bound below
   1% per run; below 0.1% requires 2,995 zero-failure runs. Consecutive control cycles are
   correlated and cannot simply be counted as independent trials.

This study has already found failures and is far too small for an "almost never" claim.
It also contains no sensor-noise or real-time hardware validation. Keep the complete
fallback set and avoid promoting the relaxed ordering or a reduced pool to a safety claim.