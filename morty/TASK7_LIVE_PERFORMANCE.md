# Task 7 Live-Loop Performance Notes

Date: 2026-11-05. Windows 11, 28 cores, MSVC Release build, bundled nuXmv **2.1.0** (`external/win32`).
Earlier measurements in [SMV_ENCODING_EVALUATION.md](SMV_ENCODING_EVALUATION.md) were made on Linux with
nuXmv 2.2.0 on a slower machine; absolute timings are not comparable across the two.

Scenario: active five-car task-7 template (two moving cars, three parked cars, 16 bounding-box
configurations, `priority` selection, BMC bound 30, `UCD_TASK7_COMPACT`).

## Summary

On this machine, a query that follows the existing plan now takes about 30 ms of nuXmv time, so the
<= 100 ms target for pure model-checking time is met in the common case. Full replans still take
0.1-1 s, and the complete cycle is still around 0.6-1 s because of non-solver overhead (mainly
Windows Defender, see below).

| Run | Before | After |
|---|---:|---:|
| Seed 0, 19 cycles, summed cycle time | 177 s | 18 s |
| Seed 1, 19-20 cycles, summed cycle time | 43 s | 18-36 s |
| Depth-1 shortcut query inside the loop | 4.3 s | ~100-130 ms (incl. output processing) |
| Depth-1 shortcut query, nuXmv only | ~80 ms (fresh process) | ~30 ms (persistent session) |

These are not identical-trajectory comparisons: the selection order and therefore the trajectories
differ between runs, and machine load varied noticeably (the second seed-1 run was ~2x slower in
its first half for no code-related reason).

## Where the Time Went

A depth-1 query took ~80 ms as a standalone nuXmv run, but 4.3 s inside the live loop.

| Cause | Cost | Status |
|---|---|---|
| Windows process watchdog (`killAfter`: WMI process scan plus a fixed 3 s sleep) | ~4 s per query | Skipped in `--live`; Windows live nuXmv calls get a native 300 s timeout (`StaticHelper::execWithoutShell`) |
| Planner translation (`kratos`) on first use of each config | 6-12 s, also mid-run when a fallback config is first used | All 16 packages share an identical planner source; the translation is copied from an up-to-date sibling package |
| `-pre cpp` on every nuXmv call | ~55 ms | Live mode preprocesses only when the directive header or an included file changes and passes `main_live.smv` without `-pre cpp` |
| Launching nuXmv through `cmd.exe` (`_popen`) | ~20-30 ms | Started directly with `CreateProcess` in live mode |
| `CREATE_NO_WINDOW` for the direct process start | ~20 ms | Not used |
| Re-parsing `envmodel_config.json` on every value lookup | ~10-15 ms per query | Cached by path, write time and size in live mode |
| Constructing `McWorkflow` (including its formula parser) | ~35 ms per call | One shared instance in live mode for input and output preparation |

### nuXmv fixed cost (fresh process, depth-1 shortcut)

| Step | Cumulative time |
|---|---:|
| Process start and `quit` | 21 ms |
| `read_model` with `-pre cpp` | 93 ms |
| `read_model` without `-pre cpp` | 27 ms |
| `flatten_hierarchy` | 40 ms |
| `go_msat` | 47 ms |
| BMC check, depth 0 only | 60 ms |
| Full depth-1 query incl. trace printing | 82 ms |

Trace printing is negligible (~3 ms). nuXmv's cone-of-influence option gave mixed results
(676 -> 1583 ms, 265 -> 195 ms on different fixtures) and was not adopted.

### Windows Defender

The first read of a file whose content changed costs ~25-30 ms; re-reading identical content is
fast (~1 ms). This appears to be content-based on-access scanning (real-time protection is enabled;
exclusions need administrator rights to inspect). Consequences per cycle:

- Input preparation rewrites all 16 `main.smv` files: 250-500 ms.
- Reading back all 16 `debug_trace_array.txt` files after each query: ~50 ms.
- A freshly written 165 KB model costs nuXmv ~30 ms extra versus an unchanged one.

Possible fixes: exclude the work folder from scanning, or in selective modes only write and read the
packages actually queried. Not implemented.

## Changes to the Search

### Bounded shortcut depth (`UCD_SHORTCUT_BMC_SLACK`)

Waypoint `p` of the previous plan lies `p + 1` steps ahead, so a shortcut is searched only to depth
`min(BMC_CNT, p + 1 + slack)` instead of `BMC_CNT`. Missed shortcuts went from 0.7-1.0 s to ~80 ms
(fresh process); all successful shortcuts on the captured fixtures were within the bound.

### Probing several waypoints (`UCD_SHORTCUT_PROBE_POINTS`)

Before any full replan, probe the next waypoints of the current plan with the best eligible
configuration and the current plan's configuration. This replaces the legacy cost-gated probe of
waypoint 0, which stopped probing after a few expensive failures.

Offline check on captured seed-1 states (bound `p + 3`):

| Waypoints tried | Cycles with a hit (of 18) |
|---|---:|
| 0 only | 8 |
| 0-4 | 13 |

Each probe cost ~110-200 ms in that offline check (fresh process).

### Persistent nuXmv sessions (`UCD_PERSISTENT_SOLVER`)

Implemented in [persistent_solver.py](persistent_solver.py). One interactive nuXmv process per
configuration keeps the model parsed, flattened and encoded (`go_msat` once). For each query:

1. The `-- INIT CONSTRAINTS` block of `main.smv` is turned into frozen copies `__obs_i` with
   `INIT __obs_i = <observed variable>` in the static model.
2. The query checks `!(__obs_0 = v0 & ... & (goal))` via `msat_check_invar_bmc ... -p "..."`.

Because a frozen variable keeps its initial value, a depth-k violation of this property corresponds
exactly to a depth-k path of the original model with `INIT var = value` and `INVARSPEC !(goal)`.
The `__` prefix hides the copies from printed traces. The session restarts when the static part
(parked constants, configuration, planner) changes; unsupported layouts and missing or stale
planner translations fall back to the per-query process.

Prototype on 19 captured seed-0 queries (same model, one session):

| Query | Fresh process | Persistent session |
|---|---:|---:|
| Session startup / `go_msat` | - | 19 ms / 35 ms (once) |
| Shortcut hit, depth 1 | ~82 ms | 30-46 ms |
| Shortcut miss, bound 3 | ~80 ms | 32-37 ms |
| Full goal, depth 12-13 | 409-654 ms | 403-409 ms |
| Full goal, depth 10 | 252 ms | 204 ms |
| Full goal, depth 9 | 216 ms | 136 ms |
| Full goal, depth 5 | 122 ms | 77 ms |
| Full goal, depth 3 | 97 ms | 47 ms |

All 19 results and witness depths matched the original encoding. A unit test compares SAT/UNSAT and
trace length against a fresh `-pre cpp` run on a small model.

## Profiled Runs

Per-cycle profiles (S = shortcut, F = full goal, priority index, +/- = success, query ms, `d` = trace
states). Selected rows only.

### Baseline (seed 0, watchdog active)

```
c00  12827ms  F0+12398/d18
c01   4787ms  S0+4378/d2
c05  10294ms  S0-5168/d0 F0+4708/d13
c11  63733ms  F0-4499/d0 F1-12021/d0 F2-12063/d0 F3-12187/d0 F4+21825/d7
```

The same F1-F3 queries took 320-345 ms standalone; the rest was the watchdog plus kratos on first
use of each configuration.

### Baseline without watchdog (seed 0)

```
c00  20206ms  F0+17882/d18
c05   4177ms  S0-1183/d0 F0+908/d13
c21 165969ms  F0-744/d0 F1-8199/d0 ... F15-17331/d0 S0-535/d0   (blind)
```

Standalone, each of those 17 failing queries took ~300-390 ms.

### All changes enabled (seed 0)

```
c00  1841ms  F0+993/d18
c01  1132ms  S0+237/d2
c04   697ms  S0+96/d2
c07  1752ms  S0-89/d0 S0-113/d0 S0-132/d0 F0+512/d11
c10   890ms  S0-127/d0 S0+149/d3
c18   671ms  F0+89/d1
```

Query times here include clearing traces, the persistent check, and native output processing (~50 ms).

## Rounding Observed Positions (`UCD_ROUND_INPUTS`)

All inspected shortcut misses failed by exactly one position unit (0.25 m), e.g. a target
`abs_pos = 218` from `174` with a maximum next velocity of `43`. `prepareInputForMortyUCD` truncated
position and velocity with `(int)`, biasing a forward car backwards.

Rounding velocities as well made seeds 0 and 1 blind in their second cycle: a parked car with a
residual speed of ~0.6 model units was observed as `v = 1`, contradicting its `INVAR v = 0`, so every
query failed at depth 0. Truncation silently maps such speeds to 0. Only positions are rounded now.

A/B on seeds 0-3 (all other settings as above; trajectories differ between runs):

| Seed | Truncate: shortcut hits / tries, cycles | Round positions: shortcut hits / tries, cycles |
|---|---|---|
| 0 | 13 / 28, 19 cycles, goal reached | 14 / 27, 21 cycles, goal reached |
| 1 | 13 / 31, 19 cycles, goal reached | 13 / 30, 22 cycles, goal reached |
| 2 | 13 / 29, 18 cycles, goal reached | 15 / 22, 19 cycles, goal reached |
| 3 | 3 / 37, 14 cycles, stopped without reaching the goal | 15 / 31, 22 cycles, goal reached |
| Sum of query time | 29.7 s | 24.7 s |

The gain is modest on seeds 0-2 and large on seed 3. Four seeds are too few for a firm conclusion,
and the truncated seed-3 stop was not investigated. The setting applies to all UCD tasks that
enable it; it defaults to off.

## Five vs. Ten Cars (Seeds 0-9)

Setup: all settings above enabled (including `UCD_ROUND_INPUTS`), `NONEGOS` 5 (2 moving, 3 parked)
vs. 10 (2 moving, 8 parked), seeds 0-9 each, runs executed one at a time. Query times are measured
in the loop and include ~50-100 ms of non-solver work per query (trace clearing, output processing).

| | 5 cars | 10 cars |
|---|---:|---:|
| Runs reaching the goal | 7 / 10 | 7 / 10 |
| Blind at the first cycle (seeds 5, 8, 9, all 16 configs fail) | 3 | 3 |
| Planning cycles (all runs) | 150 | 205 |
| Cycles served by a shortcut | 105 (70%) | 142 (69%) |
| Plan-config probe hits at waypoint 0 / 1 / 2 | 100/133, 3/30, 1/25 | 117/183, 12/61, 7/46 |
| Probe hits with another (larger-box) config | 1/4 | 6/250 |
| Median shortcut query | 106 ms | 142 ms |
| Median successful / failed full query | 276 / 452 ms | 855 / 694 ms |
| MC time per cycle: median / p75 / p90 | 113 / 285 / 832 ms | 284 / 1462 / 5190 ms |
| Cycles with MC time <= 200 ms | 71% | 10% |

Query time by kind, summed over all runs:

| Kind | 5 cars | 10 cars |
|---|---:|---:|
| Successful full queries | 24.1 s | 104.8 s |
| Failed full queries (larger configs before the winner) | 0.5 s | 118.1 s |
| Failed full queries in blind first cycles | 21.9 s | 37.9 s |
| Shortcut probes, plan config (hit + miss) | 20.1 s | 55.9 s |
| Shortcut probes, other config (hit + miss) | 0.6 s | 34.4 s |

Observations:

- **The shortcut hit rate does not collapse with more cars** (70% vs. 69% of cycles). Parked cars
  are model constants and never block a shortcut; only the two moving cars' terms can fail.
- **Why probes miss.** Each plan-config miss at waypoint 0 was re-checked with one car's target
  terms removed at a time. The blocking car was always a moving car, mostly its position. Allowing
  a position tolerance for the moving cars would have turned these misses into hits:

  | Tolerance (model units, 1 = 0.25 m) | 5 cars (33 misses) | 10 cars (66 misses) |
  |---|---:|---:|
  | +-1 | 14 | 29 |
  | +-2 | 25 (76%) | 46 (70%) |
  | +-4 | 30 (91%) | 50 (76%) |
  | > 16 or not position-only | 3 | 16 |

- **With 10 cars, the plan usually comes from a smaller box** (priority 1-3), because the largest
  boxes do not fit between the parked cars. Then every cycle first probes the largest eligible box,
  which almost never hits (6 of 250), and every replan first fails the larger boxes. Those failures
  cost more than the successful plans (118 s vs. 105 s).
- **10-car replans near the bound are expensive.** Seed 7 needs depth 24-27 plans; failing larger
  boxes take 8-10 s each and the winning plan 5-7 s, giving 16-20 s replan cycles.
- A first run of seed 7 took 497 s with 60-120 s queries while an offline analysis ran in parallel;
  an isolated rerun took 119 s and replaces it in the tables. Timings are load-sensitive.

## Not Implemented (Ranked)

1. **Shortcut position tolerance for moving cars.** +-2 units (0.5 m) would have recovered 70-76% of
   the remaining misses. It also needs a tolerant connector match in `prepareOutputForMortyUCD`,
   otherwise the stored plan is reset after a shortcut; the old plan's tail then starts up to 0.5 m
   away from the actual state (it is re-verified in the next cycle).
2. **Stop re-probing and re-trying boxes that just failed.** For 10 cars, shortcut probes with a
   larger box than the plan's (34 s, 2.4% hits) and repeated failing full queries for larger boxes
   (118 s) dominate. Probing the plan config first, and retrying larger boxes only occasionally or
   after the scene changes, keeps the priority preference without paying for it every cycle.
3. **Monotone configuration search.** If a box size has a plan, every smaller size has one too. Trying
   the largest configuration, then the smallest, detects a blind cycle in 2 queries instead of 16;
   skipping configurations that dominate a failed one keeps the final choice equal to the priority
   order. If monotonicity were ever violated, only optimality (not safety) would be affected, since
   every accepted plan is a verified witness for its own configuration. All observed cascades were
   consistent with monotonicity (e.g. widths 9 failing for all lengths, then `(9,8)` succeeding).
4. **Reduce per-cycle file traffic** (see Windows Defender above).

## SMV-Level Experiments

- Removing the section-crossing logic for the single straight section (`will_arrive_*` := FALSE plus
  `abs_pos >= 0`): mixed, e.g. 676 -> 1308 ms and 419 -> 673 ms on two full goals, 677 -> 533 ms on
  another. Not adopted. Note that cars are already pinned to section 0 by `INVAR is_on_sec_0 = 1`; the
  only remaining crossing is a wrap-around at the rear (`abs_pos < 0`).
- Cone of influence: mixed (see above).

Process overhead and query scheduling were the effective levers; further encoding changes look
unlikely to pay off on the hit path.

## Caveats

- Untested on Linux: the persistent solver, one-time preprocessing (GNU `cpp` line markers in the
  preprocessed model need checking), and the planner-translation copy.
- Without the watchdog, Windows live mode relies on the 300 s timeout in `execWithoutShell` and in
  the persistent session.
- The two errors in `test_evaluation_cleans_up_interrupted_and_timed_out_trials` are pre-existing on
  Windows (`os.killpg` does not exist there).

## Reproduction

The helper scripts are in the gitignored `tmp/t7bench/` folder; run them from the repository root.

```bash
python tmp/t7bench/profile_run.py tmp/t7bench/<out> <seed>          # per-query profile, captures fixtures
python tmp/t7bench/bench_fixtures.py tmp/t7bench/<out> nocpp 3        # standalone timing of captured queries
python tmp/t7bench/bench_fixtures.py tmp/t7bench/<out> k3 3           # same, shortcuts bounded to depth 3
python tmp/t7bench/shortcut_points.py tmp/t7bench/<out> 2 5           # hit rate of waypoints 0-4
cd tmp/t7bench && python persistent_proto.py <fixture dirs ...>       # persistent-session prototype
bash tmp/t7bench/batch.sh 10 cmp 0 1 2 3 4 5 6 7 8 9                 # NONEGOS=10 runs (template restored after)
python tmp/t7bench/analyze.py tmp/t7bench/cmp_n5_seed tmp/t7bench/cmp_n10_seed
python tmp/t7bench/miss_reasons.py tmp/t7bench/cmp_n10_seed0 ...      # blocking car and tolerance per miss
python -m unittest morty.test_live_mode
```
