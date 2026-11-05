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

## Not Implemented (Ranked)

1. **Round instead of truncate the MC inputs.** All inspected shortcut misses failed by exactly one
   position unit (0.25 m), e.g. a target `abs_pos = 218` from `174` with a maximum next velocity of
   `43`. `prepareInputForMortyUCD` uses `(int)` for position and velocity, biasing a forward car
   backwards. Rounding should raise the shortcut hit rate, but changes trajectories for all UCD tasks.
2. **Reduce per-cycle file traffic** (see Windows Defender above).
3. **Monotone configuration search.** If a box size has a plan, every smaller size has one too. Trying
   the largest configuration, then the smallest, detects a blind cycle in 2 queries instead of 16;
   skipping configurations that dominate a failed one keeps the final choice equal to the priority
   order. If monotonicity were ever violated, only optimality (not safety) would be affected, since
   every accepted plan is a verified witness for its own configuration. All observed cascades were
   consistent with monotonicity (e.g. widths 9 failing for all lengths, then `(9,8)` succeeding).
4. **Shortcut tolerance.** Allowing a small position tolerance in shortcut targets would also need a
   tolerant connector match in `prepareOutputForMortyUCD`, otherwise the stored plan is reset.

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
python -m unittest morty.test_live_mode
```
