# Parking MC Parker — usage

GUI to inspect/edit obstacle layouts and re-run the model checker. It races all configured
config variants and adopts the **first counterexample**, killing the slower ones.

## Multiple configs = the race
Give a `#`-variable a `>1` range in `src/templates/envmodel_config.tpl.json`
(e.g. `range(@sections, 3, 10)`). `generateEnvmodels` then emits one package per value
(`examples/gp_config_sections=3 … =10`; several ranges → cross-product). Every run
model-checks all of them at once; the first to find a counterexample wins.

## Commands

Already raced (config unchanged) — just open the winner:
```
python3 parking/mc_parker.py
```

From scratch (new/changed config):
```
# 1. generate the packages (fast, no nuXmv)
python3 parking/_mc_worker.py '@{../src/templates/envmodel_config.tpl.json}@.generateEnvmodels'

# 2. race + open the winner (detached, auto-kills the losers)
python3 parking/mc_parker.py --race
```

Run from the repo root with the venv active (`source .venv/bin/activate`).

## Watching a run
MC runs can be lengthy. In another terminal, monitor the live nuXmv instances (per-config
runtime + CEX/blind result as each finishes):
```
./watch_processes.bash
```

## Rules of thumb
- Changed the config? Re-run step 1 (`generateEnvmodels`) before racing — `runMCJobs` does
  **not** regenerate from the config.
- **Never** run `runMCJobs` directly in a shell: the nuXmv instances grab the terminal's stdin
  and drop you into a shared interactive prompt. `--race` runs it detached (`stdin=/dev/null`),
  which avoids this and stops the losers at the first counterexample.
- In-GUI, the **"Re-run Model Checker (first counterexample wins)"** button repeats the race
  with your edited obstacles.

## How the winner is picked
The package with a complete counterexample (CEX-marked trace **and** a freshly written preview)
whose preview is newest — i.e. the one that finished first while the others were killed. Stale
leftovers from earlier runs (older timestamps) and unfinished/killed variants are ignored.

## Free obstacles: force exactly N moves
A *move* is one drive in a single gear (moves = gear changes + 1). `synth_obstacles.py` lets the
MC place some obstacles such that parking **needs exactly** `PARK_MOVES` moves:
```
python3 parking/synth_obstacles.py            # [--apply] [--max-iters 30] [--max-sections 6]
```
Config (`envmodel_config.tpl.json`):
- `PARK_MOVES`: N (start small, e.g. 2). With N > 0 the normal race also asks for "within N moves".
- `RECT_OBSTACLES_SHIFT_{MIN,MAX}_{X,Y}s`: per obstacle, how far (m) the MC may translate the box
  given by `RECT_OBSTACLES_*`; all zero = fixed. `RECT_OBSTACLES_SHIFT_STEP` is the grid (m).
- `RECT_OBSTACLES_GROW_{MIN,MAX}_{W,H}s`: per obstacle, how much (m) the MC may grow (> 0) or
  shrink (< 0) the box; the bottom-right corner moves, combine with a shift for other anchors.
  Missing list entries mean "fixed"; boxes never shrink to zero size.
- `BMC_CNT: 0`: the reachability model is all-frozen, so a single BMC check at bound 0 is a
  complete proof (`REACHABILITY_ONLY` always uses BMC; IC3 was minutes vs. seconds here).
- Keep `@sections` at most 6; larger packages make every proof very slow (the script refuses them).

Each iteration first lets the MC pick shifts that allow an N-move route, then fixes them and asks
every package for a route with fewer moves. If none exists, the placement is certified (within the
model); otherwise that shorter route becomes a lemma ("some free box must block it") and the loop
repeats. `--apply` writes the result into `RECT_OBSTACLES_*`, resets the shifts and regenerates.
Without it, `python3 parking/mc_parker.py` opens the certified solution. In the GUI a free box is
drawn where the MC put it; dragging it moves its configured base box (the shift range stays).

The search is directed: it first checks the configured placement for a shorter route (seed; skip
with `--no-seed`), and candidates first have to cover every refuting route with `--margin` (default
0.5 m) to spare, so near-identical routes are blocked too. If no placement manages that, it falls
back to plain blocking, which alone decides "no solution".

In the GUI, **"Synthesize Free Obstacles"** runs the same loop (writing pending scene edits to the
config first) and shows every step live: each candidate with its N-move route, then the shorter
route refuting it (Play animates it). Refuted placements stay as dashed red outlines; the status
lines above the progress bar explain the current step and why the search ended. Snapshots and the
full log are in `examples/synth_history/`. "Terminate Model Checker" stops it.
