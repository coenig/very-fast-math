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
