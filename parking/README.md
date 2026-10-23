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

## Geometry and turning radius

`GEOMETRYSCALING / 1000` is the number of coordinate units per metre. It is
independent of `DISTANCESCALING`, which scales section lengths and vehicle motion.
For example, `GEOMETRYSCALING=2000` gives a 0.5 m coordinate grid. Fractional fixed
section lengths are scaled before rounding. Connector arclengths retain the legacy
policy of rounding metres before applying distance scaling, for every lane count.

`MIN_TURNING_RADIUS=0` disables radius calculations and their generated constraints.
Positive values constrain synthesized connectors. Fully fixed connectors remain
exempt; their geometry is supplied by the caller rather than synthesized.

The radius calculation uses the model's quantized trigonometry and signed integer
division, checks every grid heading and lane, and accepts a distance threshold only
when all allowed larger distances pass. Bounds are computed once per angle and
shared by section pairs. Unhandled relative angles are forbidden when the constraint
is active. Radius-only definitions are omitted from fully fixed models.

The check also accounts for the difference between model drains and the floating-point
drains used to reconstruct roads. Free sections use a coordinate-error bound derived
from the grid and maximum section length; fixed sections use their actual rounded
coordinates and lengths. Consequently, a coarse grid or long free sections can reject
a physically feasible connector. A finer geometry grid, shorter free sections, or a
larger maximum connection distance can reduce this restriction.

Curvature certification uses the convex-hull bounds of the Bezier derivative control
points, with subdivision for inconclusive intervals and rejection if the subdivision
limit is reached. It does not infer safety from point samples. If the endpoint error
is bounded by `e` metres, the connector's derivative control-point errors are bounded
by `(e, 5e, e)` and its second-derivative errors by `(12e, 12e)`; these bounds include
the change in handle length. The calculation uses double precision and a numerical
margin. Sampled curvature is used only as an independent regression-test check.

Focused native regressions can be run without regenerating packages or deleting caches:

```bash
cmake --build build --target runTests --parallel 4
bin/runTests --gtest_filter='ConnectorGeometryTests.*:TurningRadiusTests.*:GeometryScalingTests.*:SectionTemplateTests.*'
```
