# Three-approach cleanup validation

Implemented in the isolated worktree `/tmp/vfm-queue-cleanup`, based on
`ff00f48`. The original checkout was left untouched because its fleet campaign
was still running. No commit or push was made. Removed workflows can be
recovered from that checkpoint; shared generation/export helpers were moved,
not discarded.

## Checks

* 83 regression tests passed (queue models, solver encodings, monitor, boundary
  clearance, trace completion/validation, fixed/symbolic observations and runner).
* `git diff --check` and Python compilation passed.
* The bundled fleet catalogue previews 18 scenarios × 3 approaches = 54 attempts.
* Sequential refinement is rejected before output creation/model generation.
* Both smoke inputs were run under all three approaches with an overall
  300-second attempt timeout, abstract bound 60 and concrete bound 200.
* All six runs produced validated concrete XML and text traces. All eight
  requested animations rendered (six concrete, two abstract).

## Final smoke results

| Scenario | Enumeration + checking (s) | Direct checking (s) | Abstract + concrete checking (s) |
|---|---:|---:|---:|
| Symbolic 2×2, one car parks | 0.167 | 0.478 | 0.172 |
| Symbolic 2×3, two cars queue then park | 2.346 | 39.216 | 1.643 |

Generation, preprocessing, validation/export and rendering are excluded from
these columns. Full wall times and individual categories are in each run's
JSON/report. These are smoke tests on a busy machine, not a controlled
performance evaluation. Enumeration found a witness on its first candidate in
these tests; there is no general guarantee of that behaviour.

Outputs:

* `/tmp/vfm-queue-cleanup-results/parking_final`
* `/tmp/vfm-queue-cleanup-results/queue_final`

Each directory contains `REPORT.md`, `COMPARISON.md`, `GIFS.md`, `results.json`,
and per-approach models/logs/traces. Abstract diagrams show representative
queue positions; concrete diagrams use actual trace positions.

## Reproduce

```bash
cd /tmp/vfm-queue-cleanup
python -m pytest -q tests/test_queue_*.py tests/test_three_approaches.py \
  tests/test_trace_model.py tests/test_endpoint_boundary.py
python -m vfm_grid_python \
  --config vfm_grid_python/examples/queue_then_park.json \
  --output-dir /tmp/queue-cleanup-reproduction \
  --parallel 3 --timeout 300 --abstract-bound 60 --concrete-bound 200
```

Use a new output directory. Commit the package, migrated tests and their two
`tests/data/queue_pipeline` fixtures when ready; do not include temporary native
library symlinks or generated results. Do not move the cleanup onto a checkout
still used by a running campaign. Adaptive/cluster migration and queue
sequentialization are intentionally not part of this change.
