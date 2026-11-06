# Queue-based parking scenario synthesis

This package supports **three approaches with the same scenario language and
shared concrete compiler**. Run commands from the repository root.

| Approach | Implementation | Topology selection |
|---|---|---|
| External enumeration | `approaches/external_enumeration.py` | Property-blind partial-grid enumeration; check VFM until the first validated witness |
| Direct model checking | `approaches/direct_model_checking.py` | Frozen optional edges and parking labels inside one VFM query |
| Abstraction based | `approaches/abstraction_based/` | Queue model searches topology/scenario, then a whole-trace concrete monitor refines the witness |

**Queue sequentialization is not implemented.** Selecting it fails before any
generation or solver execution. The retired endpoint sequentializer is not a
valid substitute.

## Run all three

Prerequisites: Python dependencies from the repository requirements, `nuXmv`
on PATH, the working VFM shared library and Kratos used by
`scenario_generation.py`, and `main_template.json`. Native generation logs go
to each attempt's `generation.log`; they are not printed to the terminal.

```bash
python -m vfm_grid_python \
  --config vfm_grid_python/examples/queue_then_park.json \
  --output-dir /tmp/queue-comparison \
  --approach external_enumeration direct_model_checking abstraction_based \
  --abstract-bound 60 --concrete-bound 200 \
  --timeout 300 --parallel 3
```

The default is all three approaches. Supply one name to run only that approach.
The timeout is **one overall wall-time budget per attempt**, including abstract
search and refinement where applicable. A driver watchdog has 15 seconds of
cleanup grace. A timeout is not an unsatisfiability result. Bounds are upper
limits, not desired trace lengths.

The runner refuses existing output directories. It writes `campaign.json`,
`results.json`, `REPORT.md`, `COMPARISON.md`, per-attempt models/logs,
selected geometry and validated XML/text witnesses. JSON records separate
checking, generation, setup and export/validation costs. For abstraction,
checking sums abstract and concrete queries. For enumeration, search time adds
candidate-generation time to all concrete checking calls.

Successful results generate GIFs after measured attempts; use `--no-gif` to
skip. Rendering failures are separate from witness validity.

```bash
python -m vfm_grid_python --output-dir /tmp/queue-comparison --report-only
python -m vfm_grid_python.queue_render /tmp/queue-comparison
```

Use `--catalog FILE.toml` instead of `--config` for existing queue family/scale
catalogues, `--only INSTANCE ...` for a subset, and `--preview` to inspect jobs
without running them. Positive event/hold paper families remain supported;
negative predicates and legacy motion profiles are rejected, not reinterpreted.

## The input contract

A JSON config specifies rows, columns, cars, parking count, optional rigid
roles, and `scenario_stages`. Each stage has an `event` and optional `hold`.
The observer chooses when to consume events; while waiting for a stage, its hold
must remain true. Events occur at distinct ticks.

Supported positive predicates:

* `{"at": [car, "goal_0"]}` (also starts and symbolic roles).
* `{"attachment": [car, "role_node", "role_neighbour"]}`.
* `{"queue": {"cars": [0,1], "kind": "section", "ordered": false}}`.
  Queue predicates can specify `between` roles or a `near` role; connections
  and either road kind are also supported.
* `{"all": [...]}`, `{"any": [...]}`, and Boolean constants.

Absent edges/labels/starts/goals are symbolic. Parking nodes have one ordinary
neighbour. Each selected grid edge becomes one reversible concrete section;
incident sections are joined by precomputed connectors.

Defaults: ordinary roads 20m, parking roads 7m, junction radius 6m, abstract
connection length 9m, queue clearance 6m. Concrete connector lengths are computed
from their actual curves; refinement checks packing against those lengths.
Therefore an abstract queue witness is **not guaranteed** to be concretizable.

The concrete compiler uses native VFM dynamics, endpoint transitions, local
braking limits, same-road clearance/order, exact-joint exclusivity and road
exchange restrictions. There are **no whole-section/junction reservations**.
Boundary clearance is a conservative incident-road rule, not general geometric
collision checking between intersecting curves. Concrete parking observations
require zero speed. The monitor alone adds witness-route restrictions and
holds unchanged endpoint occupants stationary.

## Code map

* `approaches/`: algorithms, not duplicated dynamics.
* `concrete/`: generation worker, fixed/symbolic property compiler,
  observations, dynamics, braking, model validation and trace export.
* `scenarios/`: shared event/hold language, candidate road IDs, symbolic families,
  property-blind topology enumeration and input validation.
* `benchmark.py`, `benchmark_timing.py`, `benchmark_report.py`: local execution
  and measurements.
* `geometry.py`, `visualize.py`, `queue_visualize.py`, `queue_render.py`:
  geometry and visualization.

Direct and external concrete checking share `concrete/compiler.py`. The queue
monitor uses the same observations and safety predicates. No abstract search is
called by either direct or enumeration checking.

## Migration and reproducibility

This cleanup replaces the old endpoint/stable workflows. The checkpoint
`ff00f48` retains those implementations. Old `abstract`, `concretize`,
`sequential`, `queue_compare`, `queue_campaign`, adaptive and cluster commands
are not supported by this package version; use the single runner above.
Adaptive scaling/cluster migration is future work.

New results carry `semantics=ordered-queues.v1` and
`compiler=shared-queue-concrete.v1`. Historical method IDs are retained as
metadata (`enumerated_concrete`, `direct_symbolic`, `abstract_monitor`), but
that does **not** make old endpoint runs semantically equivalent. Do not pool
old and new results without identifying their model versions.

Root-level VFM helpers are still dependencies; this cleanup does not remove
them or modify VFM templates. Do not include machine-specific `lib`/`bin`
symlinks or generated result directories when committing this refactor.

Regression tests for the package are `tests/test_queue_*.py`,
`tests/test_three_approaches.py`, `tests/test_trace_model.py` and
`tests/test_endpoint_boundary.py`. The examples are deliberately small,
symbolic smoke tests, not performance conclusions.
