# N/P grids and VFM refinement

Run from the repository root, with the existing virtual environment activated.
The package uses the VFM shared library, Kratos and nuXmv already used by the
project. Matplotlib and Pillow provide the visualizations.

## Full pipeline

```bash
python -m vfm_grid_python \
  --config vfm_grid_python/examples/yielding_small.json \
  --output-dir vfm-with-parking/new-grid-yielding \
  --bound 30 --concrete-bound 200 --timeout 1800
```

Use a new output directory for each attempt. `--timeout` is the limit in seconds
for **each** abstract check, native generation, and concrete check. Rendering is
outside model-checking time. `--road-length 20 --parking-length 7
--junction-radius 6` are the geometry defaults, in metres. Model generation is
logged to `concrete/generation.log`; nuXmv output is in `checking.log` beside each
model. `check_result.json` records status, bound and elapsed checking time.

Examples:

- `simple.json`: one car drives along two roads and parks.
- `reverse_junction.json`: a parked car reverses out, crosses a start-to-start
  connector and reaches a different parking branch.
- `yielding_small.json`: car 0 waits in parking 4, which is not its goal,
  allowing car 1 to pass and park; car 0 then leaves and reaches parking 3.
- `yielding.json`: a longer opposing-car maneuver with three parking branches.
- `symbolic.json`: a 3×5 grid with symbolic edges, N/P labels and goals.
- `visit_all_3x3.json`: one car visits all nine nodes, in solver-chosen order,
  and then parks. Only the 3×3 dimensions, one car, and one parking count are
  specified; edges, labels, start and goal remain symbolic. Its property is
  the conjunction of `F (loc_0 = n & F all_parked)` for every node n.
- `symbolic_yielding.json`: fully symbolic 2×3 grid, exactly three parking
  nodes, and two cars. A moves into a foreign parking next to its starting
  junction and waits while B visits and leaves that junction; both then park.
  No edges, coordinates of parking nodes, starts or goals are fixed.

The maneuver examples fix edges for reproducible tests. Remove `edges`, `labels`
and/or `goals` to let nuXmv choose them; these fields are independent. Grid size
and car count can also be overridden with `--rows`, `--columns`, `--cars` (adjust
any explicit start/goal lists accordingly).

## Separate steps

```bash
python -m vfm_grid_python.abstract \
  --config vfm_grid_python/examples/reverse_junction.json \
  --output-dir /tmp/grid-abstract --bound 30 --timeout 1800

python -m vfm_grid_python.concretize /tmp/grid-abstract \
  --output-dir /tmp/grid-concrete --concrete-bound 200 --timeout 1800

python -m vfm_grid_python.visualize /tmp/grid-concrete
```

Concretization also supports `--generate-only`. `concrete/model.json` gives the
absolute SMV path. Its directory contains `trace.xml` and
`debug_trace_array.txt`; the latter is exported by nuXmv with
`show_traces -o debug_trace_array.txt 1`. Geometry, topology PNG and concrete GIF
are directly under the concrete output directory. The abstract directory holds
its own SMV, XML/text traces, selected graph JSON, PNG and GIF.

## Sequential local concretization

To refine an existing abstract witness one transition at a time:

```bash
python -m vfm_grid_python.sequential /path/to/abstract \
  --output-dir /path/to/new-concrete \
  --concrete-bound 500 --timeout 3600
```

The input directory needs `selected_graph.json` and `config.json`. Each local
model retains the incident sections of the current and next abstract positions,
including stationary vehicles. All persistent state is transferred to the next
check, with local section numbers mapped to global numbers. At a destination
junction, an arriving connector must touch the road needed by the next move
(looking beyond repeated waits). This prevents a valid abstract node arrival
from stranding the next query on an unrelated concrete connector. Each invocation of
model generation, flattening or checking has the specified timeout; it is not
a total time limit for the entire pipeline.

Cars that wait during the current abstract move, or must wait during the next
one, finish the local query with zero speed and zero acceleration. This avoids
handing a waiting car to the next query too close to a connector end to brake.
The exported monitor phase is recomputed from the first physical observation of
each milestone; braking may finish after that observation. The full concrete
monitor and transition constraints still validate the assembled execution.

`--model-cache /path/to/previous-concrete/models` reuses completed local models
from the same VFM build and templates. Geometry, fleet size and VFM parameters
must match. Use a fresh output directory for the new queries; the cache is read
only. Do not reuse a cache after changing its template files or native build.

After all segments succeed, the script generates and flattens the complete
concrete monitor model once, **without running a global search**. Python uses
this model to reconstruct every declared state variable and every state-only
`DEFINE`, including unused occupancy aliases and constant geometry fields.
It checks domains, frozen values, initial conditions, invariants, transitions,
assignments and derived values over the concatenated trace. Unsupported model
syntax fails explicitly. The existing independent motion/collision validator
also runs. Internal flattening helpers and expressions involving `next` are not
state observations and are listed separately from the exported state symbols.
Before publishing the final XML, nuXmv imports and re-exports it using only
`build_flat_model` and the trace reader/writer. Python compares the returned
values. This compatibility check performs no SMT/BDD search.

Successful outputs directly under the concrete directory are:

- `trace.xml`: full valuations in every state, using nuXmv's XML trace format.
- `debug_trace_array.txt`: the same concatenated trace in readable, delta format.
- `model_validation.json`: full-model validation and symbol counts.
- `validation.json`: geometry/movement validation and full-model report.
- `global_model/inventory.json`: the global symbol inventory and model path.
- `topology.png` and `concrete.gif`: Matplotlib visualizations (GIF skipped with
  `--no-gif`).

An unfinished run retains `partial_trace.xml` and a partial text trace. These
diagnostic files are not certified as complete global-model exports. A final
`trace.xml` is only published after validation succeeds.

`--validation-model /path/to/main_endpoint_grid.smv` reuses an existing matching
global monitor model instead of generating one. `--replay-model` additionally
asks nuXmv to validate the fixed execution, which is optional and distinct from
the Python checks.

`--backtracks 2` permits up to two retries of a preceding segment when a later
segment times out or has no witness within its bound. Each retry excludes the
exact unsuccessful terminal state and preserves attempt directories. This is a
bounded recovery strategy, not a completeness guarantee. By default backtracking
is disabled. A one-state abstract witness is checked at its initial state;
repeated milestones in a longer witness still require a real transition.

If all local segments succeeded but final global VFM generation failed, resume
only that final stage after changing the VFM build or heap setting:

```bash
python -m vfm_grid_python.finalize \
  /path/to/concrete-run --timeout 3600
```

This reads `partial_trace.xml`, reuses the saved geometry and selected graph,
and writes the final `trace.xml`, validation report and GIF. It does not rerun
the abstract or local concrete checks.

## Model semantics

Node IDs are `row * columns + column`. Every orthogonal edge is optional, with
no mandatory spanning path. Each frozen `parking_ID` label selects N or P.
Every P has exactly one selected neighbor, which must be N. Disconnected unused
nodes are permitted; occupied nodes have a selected incident edge. Neither
global connectivity nor a particular parking count is imposed implicitly.
Set `parkings` in JSON, or `--parkings` on the command line, to impose an exact
parking count. There must be at least as many parking nodes as cars because
their final goals are distinct.

Cars wait or make one atomic move per abstract tick. Distinct cars cannot share
a node or exchange positions along an edge. Simultaneous moves into vacated
nodes remain possible. Cars start on N by default, have distinct symbolic P
goals, and may use other parking nodes temporarily. `start_types` can specify
`N`, `P`, or `any` per car; `starts` and `goals` give optional explicit IDs.

Configuration accepts `initial_constraints` (INIT expressions) and `constraints`
(invariant expressions) over `loc_i`, `goal_i`, `parking_ID`, `degree_ID`, and
`edge_U_V`. `parked_i` means that car i occupies a P; `all_parked` means every
car occupies its assigned goal.

`start_i` is a frozen copy of car i's initial node, chosen symbolically unless
`starts` is supplied. `foreign_parked_i` means the car is parked somewhere other
than its goal; `beside_start_i` means its present node is adjacent, through a
selected edge, to its initial node. `adjacent_i_j` (i < j) tests current adjacency.
`distance_to_goal_i` is the Manhattan distance in grid cells, so a property can
request distant goals without selecting coordinates.

Optional `roles`, for example `{"gate": "N", "slot": "P"}`, introduce symbolic
locations named `role_gate` and `role_slot`. Their coordinates are chosen by
nuXmv. `role_gate_degree` exposes the selected degree. Roles need not be distinct
unless the property says so. These name places in the scenario, not fixed cells.

## Larger symbolic scenarios

`examples/two_shared_gates_5x5_8cars.json` selects a 5×5 partial grid with eight
cars and eight parking nodes. Cars 0–3 visit gate A before reaching their goals;
cars 4–7 visit gate B. Both gates have degree two. At each gate a car must wait
on an adjacent queue node while another occupies the gate, and later enter the
gate itself. All locations and edges are symbolic. Visiting a gate does not
require crossing to its opposite side, and the property does not assert that
the gate is a graph-theoretic cut vertex.

```bash
python -m vfm_grid_python.abstract \
  --config vfm_grid_python/examples/two_shared_gates_5x5_8cars.json \
  --output-dir vfm-with-parking/my-two-gates/abstract --bound 100 --timeout 3600

python -m vfm_grid_python.sequential vfm-with-parking/my-two-gates/abstract \
  --output-dir vfm-with-parking/my-two-gates/concrete \
  --concrete-bound 500 --timeout 3600 --backtracks 2
```

`examples/large_scenarios.json` contains five three-car scenarios:

| Scenario | Grid | Parking count | Desired history |
|---|---|---|---|
| `shared_gate_5x5` | 5×5 | 4 | A, B and C visit the same degree-two gate in order, then park. |
| `shared_spot_5x5` | 5×5 | 4 | A and B temporarily occupy C's future parking spot, then C takes it. |
| `double_yield_5x5` | 5×5 | 4 | A waits in a foreign parking while B and C visit A's original junction. |
| `distant_parking_8x5` | 8×5 | 5 | All cars reach goals at least six grid cells from their starts. |
| `gate_roundtrip_8x5` | 8×5 | 5 | The fleet passes a shared gate and parks; A unparks, revisits the gate and parks again. |

These properties also impose minimum initial goal distances. They require the
specified maneuvers to occur; they do not prove that every alternative strategy
must use those maneuvers, or that a named gate is a graph-theoretic cut vertex.

Search all abstract scenarios and generate their GIFs:

```bash
python -m vfm_grid_python.batch \
  --catalog vfm_grid_python/examples/large_scenarios.json \
  --output-dir vfm-with-parking/my-large-search \
  --bound 100 --timeout 1800
```

Add `--concretize --concrete-bound 300` to refine each successful abstract
witness with VFM and generate concrete GIFs. The batch saves `summary.json`
after each stage. `--only shared_gate_5x5` selects an individual scenario.

The default property is `INVARSPEC !all_parked`. An optional `desired_ltl` is
negated to obtain a witness of the desired history. Use finite histories with
F, U and X, as in the examples. The concrete monitor reproduces the ordered
configurations, allowing multiple concrete ticks per abstract tick: X refers
to the next abstract configuration, not the next concrete timestep. It does
not establish infinite recurrence or fairness for arbitrary LTL formulas.

Every selected edge becomes **one physical, single-lane section**. N–P sections
point toward P. A car is concretely parked only at that section's exact end
with velocity zero. Junctions have room for curves; connector lengths are
additional to straight-section lengths. P endpoints are shortened toward their
N neighbor, so the concrete drawing need not retain uniform grid spacing.
The entire geometry is rigidly transformed to place VFM section 0 at (0,0),
angle 0. Missing edges create no concrete sections or plotted connections.

VFM receives fixed topology and geometry. It retains its vehicle variables,
acceleration limits, speed bounds, lane restrictions and same-section collision
constraints. `ENDPOINT_CONNECTIONS=true` replaces the seclet movement relation
with one that supports either endpoint of a section. Acceleration is applied
in the current local frame; position and signed velocity are converted together
when entering/exiting a connector. Unchanged models use the default flag value
false. The Python compiler supplies the fixed relation via
`EnvModel_EndpointConnections.tpl`; the flag alone does not synthesize geometry.

VFM options under `vfm` override non-structural parameters of `main_template.json`.
In particular, the examples enable negative speed with `LANES_MIN_SPEEDS`.
The pipeline does not silently enable backward driving. The template config can
be selected with `--template-config`. This compiler uses fixed geometry metadata
rather than re-running VFM's generic symbolic geometry search.

The monitor restricts each car to its current abstract edge and endpoint
junctions. It conservatively reserves whole physical edges and junction
envelopes, including across tick boundaries. This prevents head-on motion and
crossing-curve conflicts but can reject feasible close convoys. An abstract
witness is a candidate, not a guarantee of concrete feasibility; a bound limit
or timeout is reported as such. Successful traces are independently checked
for movement, milestones, route adherence and reserved-space conflicts.

## Stable concurrent milestones (opt-in)

`semantics = "stable_concurrent"` selects the new contract; the default remains
`legacy`, so existing campaigns are unchanged. Each abstract node has a
deterministically selected incident holding section. At every concrete milestone,
**all cars have zero velocity and acceleration** at their canonical endpoint.
Waiting cars remain there. A moving car reserves its travel section, endpoint
holding sections and normal-node turning regions. Joint moves with overlapping
reservations are rejected; multiple cars may move in the same abstract step when
their reservations are disjoint. This is conservative, not single-car scheduling.
Scenario events are observed at these boundaries; stage holds also constrain
motion between boundaries.

The initial supported profile has 20 m road sections, 7 m parking sections and
6 m junction radius, with finite cars of length 5 m and width at most 2 m.
Unsupported geometry/profile overrides fail explicitly. Each run records a
`contract.json`; sequential refinement also independently checks abstract
reservations in `contract_validation.json`. All three benchmark methods use the
same milestone and reservation contract. Direct concrete search chooses joint
targets in the full model; it does not invoke an abstract solver first.

The paper's conditional realizability argument assumes real-valued reversible
motion, adequate geometric clearance and extendible auxiliary dynamics.
It does **not** guarantee a witness within an integer BMC bound, or preserve
every feasible concrete maneuver. Full concrete trace validation remains required.

Run the new catalog in a fresh directory (timeout is per whole method job):

```bash
python -m vfm_grid_python.benchmark \
  --catalog parking_benchmark_scenarios/stable_concurrent.toml \
  --output-dir vfm-with-parking/stable-concurrent/campaign_01 --timeout 1800
python -m vfm_grid_python.benchmark_report \
  vfm-with-parking/stable-concurrent/campaign_01 \
  --output-dir vfm-with-parking/stable-concurrent/report --gifs
```

Use `--semantics stable_concurrent` to override another benchmark catalog, or set
the same field in an abstract configuration before generating a **new** witness.
Do not relabel old traces: they may violate the contract. Reports distinguish
contract versions rather than pooling legacy and stable measurements.

## Visualization compatibility

Both this package's Matplotlib renderer and the native C++ GIF renderer support
endpoint-aware traces. Native rendering reads `env.endpoint_S_T_from_end` and
`env.endpoint_S_T_to_end` to select the correct section attachments, including
start-start and end-end connections. These flags also control car placement and
smooth interpolation across section/connector boundaries. The trace's rounded
connector length is used as the position coordinate range. Without the flags,
the native renderer retains its legacy end-to-start convention.

Rebuild the native library/binary after updating the renderer:

```bash
cmake --build build --target vfm -j 2
python render_concrete_xml.py /path/to/concrete/trace.xml --output-dir /path/to/new/native_render
```

Use the **concrete** XML or text trace, not the abstract grid trace: only the
concrete trace contains section coordinates, connector flags and vehicle
positions along sections. No model-checking rerun is needed for existing
endpoint-aware concrete traces. The Python command above calls `lib/libvfm.so`
and creates native VFM GIFs, including a smooth bird's-eye animation.

## Tests

```bash
python -m unittest discover -s tests -p test_vfm_grid_python.py
```

These include nuXmv checks of all 16 endpoint entry/exit combinations, symbolic
parking selection, forbidden adjacent swaps, and geometry validation.
