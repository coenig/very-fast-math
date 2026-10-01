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
