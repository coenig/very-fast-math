from __future__ import annotations

import math
import re
from itertools import combinations
from pathlib import Path

import networkx as nx

from benchmarking import (
    BenchmarkCounter,
    BenchmarkResult,
    IterationTimeoutError,
    iteration_timeout,
    save_benchmark_result,
)
from model_checking import Morty


def read_smv_graph_domain(env_model: Path) -> dict:
    text = env_model.read_text()
    section_matches = re.findall(
        r"\bsection_(\d+)_end\s*:\s*(-?\d+)\s*\.\.\s*(-?\d+)",
        text,
    )
    slot_matches = re.findall(
        r"\boutgoing_connection_(\d+)_of_section_(\d+)\s*:", text
    )
    connection_matches = re.findall(
        r"\barclength_from_sec_(\d+)_to_sec_(\d+)_on_lane_0\s*:"
        r"\s*(-?\d+)\s*\.\.\s*(-?\d+)",
        text,
    )
    if not section_matches:
        raise ValueError(f"No section declarations found in {env_model}")

    section_bounds = {(int(low), int(high)) for _, low, high in section_matches}
    sections = sorted({int(section) for section, _, _ in section_matches})
    if sections != list(range(len(sections))):
        raise ValueError(f"Section indexes are not contiguous: {sections}")
    if connection_matches:
        connection_bounds = {
            (int(low), int(high)) for _, _, low, high in connection_matches
        }
    elif len(sections) == 1:
        connection_bounds = {(0, 0)}
    else:
        raise ValueError(f"No lane-0 arclength declarations found in {env_model}")
    if len(section_bounds) != 1 or len(connection_bounds) != 1:
        raise ValueError("approach_3 requires uniform section and connection domains")

    slots_by_section: dict[int, set[int]] = {section: set() for section in sections}
    for slot, section in slot_matches:
        slots_by_section[int(section)].add(int(slot))
    slot_counts = {len(slots) for slots in slots_by_section.values()}
    if len(slot_counts) != 1:
        raise ValueError("Sections have different numbers of connection slots")
    for section, slots in slots_by_section.items():
        if slots != set(range(len(slots))):
            raise ValueError(f"Connection slots for section {section} are not contiguous")

    return {
        "section_count": len(sections),
        "max_outgoing": slot_counts.pop(),
        "section_bounds": section_bounds.pop(),
        "connection_bounds": connection_bounds.pop(),
    }


def iter_section_planar_graphs(section_count: int, max_outgoing: int):
    """Enumerate planar road graphs with at most ``max_outgoing`` connections.

    Only optional ``E_i -> S_j`` connection edges count toward the limit.
    Mandatory ``S_i -> E_i`` section edges do not.
    """
    if section_count < 1:
        raise ValueError("section_count must be positive")
    if max_outgoing < 0:
        raise ValueError("max_outgoing must be non-negative")

    choices_by_section = []
    for section in range(section_count):
        targets = [target for target in range(section_count) if target != section]
        choices_by_section.append(
            [
                selected
                for count in range(min(max_outgoing, len(targets)) + 1)
                for selected in combinations(targets, count)
            ]
        )

    graph = nx.MultiDiGraph()
    for section in range(section_count):
        graph.add_edge(
            f"S_{section}",
            f"E_{section}",
            kind="section",
            section=section,
        )

    def extend(section: int):
        if section == section_count:
            _, embedding = nx.check_planarity(nx.Graph(graph))
            yield graph.copy(), embedding
            return

        for targets in choices_by_section[section]:
            added_edges = []
            for slot, target in enumerate(targets):
                key = graph.add_edge(
                    f"E_{section}",
                    f"S_{target}",
                    kind="connection",
                    conn_number=slot,
                    start_section=section,
                    end_section=target,
                )
                added_edges.append((f"E_{section}", f"S_{target}", key))

            planar, _ = nx.check_planarity(nx.Graph(graph))
            if planar:
                yield from extend(section + 1)

            for start, end, key in added_edges:
                graph.remove_edge(start, end, key)

    yield from extend(0)


def _embedding_positions(
    graph: nx.MultiDiGraph,
    embedding: nx.PlanarEmbedding,
) -> dict:
    simple_graph = nx.Graph(graph)
    if nx.is_connected(simple_graph):
        return nx.combinatorial_embedding_to_pos(embedding)

    positions = {}
    x_offset = 0.0
    for nodes in nx.connected_components(simple_graph):
        component = simple_graph.subgraph(nodes)
        _, component_embedding = nx.check_planarity(component)
        component_positions = nx.combinatorial_embedding_to_pos(
            component_embedding
        )
        min_x = min(point[0] for point in component_positions.values())
        max_x = max(point[0] for point in component_positions.values())
        for node, (x, y) in component_positions.items():
            positions[node] = (x - min_x + x_offset, y)
        x_offset += max_x - min_x + 2.0
    return positions


def add_embedding_lengths(
    graph: nx.MultiDiGraph,
    embedding: nx.PlanarEmbedding,
    section_bounds: tuple[int, int],
    connection_bounds: tuple[int, int],
) -> nx.MultiDiGraph | None:
    positions = _embedding_positions(graph, embedding)
    raw_lengths = []
    scale_low = 0.0
    scale_high = math.inf

    for start, end, key, data in graph.edges(keys=True, data=True):
        x1, y1 = positions[start]
        x2, y2 = positions[end]
        raw_length = math.hypot(x2 - x1, y2 - y1)
        if raw_length == 0:
            return None
        bounds = section_bounds if data["kind"] == "section" else connection_bounds
        scale_low = max(scale_low, bounds[0] / raw_length)
        scale_high = min(scale_high, bounds[1] / raw_length)
        raw_lengths.append((start, end, key, raw_length, bounds))

    if scale_low > scale_high:
        return None

    scale = (scale_low + scale_high) / 2.0
    result = graph.copy()
    for node, (x, y) in positions.items():
        result.nodes[node]["pos"] = (float(x * scale), float(y * scale))
    for start, end, key, raw_length, bounds in raw_lengths:
        length = int(round(raw_length * scale))
        if not bounds[0] <= length <= bounds[1]:
            return None
        result[start][end][key]["length"] = length
    result.graph["embedding_scale"] = scale
    return result


def exact_graph_init(graph: nx.MultiDiGraph, max_outgoing: int) -> str:
    section_ids = sorted(
        data["section"]
        for _, _, data in graph.edges(data=True)
        if data.get("kind") == "section"
    )
    clauses = []
    targets_by_section = {section: [] for section in section_ids}
    for _, _, data in graph.edges(data=True):
        if data.get("kind") == "connection":
            targets_by_section[data["start_section"]].append(data["end_section"])

    for section in section_ids:
        targets = sorted(targets_by_section[section])
        if len(targets) > max_outgoing:
            raise ValueError(
                f"Section {section} has {len(targets)} connections, "
                f"but the model only has {max_outgoing} slots"
            )
        values = targets + [-1] * (max_outgoing - len(targets))
        for slot, target in enumerate(values):
            clauses.append(
                f"env.outgoing_connection_{slot}_of_section_{section} = {target}"
            )

    for start, end, data in graph.edges(data=True):
        length = data.get("length")
        if length is None:
            raise ValueError(f"Edge {start!r} -> {end!r} has no length")
        if data.get("kind") == "section":
            section = data["section"]
            clauses.append(
                f"env.section_{section}_end - "
                f"env.section_{section}_segment_0_pos_begin = {length}"
            )
        elif data.get("kind") == "connection":
            clauses.append(
                f"env.arclength_from_sec_{data['start_section']}"
                f"_to_sec_{data['end_section']}_on_lane_0 = {length}"
            )

    return "INIT (\n  " + " &\n  ".join(clauses) + "\n);\n"


def run(
    morty: Morty,
    *,
    max_iterations: int = 10_000,
    iteration_timeout_seconds: float | None = 60.0,
    max_outgoing_connections: int = 3,
) -> BenchmarkResult:
    """Model-check lazily enumerated planar road graphs.

    The topology generator always creates one mandatory ``S_i -> E_i`` edge
    per section and enumerates up to ``max_outgoing_connections`` optional
    ``E_i -> S_j`` edges from each section end. Non-planar partial graphs are
    pruned during enumeration. For each complete topology, a planar embedding
    is converted to integer edge lengths that fit the generated SMV domains.

    A representable candidate is encoded as one exact INIT constraint in
    ``main_approach_3.smv`` and checked with a fresh nuXmv process. The original
    ``main.smv`` is never modified. The candidate model is replaced on every
    iteration, while ``approach_3_constraints.smv`` keeps a history of emitted
    alternatives. Counterexamples are written directly into the scenario
    directory.

    The timeout applies independently to each candidate after it is yielded by
    the lazy enumerator. Timed-out candidates are skipped and enumeration
    continues until the iterator is exhausted or ``max_iterations`` is reached.
    The common benchmark result records elapsed and iteration counts.
    """
    counter = BenchmarkCounter("approach_3")
    target_path = morty.get_target_path()
    main_smv = target_path / "main.smv"
    candidate_smv = target_path / "main_approach_3.smv"
    domain = read_smv_graph_domain(target_path / "EnvModel.smv")
    if max_outgoing_connections < 0:
        raise ValueError("max_outgoing_connections must be non-negative")
    if max_outgoing_connections > domain["max_outgoing"]:
        raise ValueError(
            f"approach_3 requested {max_outgoing_connections} outgoing "
            f"connections, but the model only has {domain['max_outgoing']} slots"
        )
    possible_targets = domain["section_count"] - 1
    choices_per_section = sum(
        math.comb(possible_targets, count)
        for count in range(
            min(max_outgoing_connections, possible_targets) + 1
        )
    )
    topology_upper_bound = choices_per_section ** domain["section_count"]
    print(
        f"approach_3: enumerating up to {topology_upper_bound} labeled topologies "
        "before planarity pruning"
    )

    base_model = main_smv.read_text()
    candidate_smv.write_text(base_model)
    constraints_file = target_path / "approach_3_constraints.smv"
    constraints_file.write_text(
        "-- Candidate INIT constraints generated by approach_3.\n"
        "-- These are alternatives and must not be enabled simultaneously.\n\n"
    )

    candidate_count = 0
    checked_count = 0
    completed = True
    try:
        candidates = iter_section_planar_graphs(
            domain["section_count"], max_outgoing_connections
        )
        for candidate_count, (topology, embedding) in enumerate(candidates, start=1):
            if counter.attempted_runs >= max_iterations:
                completed = False
                break

            counter.attempted()
            try:
                with iteration_timeout(iteration_timeout_seconds):
                    graph = add_embedding_lengths(
                        topology,
                        embedding,
                        domain["section_bounds"],
                        domain["connection_bounds"],
                    )
                    if graph is not None:
                        checked_count += 1
                        init_constraint = exact_graph_init(
                            graph, domain["max_outgoing"]
                        )
                        with constraints_file.open("a") as output:
                            output.write(f"-- Candidate {candidate_count}\n")
                            output.write(init_constraint)
                            output.write("\n")

                        candidate_smv.write_text(
                            base_model + "\n" + init_constraint
                        )
                        print(
                            "approach_3: checking planar candidate "
                            f"{candidate_count} ({graph.number_of_edges()} edges)"
                        )
                        morty.model_check(
                            f"cex_it_{candidate_count - 1}.xml",
                            candidate_count - 1,
                            model_file=candidate_smv.name,
                        )
            except IterationTimeoutError as error:
                counter.timed_out()
                print(
                    f"approach_3: iteration {candidate_count - 1} "
                    f"timed out: {error}"
                )
            else:
                counter.succeeded()
            finally:
                morty.close_model_checker()
    finally:
        morty.close_model_checker()

    print(
        f"approach_3: enumerated {counter.attempted_runs} planar topologies; "
        f"checked {checked_count} with representable metric embeddings"
    )
    result = counter.result(completed=completed)
    save_benchmark_result(result, target_path)
    return result


approach_3 = run
