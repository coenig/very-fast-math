#!/usr/bin/env python3
"""Compare symbolic and externally enumerated partial-grid yielding search."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import multiprocessing as mp
import os
import resource
import shutil
import signal
import time
import traceback
from dataclasses import asdict
from pathlib import Path
from typing import Iterator

import networkx as nx

from abstract_grid_yielding import (
    YieldingGridConfig,
    render_smv,
    save_gif,
    selected_graph,
)
from scalable_grid_parking import check_model, grid_edges
from scalable_parking import parse_states


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT = ROOT / "vfm-with-parking" / "enumeration-comparison"
RESULT_FIELDS = (
    "case_id", "method", "rows", "columns", "nodes", "possible_edges",
    "topology_count", "cars", "fuel_budget", "edge_cost", "bound", "status",
    "elapsed_seconds", "generation_seconds", "checking_seconds",
    "visualization_seconds", "nuxmv_invocations", "candidates_checked",
    "winning_candidate_zero_based", "winning_candidate_one_based",
    "winning_edges", "counterexample_states", "peak_child_rss_kb",
    "run_directory", "error",
)


def iter_partial_grids(config: YieldingGridConfig) -> Iterator[tuple[int, set[tuple[int, int]]]]:
    """Yield the full edge power set by cardinality, then lexicographically."""
    edges = tuple(tuple(sorted(edge)) for edge in grid_edges(config))  # type: ignore[arg-type]
    index = 0
    for count in range(len(edges) + 1):
        for selected in itertools.combinations(edges, count):
            yield index, set(selected)
            index += 1


def graph_from_edges(config: YieldingGridConfig,
                     enabled: set[tuple[int, int]]) -> nx.Graph:
    graph = nx.Graph()
    for cell in range(config.nodes):
        row, column = divmod(cell, config.columns)
        graph.add_node(cell, row=row, column=column)
    graph.add_edges_from(sorted(enabled))
    return graph


def save_topology(graph: nx.Graph, run_dir: Path) -> None:
    nx.write_graphml(graph, run_dir / "selected_grid.graphml")
    (run_dir / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )


def base_result(method: str, config: YieldingGridConfig, run_dir: Path) -> dict:
    possible_edges = len(grid_edges(config))  # type: ignore[arg-type]
    result = {field: "" for field in RESULT_FIELDS}
    result.update(
        case_id=f"grid_{config.rows}x{config.columns}__{method}", method=method,
        rows=config.rows, columns=config.columns, nodes=config.nodes,
        possible_edges=possible_edges, topology_count=2 ** possible_edges,
        cars=config.cars, fuel_budget=config.fuel_budget,
        edge_cost=config.edge_cost, bound=config.bound, status="error",
        generation_seconds=0.0, checking_seconds=0.0,
        visualization_seconds=0.0, nuxmv_invocations=0,
        candidates_checked=0, run_directory=str(run_dir.relative_to(ROOT)),
    )
    return result


def finish_witness(result: dict, counterexample: Path, graph: nx.Graph,
                   config: YieldingGridConfig, run_dir: Path) -> None:
    target = run_dir / "counterexample.xml"
    if counterexample != target:
        shutil.move(counterexample, target)
    states = parse_states(target)
    save_topology(graph, run_dir)
    started = time.perf_counter()
    save_gif(target, graph, config, run_dir / "scenario.gif")
    result["visualization_seconds"] = time.perf_counter() - started
    result.update(
        status="witness", winning_edges=graph.number_of_edges(),
        counterexample_states=len(states),
    )


def execute_symbolic(config: YieldingGridConfig, run_dir: Path) -> dict:
    started = time.perf_counter()
    result = base_result("symbolic", config, run_dir)
    model = run_dir / "symbolic.smv"
    tick = time.perf_counter()
    model.write_text(render_smv(config))
    result["generation_seconds"] = time.perf_counter() - tick
    tick = time.perf_counter()
    result["nuxmv_invocations"] = 1
    counterexample = check_model(model, config.bound, temporal=True)
    result["checking_seconds"] = time.perf_counter() - tick
    if counterexample is None:
        result["status"] = "no_witness_within_bound"
    else:
        states = parse_states(counterexample)
        finish_witness(result, counterexample, selected_graph(config, states[0]),
                       config, run_dir)
    result["elapsed_seconds"] = time.perf_counter() - started
    result["peak_child_rss_kb"] = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    return result


def execute_enumerated(config: YieldingGridConfig, run_dir: Path) -> dict:
    started = time.perf_counter()
    result = base_result("enumerated", config, run_dir)
    candidate_model = run_dir / "candidate.smv"
    for index, enabled in iter_partial_grids(config):
        result["candidates_checked"] = index + 1
        tick = time.perf_counter()
        candidate_model.write_text(render_smv(config, enabled))
        result["generation_seconds"] += time.perf_counter() - tick
        tick = time.perf_counter()
        result["nuxmv_invocations"] += 1
        counterexample = check_model(candidate_model, config.bound, temporal=True)
        result["checking_seconds"] += time.perf_counter() - tick
        if counterexample is not None:
            shutil.copy2(candidate_model, run_dir / "winning_fixed.smv")
            result["winning_candidate_zero_based"] = index
            result["winning_candidate_one_based"] = index + 1
            finish_witness(result, counterexample,
                           graph_from_edges(config, enabled), config, run_dir)
            break
    else:
        result["status"] = "exhaustive_no_witness"
    result["elapsed_seconds"] = time.perf_counter() - started
    result["peak_child_rss_kb"] = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    return result


def worker(method: str, config_data: dict, run_dir_text: str) -> None:
    os.setsid()
    run_dir = Path(run_dir_text)
    config = YieldingGridConfig(**config_data)
    with (run_dir / "stdout.log").open("w") as stdout, \
         (run_dir / "stderr.log").open("w") as stderr:
        try:
            import contextlib
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                result = (execute_symbolic(config, run_dir) if method == "symbolic"
                          else execute_enumerated(config, run_dir))
        except BaseException as error:
            traceback.print_exc(file=stderr)
            result = base_result(method, config, run_dir)
            result.update(status="error", error=f"{type(error).__name__}: {error}")
    (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")


def run_with_timeout(method: str, config: YieldingGridConfig, run_dir: Path,
                     timeout: int) -> dict:
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
    process = mp.Process(target=worker, args=(method, asdict(config), str(run_dir)))
    started = time.perf_counter()
    process.start()
    process.join(timeout)
    if process.is_alive():
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        process.join(5)
        if process.is_alive():
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.join()
        result = base_result(method, config, run_dir)
        result.update(status="timeout", elapsed_seconds=time.perf_counter() - started,
                      error=f"exceeded whole-case timeout of {timeout} seconds")
        (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    result_file = run_dir / "result.json"
    if not result_file.exists():
        result = base_result(method, config, run_dir)
        result.update(status="error", elapsed_seconds=time.perf_counter() - started,
                      error=f"worker exited {process.exitcode} without a result")
        return result
    return json.loads(result_file.read_text())


def write_reports(results: list[dict], output: Path) -> None:
    (output / "results.jsonl").write_text(
        "".join(json.dumps(result) + "\n" for result in results)
    )
    with (output / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(results)
    try:
        from matplotlib import pyplot as plt
        labels = [f"{r['rows']}x{r['columns']}\n{r['method']}" for r in results]
        values = [float(r.get("elapsed_seconds") or 0) for r in results]
        figure, axes = plt.subplots(figsize=(max(8, len(results) * 1.15), 5))
        axes.bar(labels, values, color=["#4c78a8" if r["method"] == "symbolic"
                                       else "#f58518" for r in results])
        axes.set_ylabel("End-to-end seconds")
        axes.set_title("Symbolic vs externally enumerated partial-grid search")
        axes.tick_params(axis="x", rotation=30)
        figure.tight_layout()
        figure.savefig(output / "runtime_comparison.png", dpi=160)
        plt.close(figure)
    except ImportError:
        pass


def parse_grid(value: str) -> tuple[int, int]:
    try:
        rows, columns = (int(item) for item in value.lower().split("x", 1))
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError("grid must be ROWSxCOLUMNS") from error
    if rows < 2 or columns < 2:
        raise argparse.ArgumentTypeError("grid dimensions must be at least 2")
    return rows, columns


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--grids", nargs="+", type=parse_grid,
                        default=[(2, 2), (2, 3), (3, 3), (3, 4)])
    parser.add_argument("--cars", type=int, default=2)
    parser.add_argument("--fuel", type=int, default=100)
    parser.add_argument("--edge-cost", type=int, default=10)
    parser.add_argument("--bound", type=int, default=40)
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_root = args.output_dir / f"run_{time.time_ns()}"
    run_root.mkdir()
    results = []
    for rows, columns in args.grids:
        config = YieldingGridConfig(rows=rows, columns=columns, cars=args.cars,
                                    fuel_budget=args.fuel, edge_cost=args.edge_cost,
                                    bound=args.bound)
        config.validate()
        for method in ("symbolic", "enumerated"):
            run_dir = run_root / f"grid_{rows}x{columns}__{method}"
            result = run_with_timeout(method, config, run_dir, args.timeout)
            results.append(result)
            write_reports(results, run_root)
            print(f"{result['case_id']}: {result['status']} "
                  f"({result.get('elapsed_seconds', '')} s)")
    print(f"Results: {run_root.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
