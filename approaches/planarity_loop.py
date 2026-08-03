from __future__ import annotations

import networkx as nx

from benchmarking import (
    BenchmarkCounter,
    BenchmarkResult,
    IterationTimeoutError,
    iteration_timeout,
    save_benchmark_result,
)
from graph_tools import GraphBuilder, dump_graph
from model_checking import Morty


def run_planarity_loop(
    morty: Morty,
    *,
    approach_name: str,
    max_iterations: int = 10_000,
    iteration_timeout_seconds: float | None = 60.0,
) -> BenchmarkResult:
    counter = BenchmarkCounter(approach_name)
    graph_builder = GraphBuilder()
    target_path = morty.get_target_path()

    for iteration in range(max_iterations):
        counter.attempted()
        no_counterexample = False
        try:
            with iteration_timeout(iteration_timeout_seconds):
                cex_file = morty.model_check(
                    f"cex_it_{iteration}.xml",
                    iteration,
                )
                if cex_file is None:
                    no_counterexample = True
                else:
                    graph = graph_builder.build_from_xml_cex(cex_file)
                    planar, _ = nx.check_planarity(graph)
                    prefix = "planar_graph" if planar else "not_planar_graph"
                    dump_graph(
                        graph,
                        str(target_path / f"{prefix}_it_{iteration}.html"),
                    )
                    morty.block_topology(graph, iteration)
        except IterationTimeoutError as error:
            counter.timed_out()
            print(f"{approach_name}: iteration {iteration} timed out: {error}")
        else:
            counter.succeeded()
        finally:
            morty.close_model_checker()
        if no_counterexample:
            break

    result = counter.result(completed=True)
    save_benchmark_result(result, target_path)
    return result
