from __future__ import annotations

import networkx as nx

from benchmarking import (
    BenchmarkCounter,
    BenchmarkResult,
    IterationTimeoutError,
    iteration_timeout,
    save_benchmark_result,
)
from graph_constraints.model import problem_from_multidigraph
from graph_constraints.solver import solve_multidigraph
from graph_tools import GraphBuilder, geometric_length_graph
from model_checking import Morty


def run(
    morty: Morty,
    *,
    max_iterations: int = 10_000,
    iteration_timeout_seconds: float | None = 60.0,
    tolerance: float = 0.1,
) -> BenchmarkResult:
    """Search nuXmv counterexamples for a realizable fixed-length layout.

    Each iteration obtains a counterexample, extracts the section and
    connection lengths into a ``MultiDiGraph``, rejects non-planar topologies,
    and runs the SciPy fixed-length graph solver. Unusable topologies are
    blocked before continuing. The approach stops early when the solver finds
    a valid layout and writes its PNG visualization into the scenario
    directory.

    The timeout covers model checking, XML parsing, solving, and plotting for
    one iteration. The returned value uses the common benchmark result format.
    """
    counter = BenchmarkCounter("approach_1")
    graph_builder = GraphBuilder()
    target_path = morty.get_target_path()

    for iteration in range(max_iterations):
        counter.attempted()
        solution_found = False
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
                    geometry_graph = geometric_length_graph(graph)
                    planar, _ = nx.check_planarity(nx.Graph(geometry_graph))
                    if not planar:
                        morty.block_topology(graph, iteration)
                    else:
                        solved_graph, report = solve_multidigraph(
                            geometry_graph,
                            tolerance=tolerance,
                        )
                        if solved_graph is not None:
                            from graph_constraints.plotting import plot_coordinates

                            problem = problem_from_multidigraph(geometry_graph)
                            coordinates = {
                                str(node): solved_graph.nodes[node]["pos"]
                                for node in solved_graph.nodes
                            }
                            graph_plot = (
                                target_path
                                / f"graph_metric_layout_it_{iteration}.png"
                            )
                            plot_coordinates(
                                problem,
                                coordinates,
                                graph_plot,
                                report=report,
                                tolerance=tolerance,
                                title=f"Counterexample graph {iteration}",
                            )
                            print(f"Metric graph layout written to {graph_plot}")
                            solution_found = True
                        else:
                            morty.block_topology(graph, iteration)
        except IterationTimeoutError as error:
            counter.timed_out()
            print(f"approach_1: iteration {iteration} timed out: {error}")
        else:
            counter.succeeded()
        finally:
            morty.close_model_checker()

        if solution_found or no_counterexample:
            result = counter.result(completed=True)
            save_benchmark_result(result, target_path)
            return result

    result = counter.result(completed=False)
    save_benchmark_result(result, target_path)
    return result


approach_1 = run
