from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import networkx as nx

from .model import problem_from_multidigraph, write_solution
from .solver import solve_multidigraph


def example_graph() -> nx.MultiDiGraph:
    graph = nx.MultiDiGraph()
    graph.add_edge("a", "b", length=3.0)
    graph.add_edge("b", "c", length=4.0)
    graph.add_edge("c", "a", length=5.0)
    return graph


def load_node_link_multidigraph(path: str | Path) -> nx.MultiDiGraph:
    data = json.loads(Path(path).read_text())
    graph = nx.node_link_graph(data, directed=True, multigraph=True)
    if not isinstance(graph, nx.MultiDiGraph):
        graph = nx.MultiDiGraph(graph)
    return graph


def load_input_multidigraph(
    path: str | Path,
    *,
    input_format: str = "auto",
) -> nx.MultiDiGraph:
    input_path = Path(path)
    if input_format == "auto":
        input_format = "cex" if input_path.suffix.lower() == ".xml" else "node-link"

    if input_format == "cex":
        from .cex import build_multidigraph_from_xml_cex

        return build_multidigraph_from_xml_cex(input_path)
    if input_format == "node-link":
        return load_node_link_multidigraph(input_path)
    raise ValueError(f"unknown input format {input_format!r}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Solve fixed-length constraints from a NetworkX MultiDiGraph"
    )
    parser.add_argument(
        "input",
        nargs="?",
        help="optional CEX XML or NetworkX node-link JSON file; omit to solve the built-in example",
    )
    parser.add_argument(
        "--input-format",
        choices=("auto", "cex", "node-link"),
        default="auto",
    )
    parser.add_argument("-o", "--output", default="solution.json")
    parser.add_argument("--length-attr", default="length")
    parser.add_argument("--method", choices=("scipy", "z3"), default="scipy")
    parser.add_argument("--attempts", type=int, default=100, help="SciPy restarts")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=30.0, help="Z3 timeout in seconds")
    parser.add_argument("--tolerance", type=float, default=1e-6)
    parser.add_argument("--plot", help="optional PNG output path")
    args = parser.parse_args()

    try:
        graph = (
            load_input_multidigraph(args.input, input_format=args.input_format)
            if args.input
            else example_graph()
        )
        problem = problem_from_multidigraph(graph, length_attr=args.length_attr)
        solved_graph, report = solve_multidigraph(
            graph,
            length_attr=args.length_attr,
            method=args.method,
            attempts=args.attempts,
            seed=args.seed,
            timeout=args.timeout,
            tolerance=args.tolerance,
        )
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        print(f"input error: {error}", file=sys.stderr)
        return 2
    except ImportError as error:
        print(f"solver dependency error: {error}", file=sys.stderr)
        return 2

    if solved_graph is not None:
        coordinates = {
            str(node): solved_graph.nodes[node]["pos"] for node in solved_graph.nodes
        }
        write_solution(args.output, problem, coordinates, args.method, report)
        if args.plot:
            from .plotting import plot_solution

            plot_solution(problem, args.output, args.plot)

    print(json.dumps(report, indent=2))
    return 0 if report.get("valid") else 1


if __name__ == "__main__":
    raise SystemExit(main())
