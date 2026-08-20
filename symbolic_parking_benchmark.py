"""Benchmark structured symbolic parking searches and optionally render GIFs."""

from __future__ import annotations

import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from symbolic_parking import (
    ParkingSearchConfig,
    check_symbolic_parking_model,
    generate_symbolic_parking_model,
    reconstruct_selected_topology,
    save_counterexample_gif,
    save_selected_topology,
)


def benchmark_case(
    cars: int, output_dir: Path, bound: int, *, render_gif: bool = False
) -> dict:
    spots = cars
    aisle_sections = spots + 2
    aisle_length, spot_length, connection_length = max(30, cars * 10), 20, 10
    positions = tuple(aisle_length - 5 - 10 * index for index in range(cars))
    assignments = tuple(reversed(range(spots)))
    # Large enough for the farthest assignment while keeping finite domains small.
    budgets = tuple(
        aisle_length * (spot + 1) + connection_length * (spot + 2) + spot_length
        for spot in assignments
    )
    config = ParkingSearchConfig(
        aisle_sections, spots, positions, assignments, budgets,
        aisle_length=aisle_length, spot_length=spot_length,
        connection_length=connection_length, bmc_bound=bound,
    )
    result = {"cars": cars, "spots": spots, "aisle_sections": aisle_sections}
    started = time.perf_counter()
    try:
        generated_at = time.perf_counter()
        model = generate_symbolic_parking_model(config, experiments_dir=output_dir)
        result["generation_seconds"] = time.perf_counter() - generated_at
        result["smv_lines"] = sum(1 for _ in model.model_file.open())
        checked_at = time.perf_counter()
        cex = check_symbolic_parking_model(model)
        result["checking_seconds"] = time.perf_counter() - checked_at
        result["scenario_directory"] = str(model.target_path)
        result["witness_found"] = cex is not None
        if cex:
            result["counterexample_states"] = len(
                ET.parse(cex).getroot().findall("node")
            )
            graph = reconstruct_selected_topology(cex, config)
            result["attachments"] = {
                node: data["attachment"]
                for node, data in graph.nodes(data=True)
                if data["kind"] == "parking"
            }
            graphml, topology_json = save_selected_topology(graph, model.target_path)
            result["topology_graphml"] = str(graphml)
            result["topology_json"] = str(topology_json)
            if render_gif:
                gif = model.target_path / "symbolic_parking.gif"
                save_counterexample_gif(cex, graph, config, gif)
                result["gif"] = str(gif)
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
    result["elapsed_seconds"] = time.perf_counter() - started
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cars", nargs="+", type=int, default=[2, 3, 4])
    parser.add_argument("--bound", type=int, default=50)
    parser.add_argument("--output-dir", type=Path, default=Path("SMV_GEN"))
    parser.add_argument("--gif", action="store_true",
                        help="render a GIF for every counterexample found")
    args = parser.parse_args()
    if any(cars < 2 for cars in args.cars):
        parser.error("benchmark cases require at least two cars")
    results = []
    args.output_dir.mkdir(parents=True, exist_ok=True)
    progress = args.output_dir / "symbolic_parking_benchmark_progress.json"
    for cars in args.cars:
        result = benchmark_case(
            cars, args.output_dir, args.bound, render_gif=args.gif
        )
        results.append(result)
        print(json.dumps(result, indent=2))
        progress.write_text(json.dumps(results, indent=2) + "\n")
    output = args.output_dir / f"symbolic_parking_benchmark_{time.time_ns()}.json"
    output.write_text(json.dumps(results, indent=2) + "\n")
    print(f"Benchmark results: {output}")
    return 1 if any("error" in result for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
