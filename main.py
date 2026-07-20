from __future__ import annotations

import argparse
import csv
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

from approaches import APPROACHES
from scenario_generation import (
    PROPERTY_SPECS,
    generate_scenario,
    iter_model_configs,
)


def write_summary_csv(results: list[dict], output_file: Path) -> None:
    approaches = sorted({result["approach"] for result in results})
    grouping_keys = [
        "property",
        "sections",
        "nonegos",
        "numlanes",
        "geometry",
        "max_outgoing_connections",
    ]
    groups: dict[tuple, dict[str, dict]] = {}
    for result in results:
        key = tuple(result.get(field, "") for field in grouping_keys)
        groups.setdefault(key, {})[result["approach"]] = result

    fieldnames = [*grouping_keys]
    for approach in approaches:
        fieldnames.extend(
            [
                f"{approach}_elapsed_seconds",
                f"{approach}_attempted_runs",
                f"{approach}_successful_runs",
                f"{approach}_timed_out_runs",
                f"{approach}_completed",
                f"{approach}_scenario_directory",
                f"{approach}_error",
            ]
        )

    with output_file.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for key, by_approach in sorted(groups.items()):
            row = dict(zip(grouping_keys, key))
            for approach in approaches:
                result = by_approach.get(approach, {})
                row.update(
                    {
                        f"{approach}_elapsed_seconds": result.get(
                            "elapsed_seconds", ""
                        ),
                        f"{approach}_attempted_runs": result.get(
                            "attempted_runs", ""
                        ),
                        f"{approach}_successful_runs": result.get(
                            "successful_runs", ""
                        ),
                        f"{approach}_timed_out_runs": result.get(
                            "timed_out_runs", ""
                        ),
                        f"{approach}_completed": result.get("completed", ""),
                        f"{approach}_scenario_directory": result.get(
                            "scenario_directory", ""
                        ),
                        f"{approach}_error": result.get("error", ""),
                    }
                )
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an SMV scenario and run a graph-search approach."
    )
    parser.add_argument(
        "--approach",
        nargs="+",
        choices=["all", *(str(number) for number in sorted(APPROACHES))],
        default=["all"],
        help="approaches to execute in parallel (default: all)",
    )
    parser.add_argument("--nonegos", type=int, default=2)
    parser.add_argument(
        "--nonegos-values",
        nargs="+",
        type=int,
        help="one or more non-ego vehicle counts to benchmark",
    )
    parser.add_argument("--sections", type=int, default=10)
    parser.add_argument(
        "--sections-values",
        nargs="+",
        type=int,
        help="one or more section/edge counts to benchmark",
    )
    parser.add_argument("--lanes", type=int, default=1)
    parser.add_argument("--section-max-length", type=int, default=1000)
    parser.add_argument("--section-min-length", type=int, default=100)
    parser.add_argument(
        "--max-outgoing-connections",
        type=int,
        default=3,
        help="maximum E_i -> S_j edges from each section end (default: 3)",
    )
    parser.add_argument("--geometry", type=int, choices=(0, 1), default=0)
    parser.add_argument(
        "--property",
        nargs="+",
        choices=["all", *sorted(PROPERTY_SPECS)],
        default=["vehicle_order"],
        help="property or properties to benchmark (default: vehicle_order)",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=10_000,
        help="maximum runs per approach (default: 10000)",
    )
    parser.add_argument(
        "--iteration-timeout",
        type=float,
        default=60.0,
        help="timeout in seconds for each run (default: 60)",
    )
    parser.add_argument("--config-template", default="main_template.json")
    parser.add_argument("--output-dir", default="SMV_GEN")
    args = parser.parse_args()
    if args.max_iterations < 0:
        parser.error("--max-iterations must be non-negative")
    if args.iteration_timeout <= 0:
        parser.error("--iteration-timeout must be positive")
    if args.max_outgoing_connections < 0:
        parser.error("--max-outgoing-connections must be non-negative")
    if "all" in args.approach and len(args.approach) != 1:
        parser.error("'all' cannot be combined with individual approaches")
    if "all" in args.property and len(args.property) != 1:
        parser.error("'all' cannot be combined with individual properties")
    return args


def run_approach_worker(
    approach_number: int,
    config: dict,
    metadata: dict,
    *,
    input_config: str,
    output_dir: str,
    max_iterations: int,
    iteration_timeout: float,
    max_outgoing_connections: int,
) -> dict:
    morty = generate_scenario(
        config,
        input_config=input_config,
        experiments_dir=output_dir,
        scenario_suffix=f"approach_{approach_number}",
    )
    approach_kwargs = {
        "max_iterations": max_iterations,
        "iteration_timeout_seconds": iteration_timeout,
    }
    if approach_number == 3:
        approach_kwargs["max_outgoing_connections"] = max_outgoing_connections
    result = APPROACHES[approach_number](morty, **approach_kwargs)
    output = result.as_dict()
    output["scenario_directory"] = str(morty.get_target_path())
    output.update(metadata)
    return output


def main() -> int:
    args = parse_args()
    selected_approaches = (
        sorted(APPROACHES)
        if args.approach == ["all"]
        else sorted({int(number) for number in args.approach})
    )
    selected_properties = (
        sorted(PROPERTY_SPECS)
        if args.property == ["all"]
        else list(dict.fromkeys(args.property))
    )
    parameters = {
        "nonegos": args.nonegos_values or [args.nonegos],
        "sections": args.sections_values or [args.sections],
        "numlanes": [args.lanes],
        "section_max_length": [args.section_max_length],
        "section_min_length": [args.section_min_length],
        "max_outgoing_connections": [args.max_outgoing_connections],
        "geometry": [args.geometry],
        "property": selected_properties,
    }
    configs = list(iter_model_configs(parameters, include_metadata=True))
    jobs = [
        (config_index, approach_number, config, metadata)
        for config_index, (config, metadata) in enumerate(configs)
        for approach_number in selected_approaches
    ]
    results = []
    failed = False
    with ProcessPoolExecutor(max_workers=len(selected_approaches)) as executor:
        future_jobs = {
            executor.submit(
                run_approach_worker,
                approach_number,
                config,
                metadata,
                input_config=args.config_template,
                output_dir=args.output_dir,
                max_iterations=args.max_iterations,
                iteration_timeout=args.iteration_timeout,
                max_outgoing_connections=args.max_outgoing_connections,
            ): (config_index, approach_number)
            for config_index, approach_number, config, metadata in jobs
        }
        for future in as_completed(future_jobs):
            config_index, approach_number = future_jobs[future]
            try:
                result = future.result()
            except Exception as error:
                failed = True
                result = {
                    "approach": f"approach_{approach_number}",
                    "config_index": config_index,
                    "error": str(error),
                }
                _, _, _, metadata = next(
                    job for job in jobs
                    if job[0] == config_index and job[1] == approach_number
                )
                result.update(metadata)
            else:
                result["config_index"] = config_index
            results.append(result)
            print(json.dumps(result, indent=2))

    results.sort(
        key=lambda result: (
            result.get("property", ""),
            result.get("sections", 0),
            result.get("nonegos", 0),
            result["config_index"],
            result["approach"],
        )
    )
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    aggregate_file = output_path / f"benchmark_results_{time.time_ns()}.json"
    aggregate_file.write_text(json.dumps(results, indent=2) + "\n")
    summary_file = aggregate_file.with_suffix(".summary.csv")
    write_summary_csv(results, summary_file)
    print(f"Aggregate benchmark results written to {aggregate_file}")
    print(f"Summary benchmark table written to {summary_file}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
