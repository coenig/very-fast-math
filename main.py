from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

from approaches import APPROACHES
from scenario_generation import (
    generate_scenario,
    get_param_simple,
    iter_model_configs,
)


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
    parser.add_argument("--sections", type=int, default=10)
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
    return args


def run_approach_worker(
    approach_number: int,
    config: dict,
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
    return output


def main() -> int:
    args = parse_args()
    selected_approaches = (
        sorted(APPROACHES)
        if args.approach == ["all"]
        else sorted({int(number) for number in args.approach})
    )
    parameters = get_param_simple(
        nonegos=args.nonegos,
        sections=args.sections,
        lanes=args.lanes,
        max_length=args.section_max_length,
        min_length=args.section_min_length,
        max_outgoing_connections=args.max_outgoing_connections,
        geometry=args.geometry,
    )
    configs = list(iter_model_configs(parameters))
    jobs = [
        (config_index, approach_number, config)
        for config_index, config in enumerate(configs)
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
                input_config=args.config_template,
                output_dir=args.output_dir,
                max_iterations=args.max_iterations,
                iteration_timeout=args.iteration_timeout,
                max_outgoing_connections=args.max_outgoing_connections,
            ): (config_index, approach_number)
            for config_index, approach_number, config in jobs
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
            else:
                result["config_index"] = config_index
            results.append(result)
            print(json.dumps(result, indent=2))

    results.sort(key=lambda result: (result["config_index"], result["approach"]))
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    aggregate_file = output_path / f"benchmark_results_{time.time_ns()}.json"
    aggregate_file.write_text(json.dumps(results, indent=2) + "\n")
    print(f"Aggregate benchmark results written to {aggregate_file}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
