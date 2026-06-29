from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
import time
from itertools import product
from pathlib import Path
from typing import Callable

from model_checking import Morty


def identity(value):
    return value


def get_parameters_test_enumeration() -> dict:
    return {
        "nonegos": list(range(3, 6)),
        "sections": list(range(3, 7)),
        "numlanes": [1],
        "section_max_length": [1000],
        "section_min_length": [100],
        "max_outgoing_connections": [3],
        "geometry": [1],
    }


def get_parameters_simplest_geometries() -> dict:
    return {
        "nonegos": [5],
        "sections": [3],
        "numlanes": [1],
        "section_max_length": [1000],
        "section_min_length": [100],
        "max_outgoing_connections": [3],
        "geometry": [0],
    }


def get_param_simple(
    nonegos: int,
    sections: int,
    lanes: int,
    max_length: int = 1000,
    min_length: int = 100,
    max_outgoing_connections: int = 3,
    geometry: int = 1,
) -> dict:
    return {
        "nonegos": [nonegos],
        "sections": [sections],
        "numlanes": [lanes],
        "section_max_length": [max_length],
        "section_min_length": [min_length],
        "max_outgoing_connections": [max_outgoing_connections],
        "geometry": [geometry],
    }


def parametrize_model_config(
    input_config: str | Path,
    output_config: str | Path,
    values: dict,
) -> None:
    with Path(input_config).open() as source:
        config = json.load(source)
    config["_config"].update(values)
    Path(output_config).write_text(json.dumps(config, indent=3))


def iter_model_configs(parameters: dict):
    replacements: dict[str, list[tuple[str, Callable]]] = {
        "nonegos": [("#007", identity), ("NONEGOS", identity)],
        "numlanes": [("#008", identity), ("NUMLANES", identity)],
        "sections": [("#010", identity), ("SECTIONS", identity)],
        "section_max_length": [("SECTIONSMAXLENGTH", identity)],
        "section_min_length": [("SECTIONSMINLENGTH", identity)],
        "max_outgoing_connections": [("MAXOUTGOINGCONNECTIONS", identity)],
        "geometry": [("MODEL_INTERSECTION_GEOMETRY", identity)],
    }
    
    # Update the property from the selected number of non-ego vehicles.
    def some_stuff(nonegos):
        clauses = []
        for i in range(nonegos):
            clauses.append(f"env.veh___6{i}9___.on_straight_section >= 0")
        for i, j in zip(range(nonegos - 1), range(1, nonegos)):
            clauses.append(
                f"env.veh___6{i}9___.abs_pos > env.veh___6{j}9___.abs_pos"
            )
        return f"INVARSPEC !({' & '.join(clauses)});"
    replacements["nonegos"].append(("SPEC", some_stuff))

    for combination in product(*parameters.values()):
        selected = dict(zip(parameters.keys(), combination))
        yield {
            output_key: transform(selected[input_key])
            for input_key, output_values in replacements.items()
            for output_key, transform in output_values
        }


def generate_scenario(
    config: dict,
    *,
    input_config: str | Path = "main_template.json",
    experiments_dir: str | Path = "SMV_GEN",
    scenario_suffix: str | None = None,
) -> Morty:
    experiments_path = Path(experiments_dir)
    experiments_path.mkdir(parents=True, exist_ok=True)
    output_config = Path(
        f"/tmp/envmodel_config_{os.getpid()}_{time.time_ns()}.json"
    )
    cache_path = Path(tempfile.mkdtemp(prefix="last_demo_vfm_cache_"))
    parametrize_model_config(input_config, output_config, config)

    suffix = ""
    if scenario_suffix:
        normalized_suffix = re.sub(r"[^A-Za-z0-9_-]+", "_", scenario_suffix)
        suffix = f"_{normalized_suffix.strip('_')}"
    target_path = experiments_path / f"scenario{time.time_ns()}{suffix}"
    args = [
        str(output_config),
        "",
        ".",
        "src/templates/EnvModel.tpl",
        "src/examples/ego_less/vfm-includes.txt",
        str(target_path),
        str(cache_path),
        "src/templates",
    ]
    morty = Morty()
    try:
        morty.generate_smv_files(args)
    finally:
        output_config.unlink(missing_ok=True)
        shutil.rmtree(cache_path, ignore_errors=True)
    return morty
