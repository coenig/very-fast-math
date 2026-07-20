from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
import time
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Callable

from model_checking import Morty


def identity(value):
    return value


def vehicle_var(index: int, field: str) -> str:
    return f"env.veh___6{index}9___.{field}"


def invar_not(expression: str) -> str:
    return f"INVARSPEC !({expression});"


def and_join(clauses: list[str]) -> str:
    return " & ".join(f"({clause})" for clause in clauses)


def or_join(clauses: list[str]) -> str:
    if not clauses:
        return "FALSE"
    return " | ".join(f"({clause})" for clause in clauses)


@dataclass(frozen=True)
class PropertySpec:
    name: str
    description: str
    builder: Callable[[int], str]


def property_vehicle_order(nonegos: int) -> str:
    """Cars are on one section and appear in strict descending absolute-position order."""
    clauses = [f"{vehicle_var(0, 'on_straight_section')} >= 0"]
    for index in range(1, nonegos):
        clauses.append(
            f"{vehicle_var(index - 1, 'on_straight_section')} = "
            f"{vehicle_var(index, 'on_straight_section')}"
        )
    for index in range(1, nonegos):
        clauses.append(
            f"{vehicle_var(index - 1, 'abs_pos')} > "
            f"{vehicle_var(index, 'abs_pos')}"
        )
    return invar_not(and_join(clauses))


def property_same_position_pair(nonegos: int) -> str:
    """Some pair of cars occupies the same section and the same longitudinal position."""
    pair_clauses = []
    for left in range(nonegos):
        for right in range(left + 1, nonegos):
            pair_clauses.append(
                and_join(
                    [
                        f"{vehicle_var(left, 'on_straight_section')} >= 0",
                        f"{vehicle_var(left, 'on_straight_section')} = "
                        f"{vehicle_var(right, 'on_straight_section')}",
                        f"{vehicle_var(left, 'abs_pos')} = "
                        f"{vehicle_var(right, 'abs_pos')}",
                    ]
                )
            )
    return invar_not(or_join(pair_clauses))


def property_all_stopped(nonegos: int) -> str:
    """All cars are stopped at the same time."""
    return invar_not(
        and_join([f"{vehicle_var(index, 'v')} = 0" for index in range(nonegos)])
    )


def property_all_target_speed(nonegos: int) -> str:
    """All cars reach the template's nominal target velocity at the same time."""
    target_velocity = 50
    return invar_not(
        and_join(
            [
                f"{vehicle_var(index, 'v')} = {target_velocity}"
                for index in range(nonegos)
            ]
        )
    )


def property_close_platoon(nonegos: int) -> str:
    """Cars are on one section, ordered, and consecutive cars are within 50 model units."""
    clauses = [f"{vehicle_var(0, 'on_straight_section')} >= 0"]
    for index in range(1, nonegos):
        clauses.extend(
            [
                f"{vehicle_var(index - 1, 'on_straight_section')} = "
                f"{vehicle_var(index, 'on_straight_section')}",
                f"{vehicle_var(index - 1, 'abs_pos')} <= "
                f"{vehicle_var(index, 'abs_pos')}",
                f"{vehicle_var(index - 1, 'abs_pos')} >= "
                f"{vehicle_var(index, 'abs_pos')} - 50",
            ]
        )
    return invar_not(and_join(clauses))


PROPERTY_SPECS: dict[str, PropertySpec] = {
    "vehicle_order": PropertySpec(
        "vehicle_order",
        "same-section cars in strict descending absolute-position order",
        property_vehicle_order,
    ),
    "same_position_pair": PropertySpec(
        "same_position_pair",
        "two cars on the same section at exactly the same absolute position",
        property_same_position_pair,
    ),
    "all_stopped": PropertySpec(
        "all_stopped",
        "all cars stopped simultaneously",
        property_all_stopped,
    ),
    "all_target_speed": PropertySpec(
        "all_target_speed",
        "all cars at velocity 50 simultaneously",
        property_all_target_speed,
    ),
    "close_platoon": PropertySpec(
        "close_platoon",
        "same-section ordered platoon with consecutive cars within 50 model units",
        property_close_platoon,
    ),
}


def build_property_spec(name: str, nonegos: int) -> str:
    try:
        property_spec = PROPERTY_SPECS[name]
    except KeyError as error:
        valid = ", ".join(sorted(PROPERTY_SPECS))
        raise ValueError(f"unknown property {name!r}; valid properties: {valid}") from error
    return property_spec.builder(nonegos)


def get_parameters_test_enumeration() -> dict:
    return {
        "nonegos": list(range(3, 6)),
        "sections": list(range(3, 7)),
        "numlanes": [1],
        "section_max_length": [1000],
        "section_min_length": [100],
        "max_outgoing_connections": [3],
        "geometry": [1],
        "property": ["vehicle_order"],
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
        "property": ["vehicle_order"],
    }


def get_param_simple(
    nonegos: int,
    sections: int,
    lanes: int,
    max_length: int = 1000,
    min_length: int = 100,
    max_outgoing_connections: int = 3,
    geometry: int = 1,
    property_name: str = "vehicle_order",
) -> dict:
    return {
        "nonegos": [nonegos],
        "sections": [sections],
        "numlanes": [lanes],
        "section_max_length": [max_length],
        "section_min_length": [min_length],
        "max_outgoing_connections": [max_outgoing_connections],
        "geometry": [geometry],
        "property": [property_name],
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


def iter_model_configs(parameters: dict, *, include_metadata: bool = False):
    replacements: dict[str, list[tuple[str, Callable]]] = {
        "nonegos": [("#007", identity), ("NONEGOS", identity)],
        "numlanes": [("#008", identity), ("NUMLANES", identity)],
        "sections": [("#010", identity), ("SECTIONS", identity)],
        "section_max_length": [("SECTIONSMAXLENGTH", identity)],
        "section_min_length": [("SECTIONSMINLENGTH", identity)],
        "max_outgoing_connections": [("MAXOUTGOINGCONNECTIONS", identity)],
        "geometry": [("MODEL_INTERSECTION_GEOMETRY", identity)],
    }

    for combination in product(*parameters.values()):
        selected = dict(zip(parameters.keys(), combination))
        property_name = selected.get("property", "vehicle_order")
        config = {
            output_key: transform(selected[input_key])
            for input_key, output_values in replacements.items()
            for output_key, transform in output_values
        }
        config["SPEC"] = build_property_spec(property_name, selected["nonegos"])
        if include_metadata:
            yield config, {
                "property": property_name,
                "property_description": PROPERTY_SPECS[property_name].description,
                "nonegos": selected["nonegos"],
                "sections": selected["sections"],
                "numlanes": selected["numlanes"],
                "max_outgoing_connections": selected["max_outgoing_connections"],
                "geometry": selected["geometry"],
            }
        else:
            yield config


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
