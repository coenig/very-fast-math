#!/usr/bin/env python3
"""Shared scenario language for the three-way parking benchmarks."""

from __future__ import annotations

import re
import tomllib
from dataclasses import dataclass
from pathlib import Path

from generate_scenarios import LtlProperty, RoleDistance, RoleSpec, ScenarioSpec
from vfm_grid_parking import vehicle


ATOM = re.compile(r"^(\d+)\s*(@|!)\s*([a-z][a-z0-9_]*)$")


@dataclass(frozen=True)
class Atom:
    car: int
    role: str
    negated: bool = False

    def abstract(self) -> str:
        operator = "!=" if self.negated else "="
        return f"loc_{self.car} {operator} {self.role}"

    def concrete(self) -> str:
        operator = "!=" if self.negated else "="
        return f"{vehicle(self.car, 'on_straight_section')} {operator} {self.role}"


@dataclass(frozen=True)
class HistoryStage:
    event: tuple[Atom, ...]
    hold: tuple[Atom, ...] = ()


@dataclass(frozen=True)
class BenchmarkCase:
    spec: ScenarioSpec
    prop: LtlProperty
    stages: tuple[HistoryStage, ...]
    tier: str

    @property
    def identity(self) -> str:
        return self.spec.name


def _conjunction(atoms: tuple[Atom, ...], concrete: bool = False) -> str:
    if not atoms:
        return "TRUE"
    values = [atom.concrete() if concrete else atom.abstract() for atom in atoms]
    return "(" + " & ".join(values) + ")"


def stages_to_ltl(stages: tuple[HistoryStage, ...]) -> str:
    """Compile a finite ordered history to co-safe nested U/X LTL."""
    if not stages:
        raise ValueError("a scenario needs at least one history stage")
    last = stages[-1]
    expression = f"({_conjunction(last.hold)} U {_conjunction(last.event)})"
    for stage in reversed(stages[:-1]):
        expression = (
            f"({_conjunction(stage.hold)} U "
            f"({_conjunction(stage.event)} & X ({expression})))"
        )
    return expression


def stages_to_monitor(stages: tuple[HistoryStage, ...]) -> str:
    """Compile the history to a passive VFM observer with a dead state."""
    final = len(stages)
    dead = final + 1
    cases: list[str] = []
    for index, stage in enumerate(stages):
        cases.append(
            f"    parking_phase = {index} & {_conjunction(stage.event, True)} : {index + 1};"
        )
        if stage.hold:
            cases.append(
                f"    parking_phase = {index} & !{_conjunction(stage.hold, True)} : {dead};"
            )
    return "\n".join((
        "-- Passive observer compiled from the co-safe LTL history.",
        "VAR",
        f"  parking_phase : 0..{dead};",
        "ASSIGN",
        "  init(parking_phase) := 0;",
        "  next(parking_phase) := case",
        *cases,
        "    TRUE : parking_phase;",
        "  esac;",
        f"INVARSPEC parking_phase != {final};",
    ))


def _parse_atom(value: object, cars: int, roles: set[str], context: str) -> Atom:
    if not isinstance(value, str):
        raise ValueError(f"{context} must be a string such as '0@refuge'")
    match = ATOM.fullmatch(value.strip())
    if not match:
        raise ValueError(f"{context} must match CAR@ROLE or CAR!ROLE")
    car, operator, role = int(match.group(1)), match.group(2), match.group(3)
    if car >= cars:
        raise ValueError(f"{context} references car {car}, but n_cars={cars}")
    if role not in roles:
        raise ValueError(f"{context} references undeclared role {role}")
    return Atom(car, role, operator == "!")


def load_benchmark_cases(path: Path) -> tuple[BenchmarkCase, ...]:
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    defaults = document.get("defaults", {})
    raw_cases = document.get("scenario")
    if not isinstance(raw_cases, list) or not raw_cases:
        raise ValueError("TOML needs at least one [[scenario]]")
    result: list[BenchmarkCase] = []
    for index, raw in enumerate(raw_cases):
        context = f"scenario[{index}]"
        name = str(raw["name"])
        rows, columns = (int(value) for value in raw["grid_size"])
        cars = int(raw["n_cars"])
        roles = tuple(RoleSpec(
            name=str(item["name"]),
            exclude_initial=bool(item.get("exclude_initial", False)),
            initial_car=item.get("initial_car"),
            minimum_start_distance=int(item.get("minimum_start_distance", 0)),
            minimum_degree=int(item.get("minimum_degree", 0)),
        ) for item in raw["roles"])
        role_names = {role.name for role in roles}
        if len(role_names) != len(roles):
            raise ValueError(f"{context} contains duplicate roles")
        distances = tuple(RoleDistance(
            str(item["from"]), str(item["to"]), int(item["minimum"]),
        ) for item in raw.get("role_distances", []))
        stages: list[HistoryStage] = []
        for stage_index, item in enumerate(raw["stages"]):
            prefix = f"{context}.stages[{stage_index}]"
            event = tuple(_parse_atom(value, cars, role_names, prefix + ".event")
                          for value in item["event"])
            hold = tuple(_parse_atom(value, cars, role_names, prefix + ".hold")
                         for value in item.get("hold", []))
            if not event:
                raise ValueError(f"{prefix}.event cannot be empty")
            stages.append(HistoryStage(event, hold))
        stage_tuple = tuple(stages)
        formula = stages_to_ltl(stage_tuple)
        prop = LtlProperty(name, str(raw.get("explanation", "")), formula,
                           roles, distances)
        spec = ScenarioSpec(
            name, rows, columns, cars,
            int(raw.get("bound", defaults.get("bound", 120))),
            int(raw.get("timeout_seconds", defaults.get("timeout_seconds", 900))),
            "first_row", (prop,), int(raw["maximum_edges"]),
        )
        if rows < 2 or columns < 2 or cars < 2 or cars > columns:
            raise ValueError(f"{context} has an invalid grid/fleet combination")
        possible_edges = rows * (columns - 1) + (rows - 1) * columns
        if not 0 <= spec.maximum_edges <= possible_edges:
            raise ValueError(f"{context}.maximum_edges is out of range")
        result.append(BenchmarkCase(spec, prop, stage_tuple,
                                    str(raw.get("tier", "ordinary"))))
    identities = [case.identity for case in result]
    if len(identities) != len(set(identities)):
        raise ValueError("scenario names must be unique")
    return tuple(result)
