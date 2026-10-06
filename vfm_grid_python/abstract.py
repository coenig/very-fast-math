"""Synthesize optional grid edges, N/P labels, parking goals and car routes."""
from __future__ import annotations

import argparse
import re
from pathlib import Path
from .common import both, either, grid_edges, dump, states, check, config_args, load_config


def validate(config):
    for key in ("rows", "columns", "cars"):
        if type(config[key]) is not int or config[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    count = config["rows"] * config["columns"]
    if config["cars"] >= count:
        raise ValueError("at least one empty node is required")
    if "parkings" in config and (type(config["parkings"]) is not int or
            not config["cars"] <= config["parkings"] < count):
        raise ValueError("parkings must be at least cars (distinct goals) and less than the node count")
    for key in ("starts", "goals"):
        values = config.get(key)
        if values is not None and (len(values) != config["cars"] or
                len(set(values)) != len(values) or
                any(type(v) is not int or not 0 <= v < count for v in values)):
            raise ValueError(f"{key} must give distinct valid node IDs, one per car")
    for node, kind in config.get("labels", {}).items():
        if not 0 <= int(node) < count or kind not in ("N", "P"):
            raise ValueError("labels map valid node IDs to N or P")
    if "start_types" in config and (len(config["start_types"]) != config["cars"] or
            any(kind not in ("N","P","any") for kind in config["start_types"])):
        raise ValueError("start_types must specify N, P or any for each car")
    for name, kind in config.get("roles", {}).items():
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name) or kind not in ("N", "P", "any"):
            raise ValueError("roles map lowercase names to N, P or any")
    candidates = set(grid_edges(config["rows"], config["columns"]))
    if "edges" in config and any(tuple(sorted(e)) not in candidates for e in config["edges"]):
        raise ValueError("fixed edges must join orthogonal grid neighbours")


def render(config):
    validate(config)
    from .stable import enabled, abstract_guards
    n, cars = config["rows"] * config["columns"], config["cars"]
    edges = grid_edges(config["rows"], config["columns"])
    lines = ["MODULE main", "FROZENVAR"]
    lines += [f"  edge_{u}_{v} : boolean;" for u, v in edges]
    lines += [f"  parking_{u} : boolean;" for u in range(n)]
    lines += [f"  goal_{i} : 0..{n-1};" for i in range(cars)]
    lines += [f"  start_{i} : 0..{n-1};" for i in range(cars)]
    lines += [f"  role_{name} : 0..{n-1};" for name in config.get("roles", {})]
    lines += ["VAR"] + [f"  loc_{i} : 0..{n-1};" for i in range(cars)]
    lines.append("DEFINE")
    lines += ["  parking_count := " + " + ".join(
        f"(case parking_{u} : 1; TRUE : 0; esac)" for u in range(n)) + ";"]
    def adjacent(left, right):
        return either([f"edge_{u}_{v} & {left} = {a} & {right} = {b}"
                       for u,v in edges for a,b in ((u,v),(v,u))])
    for u in range(n):
        incident = [(a, b) for a, b in edges if u in (a, b)]
        degree = " + ".join(f"(case edge_{a}_{b} : 1; TRUE : 0; esac)" for a, b in incident) or "0"
        lines += [f"  degree_{u} := {degree};"]
    for i in range(cars):
        lines += [f"  parked_{i} := " + either([f"loc_{i} = {u} & parking_{u}" for u in range(n)]) + ";"]
        lines += [f"  foreign_parked_{i} := parked_{i} & loc_{i} != goal_{i};",
                  f"  beside_start_{i} := {adjacent(f'start_{i}', f'loc_{i}')};"]
        columns = config["columns"]
        lines += [f"  distance_to_goal_{i} := abs((loc_{i} / {columns}) - (goal_{i} / {columns})) + abs((loc_{i} mod {columns}) - (goal_{i} mod {columns}));"]
        for j in range(i):
            lines += [f"  adjacent_{j}_{i} := {adjacent(f'loc_{j}', f'loc_{i}')};"]
    lines += ["  all_parked := " + both([f"loc_{i} = goal_{i}" for i in range(cars)]) + ";"]
    for name in config.get("roles", {}):
        lines += [f"DEFINE role_{name}_degree := case " + " ".join(
            f"role_{name} = {u} : degree_{u};" for u in range(n)) + " TRUE : 0; esac;"]
    for name, kind in config.get("roles", {}).items():
        if kind != "any":
            label = "" if kind == "P" else "!"
            lines += ["INIT " + either([f"role_{name} = {u} & {label}parking_{u}" for u in range(n)]) + ";"]
    if "parkings" in config:
        lines += [f"INVAR parking_count = {config['parkings']};"]
    for u in range(n):
        lines += [f"INVAR parking_{u} -> degree_{u} = 1;"]
    for u, v in edges:
        lines += [f"INVAR edge_{u}_{v} -> !(parking_{u} & parking_{v});"]
    for u, kind in config.get("labels", {}).items():
        lines += [f"INIT parking_{int(u)} = {'TRUE' if kind == 'P' else 'FALSE'};"]
    if "edges" in config:
        fixed = {tuple(sorted(edge)) for edge in config["edges"]}
        lines += [f"INIT edge_{u}_{v} = {'TRUE' if (u,v) in fixed else 'FALSE'};" for u, v in edges]
    for i in range(cars):
        lines += [f"INIT loc_{i} = start_{i};"]
        lines += ["INIT " + either([f"goal_{i} = {u} & parking_{u}" for u in range(n)]) + ";"]
        lines += ["INVAR " + either([f"loc_{i} = {u} & degree_{u} > 0" for u in range(n)]) + ";"]
        if "starts" in config:
            lines += [f"INIT loc_{i} = {config['starts'][i]};"]
        start_type=config.get("start_types",["N"]*cars)[i]
        if start_type!="any":
            label="" if start_type=="P" else "!"
            lines += ["INIT " + either([f"loc_{i} = {u} & {label}parking_{u}" for u in range(n)]) + ";"]
        if "goals" in config:
            lines += [f"INIT goal_{i} = {config['goals'][i]};"]
        moves = [f"next(loc_{i}) = loc_{i}"]
        for u, v in edges:
            moves += [f"edge_{u}_{v} & loc_{i} = {a} & next(loc_{i}) = {b}" for a, b in ((u,v),(v,u))]
        lines += ["TRANS " + either(moves) + ";"]
        for j in range(i):
            lines += [f"INVAR loc_{i} != loc_{j};", f"INIT goal_{i} != goal_{j};",
                      f"TRANS !(next(loc_{i}) = loc_{j} & next(loc_{j}) = loc_{i});"]
    lines += ["INIT " + expr + ";" for expr in config.get("initial_constraints", [])]
    lines += ["INVAR " + expr + ";" for expr in config.get("constraints", [])]
    if enabled(config): lines += [abstract_guards(config)]
    if config.get("scenario_stages"):
        if config.get("desired_ltl"):
            raise ValueError("choose either scenario_stages or desired_ltl")
        from .benchmark_scenarios import observer
        lines += [observer(config), "INVARSPEC !scenario_accepted;"]
    elif config.get("desired_ltl"):
        lines += [f"LTLSPEC !({config['desired_ltl']});"]
    else:
        lines += ["INVARSPEC !all_parked;"]
    return "\n".join(lines) + "\n"


def run(config, output, bound=100, timeout=1800, gif=True, visualize=True):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    model = output / "abstract.smv"
    if model.exists():
        raise FileExistsError(f"use a new output directory: {output}")
    from .benchmark_timing import interval
    with interval("generation", model=str(model)):
        model.write_text(render(config))
    dump(output / "config.json", config)
    result = check(model, bound, timeout, bool(config.get("desired_ltl")))
    if result["status"] != "witness":
        return result
    trace = states(output / "trace.xml")
    first = trace[0]
    graph = {"rows": config["rows"], "columns": config["columns"],
             "nodes": [{"id": u, "row": u // config["columns"], "column": u % config["columns"],
                        "kind": "P" if first[f"parking_{u}"] == "TRUE" else "N"}
                       for u in range(config["rows"] * config["columns"])],
             "edges": [list(e) for e in grid_edges(config["rows"], config["columns"])
                       if first[f"edge_{e[0]}_{e[1]}"] == "TRUE"],
             "goals": [int(first[f"goal_{i}"]) for i in range(config["cars"])],
             "starts": [int(first[f"start_{i}"]) for i in range(config["cars"])],
             "roles": {name: int(first[f"role_{name}"]) for name in config.get("roles", {})},
             "milestones": [[int(s[f"loc_{i}"]) for i in range(config["cars"])] for s in trace]}
    dump(output / "selected_graph.json", graph)
    if config.get('semantics') == 'stable_concurrent':
        from .stable import profile, anchors
        graph['semantics']='stable_concurrent'
        graph['holding_neighbours']=anchors(graph)
        dump(output/'contract.json',profile(config))
        dump(output/'selected_graph.json',graph)
    if config.get("scenario_stages"):
        graph["scenario_phases"] = [int(s["scenario_phase"]) for s in trace]
        dump(output / "selected_graph.json", graph)
    if visualize:
        from .visualize import abstract_plot
        abstract_plot(graph, output, gif)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    config_args(parser)
    args = parser.parse_args()
    result = run(load_config(args), args.output_dir, args.bound, args.timeout, not args.no_gif)
    return 0 if result["status"] == "witness" else 2


if __name__ == "__main__":
    raise SystemExit(main())
