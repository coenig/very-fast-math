"""Refine a selected abstract witness using VFM and a milestone monitor."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

from .common import ROOT, both, either, dump, execute, check
from .geometry import build, location_expression
from .endpoint import movement, collision_constraints


def monitor(graph, geometry):
    from .stable import enabled, observation, support
    stable=enabled(graph)
    observe=(lambda g,c,n,stopped=False: observation(g,c,n,graph)) if stable else location_expression
    milestones = graph["milestones"]
    cars = len(milestones[0])
    if not milestones or any(len(m) != cars for m in milestones):
        raise ValueError("invalid milestones")
    selected_edges = {frozenset(e) for e in graph["edges"]}
    for left, right in zip(milestones, milestones[1:]):
        if len(set(right)) != cars or any(a != b and frozenset((a,b)) not in selected_edges for a,b in zip(left,right)):
            raise ValueError("abstract trace contains an invalid move")
        if any(left[i] == right[j] and left[j] == right[i] for i in range(cars) for j in range(i)):
            raise ValueError("abstract trace exchanges adjacent cars")
    lines = [f"VAR refinement_phase : 0..{len(milestones)-1};", "ASSIGN init(refinement_phase) := 0;",
             "next(refinement_phase) := case"]
    for index, milestone in enumerate(milestones[1:]):
        obs = both([observe(geometry, car, node) for car,node in enumerate(milestone)])
        lines += [f"  refinement_phase = {index} & next({obs}) : {index+1};"]
    lines += ["  TRUE : refinement_phase; esac;", f"INVARSPEC refinement_phase != {len(milestones)-1};"]
    for car, node in enumerate(milestones[0]):
        lines += [f"INIT {observe(geometry, car, node, stopped=True)};"]
    for index, milestone in enumerate(milestones):
        target = milestones[min(index+1,len(milestones)-1)]
        for car, (u,v) in enumerate(zip(milestone,target)):
            if stable:
                lines += [f"INVAR refinement_phase = {index} -> {support(geometry,car,u,v,graph)};"]
                continue
            terms = [location_expression(geometry,car,u), location_expression(geometry,car,v)]
            if u != v:
                section = next(s for s in geometry["sections"] if set(s["edge"]) == {u,v})
                terms += [f"env.veh___6{car}9___.on_straight_section = {section['id']}"]
            lines += [f"INVAR refinement_phase = {index} -> {either(terms)};"]
    lines += [collision_constraints(geometry,cars)]
    return "\n".join(lines) + "\n"


def _worker(output):
    # Native VFM writes directly to stdout; this entire worker runs with redirected FDs.
    from scenario_generation import generate_scenario
    from fixed_graph_model import _replace_singleton_domain
    from vfm_grid_parking import _geometry_metadata
    job = json.loads((output / "generation_job.json").read_text())
    geometry, graph = job["geometry"], job["graph"]
    cars, sections = len(graph["milestones"][0]), geometry["sections"]
    outgoing = {s["id"]: [] for s in sections}
    for c in geometry["connections"]:
        outgoing[c["from"]].append(c["to"])
    slots = max(1,max(map(len,outgoing.values())))
    config = {
        "#007": cars, "NONEGOS": cars, "#008": 1, "NUMLANES": 1,
        "#010": len(sections), "SECTIONS": len(sections), "SEGMENTS": 1,
        "TIMESCALING": 1000, "DISTANCESCALING": 1000,
        "SECTIONSMINLENGTH": min(s["length"] for s in sections),
        "SECTIONSMAXLENGTH": max(s["length"] for s in sections),
        "SEGMENTSMINLENGTH": 1, "MINDISTCONNECTIONS": 1,
        "MAXDISTCONNECTIONS": max([c["length"] for c in geometry["connections"]] or [1]),
        "MAXOUTGOINGCONNECTIONS": slots, "FIXED_TOPOLOGY": True,
        "MODEL_INTERSECTION_GEOMETRY": 0, "ALLOW_ZEROLENGTH_SECTIONS": False,
        "ANGLE_BASED_LC": False, "FEASIBILITY": False,
        "ENDPOINT_CONNECTIONS": True, "ENDPOINT_TRANSITIONS": movement(geometry,cars),
        "SPEC": "INVARSPEC FALSE;", "SCENGEN_MODE": False,
    }
    if job.get('symbolic_endpoint'):
        from .benchmark_symbolic import symbolic_movement
        config['ENDPOINT_TRANSITIONS'] = symbolic_movement(geometry,cars)
    if job.get('semantics')=='stable_concurrent' or graph.get('semantics')=='stable_concurrent':
        config.update(VEHICLE_LENGTH=5,VEHICLE_WIDTH=185,MINACCELNONEGO=-8,MAXACCELNONEGO=3)
    def array(values):
        return "".join(f"@({value})@" for value in values)
    config.update({
        "FIXED_SECTION_IDs": array(s["id"] for s in sections),
        "FIXED_SECTION_LENGTHs": array(s["length"] for s in sections),
        "FIXED_SECTION_SOURCE_Xs": array(s["source_x"] for s in sections),
        "FIXED_SECTION_SOURCE_Ys": array(s["source_y"] for s in sections),
        "FIXED_SECTION_ANGLEs": array(s["angle"] for s in sections),
        "FIXED_SECTION_CONNECTORS": "".join(
            f"@(@({s['id']})@@({array(outgoing[s['id']] + [-1]*(slots-len(outgoing[s['id']])) )})@)@"
            for s in sections),
    })
    protected = set(config)
    overrides = job.get("vfm", {})
    if protected.intersection(overrides):
        raise ValueError(f"structural VFM parameters cannot be overridden: {sorted(protected.intersection(overrides))}")
    config.update(overrides)
    dump(output / "vfm_parameters.json", config)
    morty = generate_scenario(config, input_config=job["template_config"],
                             experiments_dir=output / "vfm", scenario_suffix="endpoint_grid")
    target = morty.get_target_path()
    text = (target / "EnvModel.smv").read_text()
    if "#Error" in text or "#ERROR" in text:
        raise RuntimeError("VFM reported a template error; inspect generation.log")
    if "endpoint_step_v" not in text:
        raise RuntimeError("VFM did not expand the endpoint template")
    for s in sections:
        text = _replace_singleton_domain(text, f"section_{s['id']}_end", s["length"])
    lengths = {(c["from"],c["to"]):c["length"] for c in geometry["connections"]}
    for s in sections:
        for t in sections:
            if s == t:
                continue
            variable = f"arclength_from_sec_{s['id']}_to_sec_{t['id']}_on_lane_0"
            text = _replace_singleton_domain(text, variable, lengths.get((s['id'],t['id']),1))
        for slot in range(slots):
            targets = outgoing[s["id"]]
            target_id = targets[slot] if slot < len(targets) else -1
            text += f"\nINIT outgoing_connection_{slot}_of_section_{s['id']} = {target_id};\n"
    # Highway initial separation uses local coordinates even on different sections.
    text = re.sub(r"(?m)^\s*INIT abs\(veh___6\d+9___\.rel_pos\) > veh_length \| !ego\.laterally_overlapping_with_veh_\d+;\s*", "\n", text)
    text += "\n" + _geometry_metadata(geometry)
    if job.get('symbolic_endpoint'):
        from .benchmark_symbolic import transform_environment
        text = transform_environment(text,geometry)
    # Squared speed is independent of the current coordinate orientation.
    text = re.sub(r"(?m)^(\s*)((?:veh___6\d+9___|ego)\.v) = (\d+) : (\d+);",
                  r"\1abs(\2) = \3 : \4;", text)
    # Crash reporting in the highway module must also respect section identity.
    text = re.sub(r"(ego\.(?:blamable_)?crash_with_veh_(\d+) := )", r"\1veh_0_and_veh_\2_on_same_seclet & ", text)
    (target / "EnvModel_Endpoints.smv").write_text(text)
    main = (target / "main.smv").read_text()
    main = main.replace('#include "EnvModel.smv"', '#include "EnvModel_Endpoints.smv"')
    main, count = re.subn(r"(?ms)^--ADDONS\s*$.*?^--EO-ADDONS\s*$", "-- Grid initial state supplied by refinement monitor.", main)
    if count != 1:
        raise RuntimeError("could not isolate generated main addons")
    # A local sequential model supplies its own initialization/target after
    # flattening; keep the common VFM dynamics independent of either strategy.
    refinement = "INVARSPEC FALSE;" if job.get("baseline_only") else monitor(graph, geometry)
    main, count = re.subn(r"(?m)^INVARSPEC FALSE;?\s*$", refinement, main)
    if count != 1:
        raise RuntimeError("could not replace generated property")
    model = target / "main_endpoint_grid.smv"
    if job.get("scenario"):
        from .benchmark_scenarios import observer
        boundary='next(refinement_phase) != refinement_phase' if graph.get('semantics')=='stable_concurrent' else 'TRUE'
        main += "\n" + observer(job["scenario"], geometry, graph, boundary)
    model.write_text(main)
    dump(output / "model.json", {"model": str(model.resolve())})


def run(abstract_run, output, bound=200, timeout=1800, road_length=20, parking_length=7,
        radius=6, template_config=ROOT / "main_template.json", gif=True, generate_only=False):
    abstract_run, output = Path(abstract_run).resolve(), Path(output).resolve()
    graph = json.loads((abstract_run / "selected_graph.json").read_text())
    abstract_config = json.loads((abstract_run / "config.json").read_text())
    geometry = build(graph,road_length,parking_length,radius)
    output.mkdir(parents=True,exist_ok=True)
    if (output / "generation_job.json").exists():
        raise FileExistsError(f"use a new output directory: {output}")
    dump(output / "geometry.json", geometry)
    dump(output / "generation_job.json", {"graph":graph,"geometry":geometry,
         "template_config":str(Path(template_config).resolve()), "vfm":abstract_config.get("vfm",{})})
    print(f"Generating VFM ({len(geometry['sections'])} sections, {len(geometry['connections'])} connectors); log: {output / 'generation.log'}",flush=True)
    result = execute([sys.executable,"-m","vfm_grid_python.concretize","--worker",str(output)],
                     ROOT,output / "generation.log",timeout)
    dump(output / "generation_result.json",result)
    if result["status"] != "finished":
        raise RuntimeError(f"VFM generation {result['status']}; inspect {output / 'generation.log'}")
    model = Path(json.loads((output / "model.json").read_text())["model"])
    from .visualize import concrete_plot
    concrete_plot(geometry,output,None,False)
    if generate_only:
        print(f"Model: {model}")
        return {"status":"generated"}
    result = check(model,bound,timeout)
    if result["status"] == "witness":
        from .validate import validate_trace
        report = validate_trace(graph,geometry,model.parent / "trace.xml")
        dump(output / "validation.json",report)
        concrete_plot(geometry,output,model.parent / "trace.xml",gif)
        print(f"Concrete XML: {model.parent / 'trace.xml'}\nConcrete text: {model.parent / 'debug_trace_array.txt'}")
    return result


def arguments(parser):
    parser.add_argument("--concrete-bound",type=int,default=200)
    parser.add_argument("--road-length",type=int,default=20)
    parser.add_argument("--parking-length",type=int,default=7)
    parser.add_argument("--junction-radius",type=int,default=6)
    parser.add_argument("--template-config",type=Path,default=ROOT / "main_template.json")


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--worker":
        _worker(Path(sys.argv[2]))
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstract_run",type=Path)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--timeout",type=float,default=1800)
    parser.add_argument("--no-gif",action="store_true")
    parser.add_argument("--generate-only",action="store_true")
    arguments(parser)
    args=parser.parse_args()
    result=run(args.abstract_run,args.output_dir,args.concrete_bound,args.timeout,
               args.road_length,args.parking_length,args.junction_radius,args.template_config,
               not args.no_gif,args.generate_only)
    return 0 if result["status"] in ("witness","generated") else 2


if __name__ == "__main__":
    raise SystemExit(main())
