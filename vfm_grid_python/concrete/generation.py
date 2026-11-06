"""Isolated native VFM generation worker; logs are redirected by its caller."""
import json
import re
import sys
from pathlib import Path
from vfm_grid_python.common import dump
from .dynamics import movement

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
        from vfm_grid_python.concrete.symbolic import symbolic_movement
        config['ENDPOINT_TRANSITIONS'] = symbolic_movement(geometry,cars)
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
    from vfm_grid_python.concrete.braking import effective_parameters, speed_constraints
    parameters = effective_parameters(job["template_config"], overrides)
    limits, braking_report = speed_constraints(geometry, cars, parameters,
        job.get('braking_margin', 0), symbolic=job.get('symbolic_endpoint', False))
    config['ENDPOINT_TRANSITIONS'] += '\n' + limits
    dump(output / 'braking_check.json', braking_report)
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
        from vfm_grid_python.concrete.symbolic import transform_environment
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
    # Each approach supplies its initialization and property separately;
    # keep the VFM dynamics independent of the search strategy.
    refinement = "INVARSPEC FALSE;"
    main, count = re.subn(r"(?m)^INVARSPEC FALSE;?\s*$", refinement, main)
    if count != 1:
        raise RuntimeError("could not replace generated property")
    model = target / "main_endpoint_grid.smv"
    model.write_text(main)
    dump(output / "model.json", {"model": str(model.resolve())})


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] != "--worker":
        raise SystemExit("Internal worker: --worker OUTPUT")
    _worker(Path(sys.argv[2]))
