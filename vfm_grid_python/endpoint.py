"""Compile the fixed endpoint relation included by VFM's endpoint template.

Each connector has one coordinate from its canonical `from` endpoint to its
`to` endpoint. Velocity is signed in the current seclet's coordinate frame.
Acceleration is applied before frame conversion. Crossing at most one seclet
boundary per tick is enforced by the next-state position bounds.
"""
from .common import both, either


def movement(geometry, cars):
    sections, connections = geometry["sections"], geometry["connections"]
    lines = ["-- Endpoint motion: generated from selected section incidences only."]
    for c in connections:
        lines += [f"DEFINE endpoint_{c['from']}_{c['to']}_from_end := {'TRUE' if c['from_end'] else 'FALSE'};",
                  f"DEFINE endpoint_{c['from']}_{c['to']}_to_end := {'TRUE' if c['to_end'] else 'FALSE'};"]
    for car in range(cars):
        v = f"veh___6{car}9___"
        speed = f"{v}.endpoint_step_v"
        raw = f"{v}.next_abs_pos"
        lines += [f"DEFINE {speed} := min(max({v}.v + {v}.a, -max_vel), max_vel);"]
        options = []

        def at(section=None, connector=None, nxt=False):
            def var(field):
                name = f"{v}.{field}"
                return f"next({name})" if nxt else name
            return both([f"{var('on_straight_section')} = {section if section is not None else -1}",
                         f"{var('traversion_from')} = {connector['from'] if connector else -1}",
                         f"{var('traversion_to')} = {connector['to'] if connector else -1}"])

        def move(condition, destination, position, velocity):
            options.append(both([condition, destination, f"next({v}.abs_pos) = ({position})",
                                 f"next({v}.v) = ({velocity})"]))

        valid = []
        for s in sections:
            on = at(s["id"])
            valid.append(f"{on} & {v}.abs_pos >= 0 & {v}.abs_pos <= {s['length']}")
            move(f"{on} & {raw} >= 0 & {raw} <= {s['length']}", at(s["id"], nxt=True), raw, speed)
        for c in connections:
            a, b, length = c["from"], c["to"], c["length"]
            al, bl = sections[a]["length"], sections[b]["length"]
            on = at(connector=c)
            valid.append(f"{on} & {v}.abs_pos >= 0 & {v}.abs_pos <= {length}")
            move(f"{on} & {raw} >= 0 & {raw} <= {length}", at(connector=c,nxt=True), raw, speed)
            if c["from_end"]:
                move(f"{at(a)} & {raw} > {al}", at(connector=c,nxt=True), f"{raw} - {al}", speed)
                move(f"{on} & {raw} < 0", at(a,nxt=True), f"{al} + {raw}", speed)
            else:
                move(f"{at(a)} & {raw} < 0", at(connector=c,nxt=True), f"-({raw})", f"-({speed})")
                move(f"{on} & {raw} < 0", at(a,nxt=True), f"-({raw})", f"-({speed})")
            if c["to_end"]:
                move(f"{at(b)} & {raw} > {bl}", at(connector=c,nxt=True), f"{length} - ({raw} - {bl})", f"-({speed})")
                move(f"{on} & {raw} > {length}", at(b,nxt=True), f"{bl} - ({raw} - {length})", f"-({speed})")
            else:
                move(f"{at(b)} & {raw} < 0", at(connector=c,nxt=True), f"{length} + {raw}", speed)
                move(f"{on} & {raw} > {length}", at(b,nxt=True), f"{raw} - {length}", speed)
        lines += ["INVAR " + either(valid) + ";", "TRANS " + either(options) + ";"]
        for other in range(car):
            w = f"veh___6{other}9___"
            same = f"veh_{car}_and_veh_{other}_on_same_seclet"
            unchanged = both([f"next({p}.{field}) = {p}.{field}" for p in (v,w)
                              for field in ("on_straight_section", "traversion_from", "traversion_to")])
            lines += [f"TRANS ({same} & {unchanged}) -> (({v}.abs_pos < {w}.abs_pos) <-> (next({v}.abs_pos) < next({w}.abs_pos)));"]
    return "\n".join(lines) + "\n"


def collision_constraints(geometry, cars):
    """Reserve junction envelopes and physical sections across a complete tick.

This deliberately conservative refinement permits one car per physical edge,
and one car in a junction envelope (all of its curves plus vehicle-length
approach margins). Swept reservations prevent opposite cars crossing a border
in a single discrete step. It can reject some otherwise feasible close convoys.
"""
    lines = []
    for car in range(cars):
        v = f"env.veh___6{car}9___"
        resources = []
        for s in geometry["sections"]:
            resources.append((f"edge_{s['id']}", f"{v}.on_straight_section = {s['id']}"))
        for n in geometry["nodes"]:
            if n["kind"] != "N":
                continue
            terms = []
            for s in geometry["sections"]:
                if n["id"] in s["edge"]:
                    distance = f"{s['length']} - {v}.abs_pos" if s["edge"][1] == n["id"] else f"{v}.abs_pos"
                    terms.append(f"{v}.on_straight_section = {s['id']} & {distance} <= env.veh_length")
            for c in geometry["connections"]:
                if c["node"] == n["id"]:
                    terms.append(f"{v}.traversion_from = {c['from']} & {v}.traversion_to = {c['to']}")
            resources.append((f"junction_{n['id']}", either(terms)))
        for name, expr in resources:
            ident = f"occupied_{name}_{car}"
            lines += [f"DEFINE {ident} := {expr};"]
            for other in range(car):
                other_id = f"occupied_{name}_{other}"
                lines += [f"INVAR !({ident} & {other_id});",
                          f"TRANS !(({ident} | next({ident})) & ({other_id} | next({other_id})));"]
    return "\n".join(lines) + "\n"
