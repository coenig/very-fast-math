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
        flag = lambda value: ('TRUE' if value else 'FALSE') if isinstance(value,bool) else value
        lines += [f"DEFINE endpoint_{c['from']}_{c['to']}_from_end := {flag(c['from_end'])};",
                  f"DEFINE endpoint_{c['from']}_{c['to']}_to_end := {flag(c['to_end'])};"]
    for car in range(cars):
        v = f"veh___6{car}9___"
        # Occupancy is an observation of the location tuple, not independent
        # state. Only real connectors have observations in endpoint models.
        for s in sections:
            lines.append(f"DEFINE {v}.is_on_sec_{s['id']} := case {v}.on_straight_section = {s['id']} : 1; TRUE : 0; esac;")
        for c in connections:
            lines.append(f"DEFINE {v}.is_traversing_from_sec_{c['from']}_to_sec_{c['to']} := case {v}.traversion_from = {c['from']} & {v}.traversion_to = {c['to']} : 1; TRUE : 0; esac;")
            lines += [f"INVAR {v}.is_traversing_from_sec_{c['from']}_to_sec_{c['to']} = 1 -> ({v}.v <= 20 & {v}.lane_single);",
                      f"TRANS next({v}.is_traversing_from_sec_{c['from']}_to_sec_{c['to']}) = 1 -> {v}.lane_single;"]
        allowed = set(geometry.get('car_sections', [range(len(sections))]*cars)[car])
        car_roads = [s for s in sections if s['id'] in allowed]
        car_connectors = [c for c in connections if c['from'] in allowed and c['to'] in allowed]
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
        for s in car_roads:
            on = both([at(s["id"]),s.get('active','TRUE')])
            valid.append(f"{on} & {v}.abs_pos >= 0 & {v}.abs_pos <= {s['length']}")
            move(f"{on} & {raw} >= 0 & {raw} <= {s['length']}", at(s["id"], nxt=True), raw, speed)
        for c in car_connectors:
            a, b, length = c["from"], c["to"], c["length"]
            al, bl = sections[a]["length"], sections[b]["length"]
            active = c.get('active','TRUE')
            on = both([at(connector=c),active])
            valid.append(f"{on} & {v}.abs_pos >= 0 & {v}.abs_pos <= {length}")
            move(f"{on} & {raw} >= 0 & {raw} <= {length}", at(connector=c,nxt=True), raw, speed)
            # Frozen symbolic endpoint orientation uses the very same cases as
            # fixed/local geometry. Guards specialize away when labels are fixed.
            for end in (True,False):
                af,bf=c['from_end'],c['to_end']
                ag=('TRUE' if af==end else 'FALSE') if isinstance(af,bool) else (f'({af})' if end else f'!({af})')
                bg=('TRUE' if bf==end else 'FALSE') if isinstance(bf,bool) else (f'({bf})' if end else f'!({bf})')
                a_on=both([at(a),active,ag]); b_on=both([at(b),active,bg])
                if end:
                    move(f"{a_on} & {raw} > {al}", at(connector=c,nxt=True), f"{raw} - {al}", speed)
                    move(f"{on} & {ag} & {raw} < 0", at(a,nxt=True), f"{al} + {raw}", speed)
                    move(f"{b_on} & {raw} > {bl}", at(connector=c,nxt=True), f"{length} - ({raw} - {bl})", f"-({speed})")
                    move(f"{on} & {bg} & {raw} > {length}", at(b,nxt=True), f"{bl} - ({raw} - {length})", f"-({speed})")
                else:
                    move(f"{a_on} & {raw} < 0", at(connector=c,nxt=True), f"-({raw})", f"-({speed})")
                    move(f"{on} & {ag} & {raw} < 0", at(a,nxt=True), f"-({raw})", f"-({speed})")
                    move(f"{b_on} & {raw} < 0", at(connector=c,nxt=True), f"{length} + {raw}", speed)
                    move(f"{on} & {bg} & {raw} > {length}", at(b,nxt=True), f"{raw} - {length}", speed)
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
    possible_resources = []
    for car in range(cars):
        v = f"env.veh___6{car}9___"
        allowed = set(geometry.get('car_sections', [range(len(geometry['sections']))]*cars)[car])
        resources = []
        for s in geometry["sections"]:
            if s['id'] in allowed:
                resources.append((f"edge_{s['id']}", f"{v}.on_straight_section = {s['id']}"))
        for n in geometry["nodes"]:
            if n["kind"] != "N":
                continue
            terms = []
            for s in geometry["sections"]:
                if s['id'] in allowed and n["id"] in s["edge"]:
                    distance = f"{s['length']} - {v}.abs_pos" if s["edge"][1] == n["id"] else f"{v}.abs_pos"
                    terms.append(f"{v}.on_straight_section = {s['id']} & {distance} <= env.veh_length")
            for c in geometry["connections"]:
                if c['from'] in allowed and c['to'] in allowed and c["node"] == n["id"]:
                    terms.append(f"{v}.traversion_from = {c['from']} & {v}.traversion_to = {c['to']}")
            if terms:
                resources.append((f"junction_{n['id']}", either(terms)))
        for name, expr in resources:
            ident = f"occupied_{name}_{car}"
            lines += [f"DEFINE {ident} := {expr};"]
            for other in range(car):
                if name not in possible_resources[other]:
                    continue
                other_id = f"occupied_{name}_{other}"
                lines += [f"INVAR !({ident} & {other_id});",
                          f"TRANS !(({ident} | next({ident})) & ({other_id} | next({other_id})));"]
        possible_resources.append({name for name, _ in resources})
    return "\n".join(lines) + "\n"
