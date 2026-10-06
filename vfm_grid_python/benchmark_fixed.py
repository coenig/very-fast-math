"""Common concrete scenario grammar, optionally specialized to a fixed layout."""
import re

from .abstract import render
from .benchmark_scenarios import observer
from .endpoint import collision_constraints
from .geometry import location_expression


def scenario_constraints(config, graph, geometry, symbolic=False, fixed=None):
    """Reuse abstract structural/role constraints, replacing motion observations.

    Concrete cars may be between nodes (loc=-1); node-exclusive abstract motion
    is replaced by the shared concrete section/junction collision constraints.
    """
    definition = dict(config)
    if not symbolic or fixed is not None:
        layout = fixed if fixed is not None else graph
        definition.update(edges=layout['edges'],labels={str(n['id']):n['kind'] for n in layout['nodes']})
    definition.pop("scenario_stages", None)
    definition['semantics']='legacy'  # Stable abstract transition guards are replaced by the concrete macro controller.
    lines = []
    for line in render(definition).splitlines():
        if line in ("MODULE main", "VAR") or re.match(r"\s*loc_\d+ :", line):
            continue
        if line.startswith(("TRANS ", "INVARSPEC ")):
            continue
        if line.startswith("INVAR ") and re.search(r"\bloc_\d+\b", line):
            continue
        if symbolic and re.match(r'\s*(?:edge_\d+_\d+|parking_\d+) :',line):
            continue
        lines.append(line)
    if symbolic:
        from .benchmark_symbolic import observe, collisions, edge_name
        lines += [f'DEFINE {edge_name(s)} := {edge_name(s,"env.")};' for s in geometry['sections']]
        lines += [f"DEFINE parking_{n['id']} := env.parking_{n['id']};" for n in geometry['nodes']]
        observe_node = observe
    else:
        observe_node = location_expression
    from .stable import enabled, symbolic_observation, macro_controller
    stable=enabled(config)
    if stable:
        if not symbolic: raise ValueError('stable full models require the shared endpoint compiler')
        observe_node=symbolic_observation
        lines.append(macro_controller(config,geometry))
    for car in range(config["cars"]):
        branches = " ".join(f"{observe_node(geometry,car,n['id'])} : {n['id']};"
                            for n in graph["nodes"])
        lines += [f"DEFINE loc_{car} := case {branches} TRUE : -1; esac;",
                  f"INIT env.veh___6{car}9___.v = 0;"]
    # Observer atom translation can use loc_i directly: at a parking node loc_i
    # is defined only when stopped, just as in geometry.location_expression.
    lines += [observer(config,boundary='macro_busy & !next(macro_busy)' if stable else 'TRUE'), (collisions if symbolic else collision_constraints)(geometry, config["cars"]),
              "INVARSPEC !scenario_accepted;"]
    text = "\n".join(lines) + "\n"
    # Mixed integer VFM models cannot use the SMT engine's integer mod/div.
    # Coordinates have a finite domain, so compile an exact lookup instead.
    def coordinates(match):
        variable, operator, divisor = match.groups()
        divisor = int(divisor)
        count = config['rows'] * config['columns']
        terms = ' '.join(f'{variable} = {n} : {n//divisor if operator=="/" else n%divisor};'
                         for n in range(count))
        return '(case ' + terms + ' TRUE : -1; esac)'
    return re.sub(r'\b((?:loc|start|goal)_\d+|role_\w+)\s+(mod|/)\s+(\d+)', coordinates, text)
