"""Local geometry and declaration-aware continuation of flattened VFM models."""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import re

from .common import dump, execute

BLOCK = re.compile(r"(?m)^(MODULE|FROZENVAR|IVAR|VAR|DEFINE|CONSTANTS|ASSIGN|INIT|INVAR|TRANS|INVARSPEC|LTLSPEC|CTLSPEC|FAIRNESS|JUSTICE|COMPASSION)\b")
LOCATION = re.compile(r"\.(?:on_straight_section|on_section|traversion_from|traversion_to)$")
OCCUPANCY = re.compile(r"^(.*\.)(?:is_on_sec_(\d+)|is_traversing_from_sec_(\d+)_to_sec_(\d+))$")
TOPOLOGY = re.compile(r"^(?:env\.)?(?:section_\d|outgoing_connection_|arclength_from_sec_|endpoint_\d|sin_of_section_|cos_of_section_)")


def blocks(text):
    """Read nuXmv's canonical flat output, not arbitrary source SMV."""
    text = re.sub(r"(?m)^\s*--[^\n]*", "", text)
    matches = list(BLOCK.finditer(text))
    if not matches or matches[0].group(1) != "MODULE":
        raise ValueError("expected a flattened SMV module")
    return [(m.group(1), text[m.end():matches[i+1].start() if i+1<len(matches) else len(text)].strip())
            for i,m in enumerate(matches)]


def statements(text):
    start, depth = 0, 0
    for token in re.finditer(r'"(?:[^"\\]|\\.)*"|\bcase\b|\besac\b|;', text):
        word = token.group()
        if word == "case": depth += 1
        elif word == "esac": depth -= 1
        elif word == ";" and depth == 0:
            yield text[start:token.end()].strip()
            start = token.end()
        if depth < 0: raise ValueError("unbalanced flattened case")
    if depth or text[start:].strip():
        raise ValueError("unterminated flattened statement")


def schema(text):
    result = {}
    for kind, body in blocks(text):
        if kind not in ("VAR", "FROZENVAR", "IVAR"): continue
        if kind == "IVAR" and body:
            raise ValueError("sequential transfer of IVAR inputs is not supported")
        for declaration in statements(body):
            # Quoted planner identifiers may themselves contain colons.
            match = re.fullmatch(r'((?:"[^"]*"|[^":])+?)\s*:\s*(.+);', declaration, re.S)
            if not match: raise ValueError(f"unsupported declaration: {declaration}")
            name, domain = match.groups()
            result[name.strip()] = {"kind":kind, "domain":domain.strip()}
    return result


def flat_body(text, resumed=False):
    parts = []
    for kind, body in blocks(text):
        if kind in ("INVARSPEC", "LTLSPEC", "CTLSPEC"): continue
        if resumed and kind == "INIT": continue
        if resumed and kind == "ASSIGN":
            body = "\n".join(s for s in statements(body) if not re.match(r"init\s*\(",s))
            if not body: continue
        parts.append(kind + " " + body)
    return "\n\n".join(parts) + "\n"


def flatten(model, output, timeout):
    model, output = Path(model).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    flat = output / "baseline.smv"
    script = output / "flatten.cmd"
    script.write_text(f'read_model\nflatten_hierarchy\nwrite_flat_model -o "{flat}"\nquit\n')
    result = execute(["nuXmv","-quiet","-pre","cpp","-source",str(script),str(model)],
                     Path(model).parent, output / "flatten.log", timeout)
    if result["status"] != "finished" or not flat.exists():
        raise RuntimeError(f"flattening {result['status']}: {output / 'flatten.log'}")
    text = flat.read_text()
    variables = schema(text)
    dump(output / "schema.json", variables)
    return text, variables, result


def continuation_sections(frames, mapping, cars):
    """Per-car global road IDs used by an already witnessed continuation.

    Include both arms of occupied connectors, not just straight sections.
    This is geometry support only: no vehicle state is modified or prescribed.
    """
    support = [set() for _ in range(cars)]
    ids = mapping['local_to_global']
    for frame in frames:
        for car in range(cars):
            prefix = f'env.veh___6{car}9___.'
            section = int(frame[prefix+'on_straight_section'])
            occupied = [section] if section >= 0 else [
                int(frame[prefix+field]) for field in ('traversion_from', 'traversion_to')]
            for section in occupied:
                if section >= 0:
                    support[car].add(ids[section])
    return [sorted(sections) for sections in support]


class GeometryIndex:
    def __init__(self, geometry):
        self.geometry = geometry
        self.incident = {n['id']:set() for n in geometry['nodes']}
        self.connectors = {s['id']:[] for s in geometry['sections']}
        self.nodes = {n['id']:n for n in geometry['nodes']}
        for s in geometry['sections']:
            for n in s['edge']: self.incident[n].add(s['id'])
        for c in geometry['connections']:
            self.connectors[c['from']].append(c)
            self.connectors[c['to']].append(c)

    def select(self, left, right, previous=None, following=None, incoming=None, continuation=None,
               retained_sections=None):
        # Without a continuation keep the historical neighbourhood selection
        # for callers relying on it (including the stable contract).
        if following is None:
            selected = set().union(*(self.incident[n] for n in set(left+right)))
        else:
            selected = set()
            car_sections = []
            for car, (u, v, w) in enumerate(zip(left, right, following)):
                support = set(retained_sections[car]) if retained_sections is not None else set()
                moves = [(u, v), (v, w)]
                if incoming is not None:
                    moves.append((incoming[car], u))
                if continuation is not None:
                    moves.append((continuation[0][car], continuation[1][car]))
                for a, b in moves:
                    if a != b:
                        road = self.incident[a] & self.incident[b]
                        if len(road) != 1:
                            raise ValueError(f'no unique road for abstract move {a}->{b}')
                        support.update(road)
                prefix = f'env.veh___6{car}9___.'
                for field in ('on_straight_section', 'traversion_from', 'traversion_to'):
                    value = int((previous or {}).get(prefix+field, -1))
                    if value >= 0:
                        support.add(value)
                if u == v:
                    # Waiting at an abstract node permits any incident endpoint
                    # or connector. Picking just one road can force two cars
                    # onto the same reserved section, removing valid placements.
                    # Retain these alternatives also after a handoff: the initial
                    # state stays fixed, but the car may reposition within this
                    # junction. This adds at most four roads on a grid.
                    if not self.incident[u]:
                        raise ValueError(f'car {car} waits at an isolated node')
                    support.update(self.incident[u])
                car_sections.append(support)
                selected.update(support)
        if previous is None:
            # VFM's *initial* auxiliary ego.on_section is hard-coded to 0.
            # Preserve global section 0 rather than changing this initial state
            # by accidentally interpreting it as the first local section.
            selected.add(0)
        # Also retain auxiliary section-valued state, notably ego.on_section,
        # even if it is not the physical vehicle location.
        for name,value in (previous or {}).items():
            if LOCATION.search(name) and int(value) >= 0: selected.add(int(value))
        if not selected: raise ValueError("local step has no physical sections")
        ids = sorted(selected)
        to_local = {s:i for i,s in enumerate(ids)}
        sections = [deepcopy(self.geometry['sections'][i]) for i in ids]
        for s in sections: s['id'] = to_local[s['id']]
        nodes = sorted({n for s in sections for n in s['edge']})
        connections = {c['id']:c for s in ids for c in self.connectors[s]
                       if c['from'] in selected and c['to'] in selected}
        connections = [deepcopy(connections[i]) for i in sorted(connections)]
        for i,c in enumerate(connections):
            c.update(id=i, **{'from':to_local[c['from']], 'to':to_local[c['to']]})
        origin = sections[0]['source'][:]
        angle = sections[0]['angle']
        co, si = round(math.cos(math.radians(angle))),round(math.sin(math.radians(angle)))
        def transform(p):
            x,y=p[0]-origin[0],p[1]-origin[1]
            return [co*x+si*y,-si*x+co*y]
        for s in sections:
            s['source'],s['drain']=transform(s['source']),transform(s['drain'])
            s['source_x'],s['source_y']=s['source']
            s['angle']=(s['angle']-angle)%360
        for c in connections: c['curve']=[transform(p) for p in c['curve']]
        geometry = {**{k:self.geometry[k] for k in ('schema','road_length','parking_length','junction_radius')},
                    'nodes':[self.nodes[n] for n in nodes],
                    'centers':{n:transform(self.geometry['centers'][n]) for n in nodes},
                    'sections':sections,'connections':connections}
        if 'junction_extension' in self.geometry:
            geometry['junction_extension'] = deepcopy(self.geometry['junction_extension'])
            if 'junction_policy' in self.geometry:
                geometry['junction_policy'] = self.geometry['junction_policy']
        if following is not None:
            geometry['car_sections'] = [sorted(to_local[s] for s in support) for support in car_sections]
        mapping={'local_to_global':ids,'global_to_local':to_local,
                 'global_origin':origin,'rotation_degrees':angle}
        return geometry,mapping


def global_state(state, mapping):
    """Keep state/vehicle observations in global IDs; geometry is exported once."""
    ids = mapping['local_to_global']
    result = {}
    for name,value in state.items():
        if TOPOLOGY.match(name) or name.startswith(('__expr','occupied_')): continue
        match = OCCUPANCY.match(name)
        if match:
            prefix,on,a,b=match.groups()
            name = (f'{prefix}is_on_sec_{ids[int(on)]}' if on is not None else
                    f'{prefix}is_traversing_from_sec_{ids[int(a)]}_to_sec_{ids[int(b)]}')
        elif LOCATION.search(name) and int(value)>=0:
            value=str(ids[int(value)])
        result[name]=value
    return result


def validate_value(name, value, domain):
    value=str(value).strip()
    if domain == 'boolean': valid=value in ('TRUE','FALSE')
    elif domain == 'integer': valid=bool(re.fullmatch(r'-?\d+',value))
    elif domain.startswith('{'):
        valid=value in {v.strip() for v in domain[1:-1].split(',')}
    elif re.fullmatch(r'-?\d+\s*\.\.\s*-?\d+',domain):
        lo,hi=map(int,domain.split('..'))
        valid=bool(re.fullmatch(r'-?\d+',value)) and lo<=int(value)<=hi
    else:
        raise ValueError(f'unsupported state domain for {name}: {domain}')
    if not valid: raise ValueError(f'out-of-domain handoff: {name}={value}, domain {domain}')
    return value


def handoff(previous, variables, mapping):
    """Assign every declared state variable; never silently reset policy state."""
    ids = mapping['local_to_global']
    to_local = {g:i for i,g in enumerate(ids)}
    result={}
    for name,decl in variables.items():
        if TOPOLOGY.match(name):
            # All other fixed geometry is constant-folded by nuXmv. One
            # segment per section leaves only these frozen origins declared.
            if not re.fullmatch(r'env\.section_\d+_segment_0_pos_begin',name):
                raise ValueError(f'unsupported nonconstant topology state: {name}')
            value='0'
        else:
            key=name
            match=OCCUPANCY.match(name)
            if match:
                prefix,on,a,b=match.groups()
                # Occupancy booleans are VAR aliases constrained by INVAR.
                # New sections need their values derived, not reset arbitrarily.
                if on is not None:
                    value=str(int(int(previous[prefix+'on_straight_section'])==ids[int(on)]))
                else:
                    value=str(int((int(previous[prefix+'traversion_from']),int(previous[prefix+'traversion_to']))
                                  ==(ids[int(a)],ids[int(b)])))
            else:
                if key not in previous: raise ValueError(f'missing persistent handoff field: {key}')
                value=previous[key]
                if LOCATION.search(name) and int(value)>=0:
                    if int(value) not in to_local: raise ValueError(f'local model omitted occupied section {value}')
                    value=str(to_local[int(value)])
        result[name]=validate_value(name,value,decl['domain'])
    return result
