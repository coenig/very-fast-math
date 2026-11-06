"""Flattened VFM declarations and complete state-assignment validation helpers."""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import re

from vfm_grid_python.common import dump, execute

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
