"""Stable concurrent milestone contracts; legacy models never import these guards.

The certificate is for real-valued longitudinal motion with finite footprints,
not a completeness result for bounded integer checking or arbitrary VFM profiles.
"""
from .common import both, either, grid_edges
import math

VERSION = 'stable-concurrent.v1'


def enabled(config):
    mode = config.get('semantics', 'legacy')
    if mode not in ('legacy', 'stable_concurrent'):
        raise ValueError(f'unknown semantics: {mode}')
    return mode == 'stable_concurrent'


def profile(config, road=20, parking=7, radius=6):
    # Restrict this certificate to the audited one-lane endpoint profile.
    allowed = {'LANES_MIN_SPEEDS': '@(-5)@', 'LANES_MAX_SPEEDS': '@(5)@', 'MAXSPEEDNONEGO': 5}
    if any(k not in allowed or v != allowed[k] for k,v in config.get('vfm', {}).items()):
        raise ValueError('uncertified_model_profile: unsupported VFM overrides')
    if (road, parking, radius) != (20, 7, 6):
        raise ValueError('uncertified_model_profile: certified geometry is 20/7/6 metres')
    # A rear-reference finite vehicle is contained in a disc of this radius.
    # Cubic curves lie in the convex hull of their control points; the local
    # endpoint compiler's turning hull is inside the radius-sized node square.
    body_radius=math.hypot(5,1)
    pocket=1
    margins=dict(parking_clearance=parking-pocket-body_radius,
                 node_separation=road-2*(pocket+body_radius),
                 parked_to_neighbour=road-parking-5-pocket-body_radius,
                 junction_radius=radius-body_radius)
    if min(margins.values())<=0: raise ValueError('uncertified_model_profile: insufficient finite-body clearance')
    return dict(version=VERSION, road_length=road, parking_length=parking,
                junction_radius=radius, max_vehicle_length=5, max_vehicle_width=2,
                enclosure='Bezier control hull plus finite-body radius, one-metre endpoint pockets',
                clearance_margins=margins,
                real_valued_theorem=True, integer_bmc_guaranteed=False,
                auxiliary_extension='assumption; full generated trace validation remains mandatory')


def anchors(graph):
    kinds = {n['id']: n['kind'] for n in graph['nodes']}
    neighbours = {u: [] for u in kinds}
    for u,v in graph['edges']:
        neighbours[u].append(v); neighbours[v].append(u)
    return {u: min(vs, key=lambda v: (kinds[v]!='N',v)) if vs else -1
            for u,vs in neighbours.items()}


def definitions(config):
    edges=grid_edges(config['rows'],config['columns'])
    lines=[]
    for u in range(config['rows']*config['columns']):
        neighbours=sorted(v if u==a else a for a,v in edges if u in (a,v))
        choices=[]
        for prefer_normal in (True,False):
            for v in neighbours:
                a,b=sorted((u,v))
                choices.append(f'edge_{a}_{b}'+(f' & !parking_{v}' if prefer_normal else '')+f' : {v};')
        lines.append(f'DEFINE stable_anchor_{u} := case '+ ' '.join(choices)+' TRUE : -1; esac;')
    return '\n'.join(lines)+'\n'


def edge_used(u,v,source,target):
    anchor=lambda x: either([f'{x} = {u} & stable_anchor_{u} = {v}',
                            f'{x} = {v} & stable_anchor_{v} = {u}'])
    return either([anchor(source),anchor(target),
                   f'{source} = {u} & {target} = {v}',f'{source} = {v} & {target} = {u}'])


def compatible(config,sources,targets):
    terms=[]
    for i in range(config['cars']):
        for j in range(i):
            for u,v in grid_edges(config['rows'],config['columns']):
                terms.append(f'!({edge_used(u,v,sources[i],targets[i])} & {edge_used(u,v,sources[j],targets[j])})')
            for u in range(config['rows']*config['columns']):
                a=either([f'{sources[i]} = {u}',f'{targets[i]} = {u}'])
                b=either([f'{sources[j]} = {u}',f'{targets[j]} = {u}'])
                terms.append(f'!(!parking_{u} & {a} & {b})')
    return both(terms)


def abstract_guards(config):
    profile(config)
    src=[f'loc_{i}' for i in range(config['cars'])]
    dst=[f'next(loc_{i})' for i in range(config['cars'])]
    lines=[definitions(config),'INVAR '+compatible(config,src,src)+';',
           'TRANS '+compatible(config,src,dst)+';']
    from .benchmark_scenarios import predicate
    import re
    for k,stage in enumerate(config.get('scenario_stages',[])):
        hold=predicate(stage.get('hold',True),config)
        transit=re.sub(r'\bloc_(\d+)\b',lambda m:f'(case next(loc_{m[1]}) = loc_{m[1]} : loc_{m[1]}; TRUE : -1; esac)',hold)
        lines.append(f'TRANS scenario_phase = {k} -> ({transit});')
    return '\n'.join(lines)+'\n'


def validate_witness(graph,config):
    """Recheck contract admission independently of the abstract solver export."""
    chosen=anchors(graph); kinds={n['id']:n['kind'] for n in graph['nodes']}
    def resources(u,v):
        if chosen[u]<0 or chosen[v]<0: raise ValueError('stable holding node is isolated')
        edges={frozenset((u,chosen[u])),frozenset((v,chosen[v]))}
        if u!=v: edges.add(frozenset((u,v)))
        return {('edge',tuple(sorted(e))) for e in edges}|{('node',n) for n in (u,v) if kinds[n]=='N'}
    milestones=graph['milestones']
    pairs=[(milestones[0],milestones[0])]+list(zip(milestones,milestones[1:]))
    for left,right in pairs:
        footprints=[resources(u,v) for u,v in zip(left,right)]
        if any(a&b for i,a in enumerate(footprints) for b in footprints[:i]):
            raise ValueError('abstract witness violates stable concurrent resource contract')
    def evaluate(term,locations):
        if type(term) is bool: return term
        op,value=next(iter(term.items()))
        if op=='all': return all(evaluate(t,locations) for t in value)
        if op=='any': return any(evaluate(t,locations) for t in value)
        if op=='not': return not evaluate(value,locations)
        if op!='at': raise ValueError('unsupported stable hold predicate')
        car,target=value
        if target.startswith('role_'): node=graph['roles'][target[5:]]
        else:
            kind,index=target.split('_'); node=graph[kind+'s'][int(index)]
        return locations[car]==node
    stages=config.get('scenario_stages',[])
    for i,(left,right) in enumerate(zip(milestones,milestones[1:])):
        if stages:
            phase=graph['scenario_phases'][i]
            if phase<len(stages) and not evaluate(stages[phase].get('hold',True),[u if u==v else -1 for u,v in zip(left,right)]):
                raise ValueError('abstract hold is not preserved during travel')
    return dict(valid=True,holding_neighbours=chosen,transitions=len(milestones)-1)


def observation(geometry,car,node,graph):
    neighbour=anchors(graph)[node]
    section=next((s for s in geometry['sections'] if set(s['edge'])=={node,neighbour}),None)
    if section is None: return 'FALSE'
    p=f'env.veh___6{car}9___'
    pos=section['length'] if section['edge'][1]==node else 0
    return both([f"{p}.on_straight_section = {section['id']}",f'{p}.abs_pos = {pos}',
                 f'{p}.v = 0',f'{p}.a = 0'])


def support(geometry,car,u,v,graph):
    """Selected edge plus endpoint anchoring/turning pockets; no third junction."""
    if u==v: return observation(geometry,car,u,graph)
    lookup={frozenset(s['edge']):s for s in geometry['sections']}
    chosen=anchors(graph); travel=lookup[frozenset((u,v))]
    p=f'env.veh___6{car}9___'
    terms=[f"{p}.on_straight_section = {travel['id']}"]
    for node in (u,v):
        anchor=lookup[frozenset((node,chosen[node]))]
        if anchor['id']==travel['id']: continue
        distance=f"{anchor['length']} - {p}.abs_pos" if anchor['edge'][1]==node else f'{p}.abs_pos'
        terms.append(f"{p}.on_straight_section = {anchor['id']} & ({distance}) >= 0 & ({distance}) <= 1")
        terms += [f"{p}.traversion_from = {c['from']} & {p}.traversion_to = {c['to']}"
                  for c in geometry['connections'] if c['node']==node and {c['from'],c['to']}=={anchor['id'],travel['id']}]
    return either(terms)


def symbolic_observation(geometry,car,node):
    from .benchmark_symbolic import at_end
    p=f'env.veh___6{car}9___'; terms=[]
    for s in geometry['sections']:
        if node not in s['edge']: continue
        other=next(n for n in s['edge'] if n!=node)
        pos=f"case {at_end(s,node,'env.')} : env.section_{s['id']}_end; TRUE : 0; esac"
        terms.append(both([f'stable_anchor_{node} = {other}',f"{p}.on_straight_section = {s['id']}",f'{p}.abs_pos = ({pos})']))
    return both([either(terms),f'{p}.v = 0',f'{p}.a = 0'])


def symbolic_support(geometry,car,u,v):
    from .benchmark_symbolic import at_end
    if u==v: return symbolic_observation(geometry,car,u)
    travel=next(s for s in geometry['sections'] if set(s['edge'])=={u,v})
    p=f'env.veh___6{car}9___'; terms=[f"{p}.on_straight_section = {travel['id']}"]
    for node in (u,v):
        for anchor in geometry['sections']:
            if node not in anchor['edge'] or anchor['id']==travel['id']: continue
            other=next(n for n in anchor['edge'] if n!=node)
            distance=f"case {at_end(anchor,node,'env.')} : env.section_{anchor['id']}_end - {p}.abs_pos; TRUE : {p}.abs_pos; esac"
            parts=[f"{p}.on_straight_section = {anchor['id']} & ({distance}) >= 0 & ({distance}) <= 1"]
            parts += [f"{p}.traversion_from = {c['from']} & {p}.traversion_to = {c['to']}"
                      for c in geometry['connections'] if c['node']==node and {c['from'],c['to']}=={anchor['id'],travel['id']}]
            terms.append(both([f'stable_anchor_{node} = {other}',either(parts)]))
    return either(terms)


def macro_controller(config,geometry):
    """Joint target selection inside the full concrete search, not an abstract pass."""
    n=config['rows']*config['columns']; cars=config['cars']
    edges=grid_edges(config['rows'],config['columns'])
    src=[f'macro_source_{i}' for i in range(cars)]; dst=[f'macro_target_{i}' for i in range(cars)]
    lines=[definitions(config),'VAR macro_busy : boolean;', 'INIT !macro_busy;']
    reached=both([either([f'{dst[c]} = {u} & {symbolic_observation(geometry,c,u)}' for u in range(n)]) for c in range(cars)])
    lines += ['DEFINE macro_reached := '+reached+';',
              'TRANS !macro_busy -> next(macro_busy);',
              'TRANS macro_busy -> (next(macro_busy) <-> !next(macro_reached));',
              'INVAR macro_busy -> '+compatible(config,src,dst)+';',
              'INVAR !macro_busy -> '+compatible(config,src,src)+';']
    for c in range(cars):
        lines += [f'VAR {src[c]} : 0..{n-1}; {dst[c]} : 0..{n-1};',
                  f'INIT {src[c]} = start_{c}; INIT {dst[c]} = start_{c};',
                  f'TRANS !macro_busy -> next({src[c]}) = {src[c]};',
                  f'TRANS macro_busy -> next({dst[c]}) = {dst[c]};',
                  f'TRANS macro_busy -> next({src[c]}) = (case next(macro_busy) : {src[c]}; TRUE : {dst[c]}; esac);']
        legal=[f'{src[c]} = {dst[c]}']+[f'edge_{u}_{v} & {src[c]} = {a} & {dst[c]} = {b}' for u,v in edges for a,b in ((u,v),(v,u))]
        lines += ['INVAR macro_busy -> '+either(legal)+';',
                  'INVAR !macro_busy -> '+either([f'{src[c]} = {u} & {symbolic_observation(geometry,c,u)}' for u in range(n)])+';']
        pairs=[(u,u) for u in range(n)]+[(a,b) for u,v in edges for a,b in ((u,v),(v,u))]
        lines += ['INVAR macro_busy -> '+either([f'{src[c]} = {u} & {dst[c]} = {v} & {symbolic_support(geometry,c,u,v)}' for u,v in pairs])+';']
    from .benchmark_scenarios import predicate
    import re
    for k,stage in enumerate(config['scenario_stages']):
        hold=predicate(stage.get('hold',True),config)
        transit=re.sub(r'\bloc_(\d+)\b',lambda m:f'(case macro_source_{m[1]} = macro_target_{m[1]} : macro_source_{m[1]}; TRUE : -1; esac)',hold)
        lines.append(f'INVAR macro_busy & scenario_phase = {k} -> ({transit});')
    return '\n'.join(lines)+'\n'
