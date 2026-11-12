"""Versioned joint-valued paper scenarios, shared by all four approaches.

Legacy node roles and event/hold observers are deliberately not reinterpreted.
Paper stages are P0 & (H0 U (P1 & ...)); P0 holds initially and multiple events
may occur in the same state. Concrete atoms use the actual integer VFM state.
"""
import re
from vfm_grid_python.common import both, either, ROOT
from .roads import inventory

VERSION = 'joint-paper.v1'


def names(config):
    return ([f'{p}_{i}' for p in ('start','goal') for i in range(config['cars'])]
            + [f'role_{n}' for n in config.get('roles',{})])


def role(name, config):
    if name not in names(config):
        raise ValueError(f'unknown joint role {name!r}')
    return name


def node(name, config):
    return role(name,config)+'_node'


def placement(term, config):
    if type(term) is bool:
        return 'TRUE' if term else 'FALSE'
    if not isinstance(term,dict) or len(term)!=1:
        raise ValueError('placement must have exactly one operator')
    op, val = next(iter(term.items()))
    if op == 'all':
        return both([placement(t,config) for t in val])
    if op in ('parking','dead'):
        z=role(val,config)
        return z+'_parking' if op=='parking' else f'{z}_degree = 1'
    if op=='degree':
        z,k=val
        if type(k) is not int or not 0 <= k <= 4:
            raise ValueError('degree must be in 0..4')
        return f'{role(z,config)}_degree = {k}'
    if op in ('distinct','same_node','adj','sec'):
        a,b = [role(z,config) for z in val]
        if op=='distinct': return f'{a} != {b}'
        if op=='same_node': return f'{a}_node = {b}_node'
        roads,joints=inventory(config)
        return either([both([f'{a if op=="sec" else a+"_node"} = {u}',
                             f'{b if op=="sec" else b+"_node"} = {v}',r['active']])
                       for r in roads if r['kind']=='section'
                       for u,v in ((r['joints'] if op=='sec' else r['edge']),
                                   list(reversed(r['joints'] if op=='sec' else r['edge'])))])
    if op in ('row','col'):
        a, relation, b = val
        if relation not in ('<','=', '>','<=','>='):
            raise ValueError('invalid coordinate comparison')
        operator='/' if op=='row' else 'mod'
        lhs=f'({node(a,config)} {operator} {config["columns"]})'
        rhs=str(b) if type(b) is int else f'({node(b,config)} {operator} {config["columns"]})'
        return f'{lhs} {relation} {rhs}'
    raise ValueError(f'unknown paper placement predicate {op}')


def atom(term, config, abstract=False):
    if type(term) is bool: return 'TRUE' if term else 'FALSE'
    if not isinstance(term,dict) or len(term)!=1:
        raise ValueError('event must have exactly one operator')
    op,val=next(iter(term.items()))
    if op=='all': return both([atom(x,config,abstract) for x in val])
    if op in ('wait','parked'):
        return both([atom({'at':val},config,abstract),atom({'stop':val[0]},config,abstract)])
    if op in ('stop','safe'): i=val
    elif op in ('at','at_junction','slow'): i=val[0]
    else: raise ValueError(f'unsupported paper atom {op}; legacy queues/negation are not paper atoms')
    if type(i) is not int or not 0 <= i < config['cars']:
        raise ValueError('unknown car')
    if op in ('at','at_junction'):
        z=role(val[1],config)
        if op=='at':
            sections=sum(r['kind']=='section' for r in inventory(config)[0])
            return f'(road_{i} < {sections} & road_{i} >= 0 & joint_{i} = {z})'
        return f'junction_node_{i} = {z}_node'
    if op=='slow' and (type(val[1]) is not int or val[1]<0):
        raise ValueError('Slow bound must be a nonnegative integer in VFM')
    if abstract: return 'TRUE'
    v=f'env.veh___6{i}9___.v'
    if op=='stop': return f'{v} = 0'
    if op=='slow': return f'abs({v}) <= {val[1]}'
    return f'paper_safe_{i}'


def holding_joints(term):
    """Pending conditions are conjunctions of Wait (At and Stop)."""
    if term is True: return []
    if not isinstance(term,dict) or len(term)!=1:
        raise ValueError('paper hold must be true or a conjunction of Wait/Parked')
    op,val=next(iter(term.items()))
    if op=='all':
        ats=[]; stops=set()
        def visit(t):
            if t is True: return
            if not isinstance(t,dict) or len(t)!=1: raise ValueError('invalid pending condition')
            kind,value=next(iter(t.items()))
            if kind=='all':
                for x in value: visit(x)
            elif kind in ('wait','parked'):
                ats.append(value);stops.add(value[0])
            elif kind=='at': ats.append(value)
            elif kind=='stop': stops.add(value)
            else: raise ValueError('paper holds require At and Stop')
        for t in val: visit(t)
        if {p[0] for p in ats}!=stops: raise ValueError('every pending At needs Stop for the same car')
        return ats
    if op in ('wait','parked'): return [val]
    # Accept the expanded mathematical syntax At_i(z) & Stop_i too.
    raise ValueError('use wait/parked in paper pending conditions')


def observer(config, abstract=False):
    stages=config['scenario_stages']; n=len(stages)
    events=[atom(s['event'],config,abstract) for s in stages]
    def at_hold(k):
        return both([atom({'at':p},config,True) for p in holding_joints(stages[k].get('hold',True))])
    initial=either([both([f'scenario_phase = {k}',*events[:k]]) for k in range(1,n+1)])
    options=[f'scenario_phase = {n} & next(scenario_phase) = {n}']
    for k in range(1,n):
        hold=atom(stages[k-1].get('hold',True),config,abstract)
        options.append(both([f'scenario_phase = {k}',hold,f'next(scenario_phase) = {k}']))
        for r in range(k+1,n+1):
            target=both(events[k:r]+([at_hold(k-1)] if abstract else []))
            options.append(both([f'scenario_phase = {k}',hold,
                                 f'next(scenario_phase) = {r}',f'next({target})']))
    return '\n'.join([f'VAR scenario_phase : 0..{n};',f'INIT {initial};',
                       'TRANS '+either(options)+';',
                       f'DEFINE scenario_accepted := scenario_phase = {n};'])


def static(config):
    """Single source for joint declarations and gamma (also consumed by All-SMT)."""
    roads,joints=inventory(config); out=['FROZENVAR']
    out += [f' {z} : 0..{len(joints)-1};' for z in names(config)]
    owners={j:r for r in roads if r['kind']=='section' for j in r['joints']}
    for z in names(config):
        out += [f'DEFINE {z}_node := case '+' '.join(f'{z} = {j} : {v["node"]};' for j,v in enumerate(joints))+' TRUE : 0; esac;',
                f'DEFINE {z}_parking := '+either([f'{z} = {j} & parking_{v["node"]}' for j,v in enumerate(joints)])+';',
                f'DEFINE {z}_degree := case '+' '.join(f'{z}_node = {u} : degree_{u};' for u in range(config['rows']*config['columns']))+' TRUE : 0; esac;',
                'INIT '+either([f'{z} = {j} & {owners[j]["active"]}' for j in owners])+';']
    for name,kind in config.get('roles',{}).items():
        if kind not in ('N','P','any'): raise ValueError('role kind must be N, P or any')
        if kind!='any': out.append(f'INIT {"!" if kind=="N" else ""}role_{name}_parking;')
    for i in range(config['cars']):
        out += [f'INIT goal_{i}_parking;']
        kind=config.get('start_types',['N']*config['cars'])[i]
        if kind!='any': out.append(f'INIT {"!" if kind=="N" else ""}start_{i}_parking;')
        for prefix,key in (('start','starts'),('goal','goals')):
            if key in config: out.append(f'INIT {prefix}_{i} = {config[key][i]};')
        for j in range(i):
            if config.get('distinct_starts',True): out.append(f'INIT start_{i} != start_{j};')
            if config.get('distinct_goals',True): out.append(f'INIT goal_{i} != goal_{j};')
    out += ['INIT '+placement(t,config)+';' for t in config.get('placements',[])]
    return out


def junction_observations(config):
    roads,joints=inventory(config); out=[]
    for i in range(config['cars']):
        cases=[f'road_{i} = {r["id"]} : {joints[r["joints"][0]]["node"]};'
               for r in roads if r['kind']=='connection']
        cases += [f'joint_{i} = {j} : {x["node"]};' for j,x in enumerate(joints)]
        out.append(f'DEFINE junction_node_{i} := case '+' '.join(cases)+' TRUE : -1; esac;')
    return out


def safety_observations(config, geometry, symbolic=False):
    """Current velocity fixes the first displacement; then brake maximally.

    Clearance has the paper's same-road/dead-end meaning, not a swept-geometry
    safety certificate. The native environment independently checks collisions.
    """
    from vfm_grid_python.concrete.braking import effective_parameters
    from vfm_grid_python.concrete.observations import road_mapping
    p=effective_parameters(ROOT/'main_template.json',config.get('vfm',{}))
    speed,body=p['MAXSPEEDNONEGO'],p['VEHICLE_LENGTH']
    braking=min(-p['MINACCELNONEGO'],p['MAXACCELNONEGO'])
    if braking<=0: raise ValueError('Safe requires both braking directions')
    mapping=road_mapping(config,geometry,symbolic); roads,joints=inventory(config)
    maximum=(speed+1)*speed+body+1
    out=junction_observations(config)
    for i in range(config['cars']):
        car=f'env.veh___6{i}9___'
        out += [f'DEFINE paper_step_{i} := {car}.v;',
                f'DEFINE paper_stop_distance_{i} := case '+ ' '.join(
                    f'abs(paper_step_{i}) = {w} : {sum(range(w,0,-braking))};' for w in range(speed+1))+' TRUE : 0; esac;']
        cases=[]
        for r,item in mapping.items():
            flip=item['flipped']
            orientation=(f'(case {flip} : -paper_step_{i}; TRUE : paper_step_{i}; esac)'
                         if isinstance(flip,str) else f'-paper_step_{i}' if flip else f'paper_step_{i}')
            cases.append(f'road_{i} = {r} : {orientation};')
        out.append(f'DEFINE paper_direction_{i} := case '+' '.join(cases)+' TRUE : 0; esac;')
        clearance=str(maximum)
        clearance_terms=[]
        for j in range(config['cars']):
            if j==i: continue
            ahead=f'((paper_direction_{i} > 0 & qpos_{j} > qpos_{i}) | (paper_direction_{i} < 0 & qpos_{j} < qpos_{i}))'
            d=f'(case road_{j} = road_{i} & {ahead} : abs(qpos_{j} - qpos_{i}) - {body}; TRUE : {maximum}; esac)'
            clearance_terms.append(d)
        for r,item in mapping.items():
            if item['kind']!='section': continue
            for side,joint in enumerate(item['joints']):
                u=joints[joint]['node']
                guard=f'road_{i} = {r} & degree_{u} = 1 & paper_direction_{i} {"<" if side==0 else ">"} 0'
                d=f'qpos_{i}' if side==0 else f'({item["length"]} - qpos_{i})'
                clearance_terms.append(f'(case {guard} : {d}; TRUE : {maximum}; esac)')
        # nuXmv lowers min(a,b) to a case mentioning a twice. Nesting the
        # complete previous expression therefore doubles its printed size at
        # every step. Named DEFINEs keep the identical recurrence linear.
        for k, term in enumerate(clearance_terms):
            name=f'paper_clearance_{i}_step_{k}'
            out.append(f'DEFINE {name} := min({clearance}, {term});')
            clearance=name
        out += [f'DEFINE paper_clearance_{i} := {clearance};',
                f'DEFINE paper_safe_{i} := paper_stop_distance_{i} <= paper_clearance_{i};']
    return out


def validate(config):
    if config.get('initial_constraints') or config.get('constraints'):
        # Enumerator placement INITs are added internally after normalization.
        raise ValueError('paper inputs use placements, not raw legacy SMV constraints')
    for z in config.get('roles',{}):
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',z): raise ValueError('invalid role name')
    declared=names(config)
    aliases=[z+suffix for z in declared for suffix in ('_node','_parking','_degree')]
    if set(declared) & set(aliases):
        raise ValueError('role name collides with a generated joint observation')
    starts=config.get('start_types',['N']*config['cars'])
    if len(starts)!=config['cars'] or any(kind not in ('N','P','any') for kind in starts):
        raise ValueError('start_types needs N, P or any for each car')
    joint_count=len(inventory(config)[1])
    for field in ('starts','goals'):
        if field in config and (len(config[field])!=config['cars'] or any(
                type(j) is not int or not 0<=j<joint_count for j in config[field])):
            raise ValueError(f'{field} needs one valid joint ID per car (not grid-node IDs)')
    for term in config.get('placements',[]): placement(term,config)
    for stage in config['scenario_stages']:
        atom(stage['event'],config)
        holding_joints(stage.get('hold',True))
        atom(stage.get('hold',True),config)
