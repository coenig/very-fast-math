"""Shared road/rank/joint observations and safety for all concrete approaches."""
from itertools import combinations
from vfm_grid_python.scenarios.roads import inventory
from .symbolic import edge_name, reverse, connector_active


def road_mapping(config, geometry, symbolic=False):
    roads, joints = inventory(config)
    sections = {frozenset(s['edge']): s for s in geometry['sections']}
    result = {}
    for road in roads:
        if road['kind'] == 'section':
            s = sections.get(frozenset(road['edge']))
            if s:
                result[road['id']] = dict(kind='section', id=s['id'],
                    length=f'env.section_{s["id"]}_end' if symbolic else s['length'],
                    flipped=reverse(s,'env.') if symbolic else s['edge'] != road['edge'],
                    active=edge_name(s,'env.') if symbolic else 'TRUE', joints=road['joints'])
        else:
            a,b = [sections.get(frozenset((joints[j]['node'],joints[j]['neighbour']))) for j in road['joints']]
            if a and b:
                c = next((c for c in geometry['connections'] if {c['from'],c['to']} == {a['id'],b['id']}),None)
                if c:
                    result[road['id']] = dict(kind='connection',id=c['id'],source=c['from'],target=c['to'],
                        length=c['length'],flipped=c['from'] != a['id'],joints=road['joints'],
                        active=connector_active(c,geometry,'env.') if symbolic else 'TRUE')
    return result


def occupied(car, item):
    p = f'env.veh___6{car}9___'
    on = (f'{p}.on_straight_section = {item["id"]}' if item['kind']=='section' else
          f'{p}.on_straight_section = -1 & {p}.traversion_from = {item["source"]} & {p}.traversion_to = {item["target"]}')
    return f'({item["active"]}) & ({on})'


def observations(config, graph, geometry, symbolic=False):
    mapping=road_mapping(config,geometry,symbolic)
    _,joints=inventory(config)
    lines=['-- Shared queue observations; no whole-road reservations.', 'DEFINE']
    for prefix in ('start','goal'):
        lines += [f' {prefix}_{i} := {v};' for i,v in enumerate(graph[prefix+'s'])]
    lines += [f' role_{name} := {v};' for name,v in graph['roles'].items()]
    for i in range(config['cars']):
        p=f'env.veh___6{i}9___'
        lines += [f' road_{i} := case']+[f' {occupied(i,m)} : {r};' for r,m in mapping.items()]
        lines += [' TRUE : -1; esac;',f' qpos_{i} := case']
        for r,m in mapping.items():
            pos=f'{p}.abs_pos'; rev=f'{m["length"]} - {pos}'
            if isinstance(m['flipped'],str): pos=f'case {m["flipped"]} : {rev}; TRUE : {pos}; esac'
            elif m['flipped']: pos=rev
            lines.append(f' road_{i} = {r} : {pos};')
        lines += [' TRUE : -1; esac;',f' loc_{i} := case']
        for r,m in mapping.items():
            if m['kind']!='section': continue
            for j,pos in zip(m['joints'],(0,m['length'])):
                node=joints[j]['node']
                stop=(f' & (!env.parking_{node} | {p}.v = 0)' if symbolic else
                      f' & {p}.v = 0' if graph['nodes'][node]['kind']=='P' else '')
                lines.append(f' road_{i} = {r} & qpos_{i} = {pos}{stop} : {node};')
        lines += [' TRUE : -1; esac;',f' joint_{i} := case']
        for r,m in mapping.items():
            for j,pos in zip(m['joints'],(0,m['length'])):
                lines.append(f' road_{i} = {r} & qpos_{i} = {pos} : {j};')
        lines += [' TRUE : -1; esac;']
        terms=[f'(case road_{j} = road_{i} & qpos_{j} < qpos_{i} : 1; TRUE : 0; esac)' for j in range(config['cars']) if j!=i]
        lines += [f' rank_{i} := '+(' + '.join(terms) or '0')+';']
    return lines


def fleet_safety(config):
    lines=[]
    for i,j in combinations(range(config['cars']),2):
        lines += [f'INVAR road_{i} = road_{j} -> abs(qpos_{i} - qpos_{j}) >= {config.get("queue_clearance",6)};',
                  f'INVAR joint_{i} >= 0 -> joint_{i} != joint_{j};',
                  f'TRANS !(road_{i} != road_{j} & next(road_{i}) = road_{j} & next(road_{j}) = road_{i});']
    return lines
