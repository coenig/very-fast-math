"""Optional-edge endpoint compiler shared by symbolic and fixed enumeration.

Each candidate grid edge has one reversible section slot. Frozen topology
flags select active slots and N/P labels; inactive slots cannot hold a car.
Every junction curve is precomputed in the regular-node frame. Converting the
far end of an edge into a parking leaf does not move its regular-node endpoint.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import sys
import time

from vfm_grid_python.common import ROOT, both, either, grid_edges, dump, execute, check, states
from vfm_grid_python.geometry import build
from vfm_grid_python.concrete.dynamics import movement


def skeleton(config):
    return dict(rows=config['rows'],columns=config['columns'],
                nodes=[dict(id=i,row=i//config['columns'],column=i%config['columns'],kind='N')
                       for i in range(config['rows']*config['columns'])],
                edges=[list(e) for e in grid_edges(config['rows'],config['columns'])],
                milestones=[[0]*config['cars']])


def edge_name(section, prefix=''):
    u,v=sorted(section['edge'])
    return f'{prefix}edge_{u}_{v}'


def reverse(section,prefix=''):
    return f"({edge_name(section,prefix)} & {prefix}parking_{section['edge'][0]})"


def at_end(section,node,prefix=''):
    expr=reverse(section,prefix)
    return expr if node==section['edge'][0] else f'!{expr}'


def connector_active(c,geometry,prefix=''):
    return both([edge_name(geometry['sections'][i],prefix) for i in (c['from'],c['to'])]+
                [f"!{prefix}parking_{c['node']}"])


def symbolic_movement(geometry,cars):
    g=deepcopy(geometry)
    # A numeric sentinel avoids a very deep nested max expression in the parser.
    g['maximum_element_length'] = max([geometry['road_length'],geometry['parking_length']] + [c['length'] for c in geometry['connections']])
    for s in g['sections']:
        s['length']=f"section_{s['id']}_end"
        s['active']=edge_name(s)
    for c in g['connections']:
        c['active']=connector_active(c,g)
        c['from_end']=at_end(g['sections'][c['from']],c['node'])
        c['to_end']=at_end(g['sections'][c['to']],c['node'])
    return movement(g,cars)


def transform_environment(text,geometry):
    """Replace fixed topology metadata before flattening can constant-fold it."""
    lines=['FROZENVAR']
    lines += [f"{edge_name(s)} : boolean;" for s in geometry['sections']]
    lines += [f"parking_{n['id']} : boolean;" for n in geometry['nodes']]
    def domain(name,value):
        nonlocal text
        pattern=r'(?m)(\b'+re.escape(name)+r'\s*:\s*)-?\d+\s*\.\.\s*-?\d+\s*;'
        text,count=re.subn(pattern,lambda m:m[1]+value+';',text)
        if count!=1: raise ValueError(f'cannot replace symbolic domain {name}: {count}')
    def define(name,value):
        nonlocal text
        pattern=r'(?m)(\b'+re.escape(name)+r'\s*:=\s*)[^;]+;'
        text,count=re.subn(pattern,lambda m:m[1]+value+';',text)
        if count!=1: raise ValueError(f'cannot replace symbolic definition {name}: {count}')
    road,parking=geometry['road_length'],geometry['parking_length']
    for s in geometry['sections']:
        i=s['id']; u,v=s['edge']; rev=reverse(s)
        length=f"case {edge_name(s)} & (parking_{u} | parking_{v}) : {parking}; TRUE : {road}; esac"
        domain(f'section_{i}_end',f'{min(road,parking)} .. {max(road,parking)}')
        lines += [f'INIT section_{i}_end = ({length});']
        angle=s['angle']; dx=round(math.cos(math.radians(angle))); dy=round(math.sin(math.radians(angle)))
        for axis,k,d in (('x',0,dx),('y',1,dy)):
            a,b=s['source'][k],s['drain'][k]
            domain(f'section_{i}.source.{axis}',f'{min(a,b)} .. {max(a,b)}')
            lines += [f'INIT section_{i}.source.{axis} = (case {rev} : {b}; TRUE : {a}; esac);']
            drain=f'case {rev} : {b-parking*d}; {edge_name(s)} & parking_{v} : {a+parking*d}; TRUE : {b}; esac'
            define(f'section_{i}.drain.{axis}',drain)
        new_angle=(angle+180)%360
        domain(f'section_{i}.angle_raw',f'{min(angle,new_angle)//45} .. {max(angle,new_angle)//45}')
        lines += [f'INIT section_{i}.angle_raw = (case {rev} : {new_angle//45}; TRUE : {angle//45}; esac);']
        define(f'sin_of_section_{i}_angle',f'case {rev} : {-dy*100}; TRUE : {dy*100}; esac')
        define(f'cos_of_section_{i}_angle',f'case {rev} : {-dx*100}; TRUE : {dx*100}; esac')
    outgoing={s['id']:[] for s in geometry['sections']}
    for c in geometry['connections']: outgoing[c['from']].append(c)
    for s in geometry['sections']:
        for slot,c in enumerate(outgoing[s['id']]):
            name=f"outgoing_connection_{slot}_of_section_{s['id']}"
            domain(name,f"-1 .. {len(geometry['sections'])-1}")
            text,count=re.subn(r'(?m)^INIT '+name+r' = -?\d+;',
                               f"INIT {name} = (case {connector_active(c,geometry)} : {c['to']}; TRUE : -1; esac);",text)
            if count!=1: raise ValueError(f'cannot guard connector {name}')
    return text+'\n'+'\n'.join(lines)+'\n'

def validate_specialization(active,graph):
    """Check selected geometry against the compact sequential geometry builder.

    IDs and the root's rigid coordinate frame may differ. Physical section
    directions, endpoint coordinates and all connection curves must agree.
    """
    expected=build(graph,active['road_length'],active['parking_length'],active['junction_radius'])
    by_edge={frozenset(s['edge']):s for s in active['sections']}
    first=by_edge[frozenset(expected['sections'][0]['edge'])]
    angle=math.radians(first['angle']); co,si=round(math.cos(angle)),round(math.sin(angle))
    def transform(p):
        return [co*p[0]-si*p[1]+first['source'][0],si*p[0]+co*p[1]+first['source'][1]]
    mapping={}
    for s in expected['sections']:
        target=by_edge[frozenset(s['edge'])]; mapping[s['id']]=target['id']
        if s['edge']!=target['edge'] or s['length']!=target['length'] or s['kind']!=target['kind']:
            raise ValueError('symbolic/fixed section specialization mismatch')
        for name in ('source','drain'):
            if math.dist(transform(s[name]),target[name])>1e-6:
                raise ValueError('symbolic/fixed endpoint geometry mismatch')
        if (s['angle']+first['angle'])%360 != target['angle']:
            raise ValueError('symbolic/fixed section angle mismatch')
    actual={(c['from'],c['to']):c for c in active['connections']}
    if len(actual)!=len(expected['connections']): raise ValueError('connection count mismatch')
    for c in expected['connections']:
        target=actual[mapping[c['from']],mapping[c['to']]]
        if any(target[k]!=c[k] for k in ('from_end','to_end','length','node')):
            raise ValueError('symbolic/fixed connector specialization mismatch')
        if len(c['curve'])!=len(target['curve']) or any(math.dist(transform(p),q)>1e-6 for p,q in zip(c['curve'],target['curve'])):
            raise ValueError('symbolic/fixed connection curve mismatch')
    return dict(valid=True,sections=len(by_edge),connections=len(actual),
                checked='compact and symbolic geometry agree up to section IDs and a rigid transform')
