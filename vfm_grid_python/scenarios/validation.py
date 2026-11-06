"""Validate the common input before any approach starts expensive work."""
from copy import deepcopy
import re


def normalize(config):
    from vfm_grid_python.approaches.abstraction_based.abstract import render
    c=deepcopy(config)
    c.setdefault('queue_solver','sat')
    c.setdefault('queue_transfer_encoding','joint')
    c.setdefault('queue_capacity_encoding','arithmetic')
    c.setdefault('roles',{})
    c.setdefault('section_length',20)
    c.setdefault('parking_length',7)
    c.setdefault('junction_radius',6)
    c.setdefault('connection_length',9)
    c.setdefault('queue_clearance',6)
    c.setdefault('vfm',dict(MAXSPEEDNONEGO=5,LANES_MIN_SPEEDS='@(-5)@',LANES_MAX_SPEEDS='@(5)@'))
    if not c.get('scenario_stages'):
        c['scenario_stages']=[dict(event={'all':[{'at':[i,f'goal_{i}']} for i in range(c['cars'])]})]
    if 'parkings' not in c:
        raise ValueError('the common three-approach pipeline requires a parking count')
    for field in ('initial_constraints','constraints'):
        if any(re.search(r'\btip_\d+\b',expr) for expr in c.get(field,[])):
            raise ValueError('raw abstract tip variables are not shared semantics; use attachment predicates')
    render(c)
    return c
