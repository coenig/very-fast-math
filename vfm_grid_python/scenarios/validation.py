"""Validate the common input before any approach starts expensive work."""
from copy import deepcopy
import re


def normalize(config):
    from vfm_grid_python.approaches.abstraction_based.abstract import render
    c=deepcopy(config)
    c.setdefault('profile','legacy')
    if c['profile'] not in ('legacy','paper'):
        raise ValueError('profile must be legacy or paper')
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
    if c['profile']=='paper' and not c.get('scenario_stages'):
        c['scenario_stages']=[dict(event=True),dict(event={'all':[
            {'parked':[i,f'goal_{i}']} for i in range(c['cars'])]})]
    if not c.get('scenario_stages'):
        c['scenario_stages']=[dict(event={'all':[{'at':[i,f'goal_{i}']} for i in range(c['cars'])]})]
    if 'parkings' not in c:
        raise ValueError('the common three-approach pipeline requires a parking count')
    for field in ('initial_constraints','constraints'):
        if any(re.search(r'\btip_\d+\b',expr) for expr in c.get(field,[])):
            raise ValueError('raw abstract tip variables are not shared semantics; use attachment predicates')
    if c['profile']=='paper':
        c.setdefault('paper_cell_clearance', True)
        if type(c['paper_cell_clearance']) is not bool:
            raise ValueError('paper_cell_clearance must be a boolean')
        from .paper import validate
        from vfm_grid_python.approaches.abstraction_based.paper_cells import render as paper_render
        validate(c)
        paper_render(c)
    else:
        render(c)
    return c
