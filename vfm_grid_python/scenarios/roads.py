"""Stable candidate road/joint IDs shared by all model compilers."""
from itertools import combinations
from vfm_grid_python.common import grid_edges

def inventory(config):
    edges = grid_edges(config['rows'], config['columns'])
    roads, joints = [], []
    for u, v in edges:
        pair = [len(joints), len(joints)+1]
        joints.extend([dict(node=u, neighbour=v), dict(node=v, neighbour=u)])
        roads.append(dict(id=len(roads), kind='section', joints=pair,
                          active=f'edge_{u}_{v}', edge=[u, v]))
    for node in range(config['rows'] * config['columns']):
        attached = [(r, j) for r in roads if r['kind'] == 'section'
                    for j in r['joints'] if joints[j]['node'] == node]
        for (a, ja), (b, jb) in combinations(attached, 2):
            roads.append(dict(id=len(roads), kind='connection', joints=[ja, jb],
                              active=f"({a['active']} & {b['active']})"))
    return roads, joints
