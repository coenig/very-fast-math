"""Paper-style neighbour cells on the shared symbolic road inventory.

This implements the paper's cell transition system, not its entire theorem:
experiments keep the common event/hold observer and integer VFM dynamics.
Concrete milestones use cell regions, with endpoint observations restored on
sections. They do NOT force all cars to stop at every milestone. In particular,
two-goal sequentialization is still experimental, not guaranteed backtrack-free.
"""
from dataclasses import dataclass
from fractions import Fraction
from itertools import combinations
import re

from vfm_grid_python.common import both
from vfm_grid_python.scenarios.roads import inventory
from vfm_grid_python.geometry import build
from vfm_grid_python.concrete.symbolic import skeleton
from vfm_grid_python.concrete.observations import road_mapping
from . import abstract


@dataclass(frozen=True)
class CellDomain:
    length: int
    car_length: int

    def __post_init__(self):
        if (type(self.length) is not int or type(self.car_length) is not int
                or self.car_length < 1 or self.length <= self.car_length):
            raise ValueError('paper cells require road length > positive car length')

    @property
    def count(self):
        return (self.length + self.car_length - 1) // self.car_length

    @property
    def width(self):
        return Fraction(self.length, self.count)

    def bounds(self, cell):
        """Integer members of [k L/n,(k+1)L/n), closing the last cell."""
        if not 0 <= cell < self.count:
            raise ValueError('cell outside road')
        lo = (cell * self.length + self.count - 1) // self.count
        hi = (self.length if cell == self.count - 1 else
              ((cell + 1) * self.length + self.count - 1) // self.count - 1)
        return lo, hi

    def canonical(self, cell):
        if not 0 <= cell < self.count:
            raise ValueError('cell outside road')
        return Fraction(cell * self.length, self.count - 1)


def dimensions(config):
    """Resolve car length from the same defaults/overrides as VFM."""
    from vfm_grid_python.common import ROOT
    from vfm_grid_python.concrete.braking import effective_parameters
    parameters = effective_parameters(ROOT/'main_template.json', config.get('vfm', {}))
    return parameters['VEHICLE_LENGTH'], parameters['MAXSPEEDNONEGO']


def road_lengths(config):
    geometry = build(skeleton(config), config['section_length'],
                     config['parking_length'], config['junction_radius'])
    return road_mapping(config, geometry)


def assumptions(config):
    body, speed = dimensions(config)
    lengths = {config['section_length'], config['parking_length']}
    lengths.update(m['length'] for m in road_lengths(config).values()
                   if m['kind'] == 'connection')
    widths = [CellDomain(length, body).width for length in sorted(lengths)]
    width = min(widths)
    return dict(car_length=body, max_speed=speed, minimum_cell_width=str(width),
                paper_small_step_assumption=Fraction(speed) < width,
                milestone_clearance_filter=config.get('paper_cell_clearance', True),
                guaranteed_greedy_refinement=False,
                note='Shared VFM dynamics, integer coordinates and event/hold observer; '
                     'not a validation of the full paper theorem.')


def concrete_bounds(kind, length, body, cell):
    """The actual region used by a concrete milestone, in road coordinates."""
    domain = CellDomain(length, body)
    lo, hi = domain.bounds(cell)
    if kind == 'section':
        if cell == 0:
            return 0, 0
        if cell == domain.count - 1:
            return length, length
    else:
        lo, hi = max(1, lo), min(length - 1, hi)
    return lo, hi


def clearance_regions(config):
    """Candidate regions with guards for symbolic normal/parking lengths.

    This table does not choose the topology. A guard selects the same length
    as length_r in the abstract model and in the concrete road mapping.
    """
    roads, _ = inventory(config)
    mapping = road_lengths(config)
    body, _ = dimensions(config)
    regions = []
    for road in roads:
        if road['kind'] == 'section':
            u, v = road['edge']
            parked = f'(parking_{u} | parking_{v})'
            lengths = [(config['section_length'], f'!{parked}'),
                       (config['parking_length'], parked)]
            if config['section_length'] == config['parking_length']:
                lengths = [(config['section_length'], 'TRUE')]
        else:
            lengths = [(mapping[road['id']]['length'], 'TRUE')]
        for length, guard in lengths:
            for cell in range(CellDomain(length, body).count):
                lo, hi = concrete_bounds(road['kind'], length, body, cell)
                regions.append(dict(road=road['id'], cell=cell, guard=guard,
                                    lo=lo, hi=hi, length=length,
                                    joints=road['joints']))
    return regions


def pair_clearance_constraints(config):
    """Necessary static feasibility checks, not a concretizability theorem.

    boundary_separation requires d_i(j)+d_k(j)>vehicle_length at every joint.
    If even the largest distances allowed by two cells fail that inequality,
    that abstract pair has no concrete realization. Boolean threshold sets
    share this check without a large selected-distance arithmetic lookup.
    Same-road pairs likewise need room for queue_clearance. These tests are
    existential/pairwise: they do not forbid all pairs whose regions overlap,
    nor guarantee simultaneous feasibility of three or more cars/dynamics.
    """
    if not config.get('paper_cell_clearance', True):
        return []
    regions = clearance_regions(config)
    body, _ = dimensions(config)
    gap = config.get('queue_clearance', 6)
    flat = config.get('paper_cell_encoding', 'baseline') not in ('baseline', 'folded_counts')
    if flat:
        from .cell_encoding import layout
        ids = {rc: p for p, rc in enumerate(layout(config)[2])}

    def member(region, car):
        pos = (f'position_{car} = {ids[region["road"], region["cell"]]}' if flat else
               f'road_{car} = {region["road"]} & cell_{car} = {region["cell"]}')
        return '(' + pos + (f' & {region["guard"]}' if region['guard'] != 'TRUE' else '') + ')'

    lines = ['-- Necessary clearance of concrete milestone regions.']
    near = {}
    for region in regions:
        if region['lo'] > region['hi']:
            lines += [f'INVAR !{member(region, i)};' for i in range(config['cars'])]
            continue
        for joint, maximum in zip(region['joints'], (region['hi'], region['length']-region['lo'])):
            if maximum <= body:
                near.setdefault(joint, {}).setdefault(maximum, []).append(region)
    for joint, groups in sorted(near.items()):
        distances = sorted(groups)
        for d in distances:
            for i in range(config['cars']):
                terms = [member(r, i) for r in groups[d]]
                lines.append(f'DEFINE cell_near_{joint}_{d}_{i} := ' + ' | '.join(terms) + ';')
        for i, j in combinations(range(config['cars']), 2):
            for d in distances:
                for e in distances:
                    if d + e <= body:
                        lines.append(f'INVAR !(cell_near_{joint}_{d}_{i} & cell_near_{joint}_{e}_{j});')
    by_road = {}
    for r in regions:
        by_road.setdefault(r['road'], []).append(r)
    for candidates in by_road.values():
        for a, b in combinations(candidates, 2):
            if (a['guard'] != b['guard'] or a['cell'] == b['cell'] or
                    max(b['hi'] - a['lo'], a['hi'] - b['lo']) >= gap):
                continue
            for i, j in combinations(range(config['cars']), 2):
                lines += [f'INVAR !({member(a, i)} & {member(b, j)});',
                          f'INVAR !({member(b, i)} & {member(a, j)});']
    return lines


def clearance_constraints(config):
    """Existential point placement for all cars, with no dynamic constraints.

    Pairwise upper bounds alone miss conflicts where one car needs incompatible
    positions to clear two others. cell_point_i is an independent spatial
    witness at each abstract tick; it is not fixed by the concrete milestone
    and has no next-state equation. Its sole purpose is joint feasibility of
    the existing regions and concrete same-road/boundary separation rules.
    """
    if not config.get('paper_cell_clearance', True):
        return []
    regions = clearance_regions(config)
    roads, joints = inventory(config)
    body, _ = dimensions(config)
    gap = config.get('queue_clearance', 6)
    flat = config.get('paper_cell_encoding', 'baseline') not in ('baseline', 'folded_counts')
    if flat:
        from .cell_encoding import layout
        positions = layout(config)[2]
        ids = {rc: p for p, rc in enumerate(positions)}
        spans = {r['id']: [p for p, (road, _) in enumerate(positions) if road==r['id']]
                 for r in roads}
    maximum = max(r['length'] for r in regions)
    lines = pair_clearance_constraints(config)
    for i in range(config['cars']):
        point = f'cell_point_{i}'
        lines.append(f'VAR {point} : 0..{maximum};')
        for road in roads:
            r = road['id']
            on = (f'position_{i} >= {spans[r][0]} & position_{i} <= {spans[r][-1]}'
                  if flat else f'road_{i} = {r}')
            lines.append(f'DEFINE cell_on_{r}_{i} := {on};')
        for region in regions:
            r, c = region['road'], region['cell']
            on = (f'position_{i} = {ids[r,c]}' if flat else f'road_{i} = {r} & cell_{i} = {c}')
            lines.append(f'INVAR ({on} & {region["guard"]}) -> '
                         f'({point} >= {region["lo"]} & {point} <= {region["hi"]});')
        for joint in range(len(joints)):
            terms = []
            for road in roads:
                if joint not in road['joints']:
                    continue
                r = road['id']
                for region in (x for x in regions if x['road']==r and x['cell']==0):
                    distance = point if road['joints'][0]==joint else f'{region["length"]} - {point}'
                    terms.append(f'cell_on_{r}_{i} & {region["guard"]} : {distance};')
            lines.append(f'DEFINE cell_distance_{joint}_{i} := case ' + ' '.join(terms)
                         + f' TRUE : {body+1}; esac;')
    for i, j in combinations(range(config['cars']), 2):
        for road in roads:
            r = road['id']
            lines.append(f'INVAR (cell_on_{r}_{i} & cell_on_{r}_{j}) -> '
                         f'abs(cell_point_{i} - cell_point_{j}) >= {gap};')
        for joint in range(len(joints)):
            lines.append(f'INVAR cell_distance_{joint}_{i} + cell_distance_{joint}_{j} > {body};')
    return lines


def render(config):
    roads, _ = inventory(config)
    body, _ = dimensions(config)
    mapping = road_lengths(config)
    # Validate all candidate lengths, including symbolic parking sections.
    assumptions(config)
    text = abstract.render(config, motion_profile='paper_cells')
    for r, item in mapping.items():
        if item['kind'] == 'connection':
            text = re.sub(rf' length_{r} := \d+;',
                          f' length_{r} := {item["length"]};', text)
    maximum = max(config['section_length'], config['parking_length'],
                  *(m['length'] for m in mapping.values()))
    largest = CellDomain(maximum, body).count
    extra = ['-- Paper neighbour cells; rank and tip are observations, not state.',
             'CONSTANTS left, interior, right;']
    for i in range(config['cars']):
        extra += [f'VAR cell_{i} : 0..{largest-1};',
                  f'DEFINE ilen_{i} := case ' + ' '.join(
                      f'road_{i} = {r["id"]} : length_{r["id"]};' for r in roads) + ' TRUE : 1; esac;',
                  f'DEFINE ncells_{i} := (ilen_{i} + {body-1}) / {body};',
                  f'INVAR cell_{i} < ncells_{i};',
                  f'DEFINE tip_{i} := case cell_{i} = 0 : left; '
                  f'cell_{i} = ncells_{i} - 1 : right; TRUE : interior; esac;',
                  f'DEFINE rank_{i} := ' + (' + '.join(
                      f'(case road_{j} = road_{i} & cell_{j} < cell_{i} : 1; TRUE : 0; esac)'
                      for j in range(config['cars']) if i != j) or '0') + ';']
        # Existing transfer builder already links opposite road kinds through
        # the same joint. tip now means endpoint-touching CELL, not exact point.
        transfer = abstract.transfer_constraint(roads, i, 'joint')
        transfer = transfer.replace(f'(next(road_{i}) = road_{i})',
            f'(next(road_{i}) = road_{i} & abs(next(cell_{i}) - cell_{i}) <= 1)')
        extra.append(transfer)
    for i in range(config['cars']):
        for j in range(i):
            extra += [
                f'INVAR !(road_{i} = road_{j} & cell_{i} = cell_{j});',
                f'TRANS !((next(road_{i}) = road_{j} & next(cell_{i}) = cell_{j}) & '
                f'(next(road_{j}) = road_{i} & next(cell_{j}) = cell_{i}));',
                # Oriented pairs implement both implications in Phi_{i,j}.
                f'TRANS (next(road_{i}) != road_{i} & next(road_{j}) != road_{j}) -> '
                f'!((next(road_{i}) = road_{j} & next(cell_{i}) = cell_{j}) | '
                f'(next(road_{j}) = road_{i} & next(cell_{j}) = cell_{i}));']
    text = text + '\n' + '\n'.join(extra) + '\n'
    encoding = config.get('paper_cell_encoding', 'baseline')
    if encoding != 'baseline':
        from .cell_encoding import rewrite
        text = rewrite(text, config, encoding)
    text += '\n' + '\n'.join(clearance_constraints(config)) + '\n'
    return text


def milestone(frame, config, mapping):
    body, _ = dimensions(config)
    terms = []
    for i in range(config['cars']):
        r, cell = int(frame[f'road_{i}']), int(frame[f'cell_{i}'])
        item = mapping[r]
        lo, hi = concrete_bounds(item['kind'], item['length'], body, cell)
        if lo > hi:
            raise ValueError(f'road {r}, cell {cell} has no integer realization')
        terms += [f'road_{i} = {r}', f'qpos_{i} >= {lo}', f'qpos_{i} <= {hi}',
                  f'rank_{i} = {frame[f"rank_{i}"]}']
    terms.append(f'scenario_phase = {frame["scenario_phase"]}')
    return both(terms)


def validate_trace(frames, config):
    """Independent movement rules and necessary milestone-clearance checks."""
    roads, _ = inventory(config)
    mapping = road_lengths(config)
    body, _ = dimensions(config)
    errors = []
    previous = None
    def domain(r, frame):
        road = roads[r]
        length = mapping[r]['length']
        if road['kind'] == 'section':
            length = (config['parking_length'] if any(
                frame[f'parking_{n}'] == 'TRUE' for n in road['edge'])
                else config['section_length'])
        return CellDomain(length, body)

    def endpoint(r, cell, frame):
        count = domain(r, frame).count
        if cell == 0:
            return roads[r]['joints'][0]
        if cell == count - 1:
            return roads[r]['joints'][1]
        return -1

    for tick, frame in enumerate(frames):
        current = [(int(frame[f'road_{i}']), int(frame[f'cell_{i}']))
                   for i in range(config['cars'])]
        if len(set(current)) != len(current):
            errors.append(f'tick {tick}: duplicate cell')
        for r, cell in current:
            region_domain = domain(r, frame)
            if not 0 <= cell < region_domain.count:
                errors.append(f'tick {tick}: cell outside road')
            elif config.get('paper_cell_clearance', True):
                lo, hi = concrete_bounds(roads[r]['kind'], region_domain.length, body, cell)
                if lo > hi:
                    errors.append(f'tick {tick}: cell has no integer realization')
        if config.get('paper_cell_clearance', True):
            has_points = any(f'cell_point_{i}' in frame for i in range(len(current)))
            points = {}
            if has_points:
                for i, (r, c) in enumerate(current):
                    key = f'cell_point_{i}'
                    if key not in frame:
                        errors.append(f'tick {tick}: missing packing witness for car {i}')
                        continue
                    points[i] = int(frame[key])
                    d = domain(r, frame)
                    if 0 <= c < d.count:
                        lo, hi = concrete_bounds(roads[r]['kind'], d.length, body, c)
                        if not lo <= points[i] <= hi:
                            errors.append(f'tick {tick}: packing witness outside cell for car {i}')
            for i, j in combinations(range(len(current)), 2):
                r, c = current[i]; s, d = current[j]
                dr, ds = domain(r, frame), domain(s, frame)
                if not (0 <= c < dr.count and 0 <= d < ds.count):
                    continue
                lo, hi = concrete_bounds(roads[r]['kind'], dr.length, body, c)
                low, high = concrete_bounds(roads[s]['kind'], ds.length, body, d)
                if i in points and j in points:
                    lo = hi = points[i]
                    low = high = points[j]
                if r == s and max(high-lo, hi-low) < config.get('queue_clearance', 6):
                    errors.append(f'tick {tick}: cars {i},{j} lack same-road clearance')
                for joint in set(roads[r]['joints']) & set(roads[s]['joints']):
                    a = hi if roads[r]['joints'][0] == joint else dr.length-lo
                    b = high if roads[s]['joints'][0] == joint else ds.length-low
                    if a+b <= body:
                        errors.append(f'tick {tick}: cars {i},{j} lack joint clearance at {joint}')
        if previous is not None:
            for (r, c), (s, d) in zip(previous, current):
                if r == s:
                    if abs(c-d) > 1:
                        errors.append(f'tick {tick}: non-neighbour move')
                else:
                    if (roads[r]['kind'] == roads[s]['kind'] or
                            endpoint(r, c, old) != endpoint(s, d, frame) or
                            endpoint(r, c, old) < 0):
                        errors.append(f'tick {tick}: invalid road transfer')
            for i in range(len(current)):
                for j in range(i):
                    if current[i] == previous[j] and current[j] == previous[i]:
                        errors.append(f'tick {tick}: swap')
                    if all(previous[k][0] != current[k][0] for k in (i,j)) and (
                            current[i] == previous[j] or current[j] == previous[i]):
                        errors.append(f'tick {tick}: joint crossing conflict')
        previous, old = current, frame
    return errors
