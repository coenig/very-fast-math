"""Stopped terminal targets for rolling, cell-based refinement queries.

The ``stopped`` mode leaves positions free within the milestone cells. The
stronger ``canonical-stopped`` experiment uses fixed representatives. The
appendix uses real canonical points. VFM uses integer positions: select
nearest integer representatives (ties upwards) and explicitly check their cell
membership and clearance. This is an opt-in terminal condition, not a change
to the abstraction or a claim that the real-valued theorem has been verified
for every VFM constraint. No rounding is hidden in the experiment metadata.
"""
from fractions import Fraction
from itertools import combinations

from vfm_grid_python.common import both
from vfm_grid_python.concrete.observations import road_mapping
from .paper_cells import CellDomain, concrete_bounds, dimensions, milestone


class UnsupportedStoppedTarget(ValueError):
    """The chosen stopped reference is not valid in the concrete model."""


def representatives(kind, length, body, *, epsilon=Fraction(1)):
    domain = CellDomain(length, body)
    if kind not in ('section', 'connection'):
        raise ValueError(f'unsupported road kind: {kind}')
    epsilon = epsilon if kind == 'connection' else 0
    exact = [Fraction(epsilon) + Fraction(k * (length - 2 * epsilon), domain.count - 1)
             for k in range(domain.count)]
    positions = [(2 * p.numerator + p.denominator) // (2 * p.denominator) for p in exact]
    for k, position in enumerate(positions):
        lo, hi = concrete_bounds(kind, length, body, k)
        if not lo <= position <= hi:
            raise UnsupportedStoppedTarget(f'{kind} length {length}, cell {k}: integer canonical '
                             f'position {position} is outside [{lo}, {hi}]')
    return positions, exact


def build_targets(frames, config, geometry, *, mode='canonical-stopped'):
    """Return predicates plus an auditable description of the terminal states.

    Every predicate includes the road/cell assignment, queue ranks, property
    progress and zero velocities. Canonical mode also fixes representative
    positions. Connections are allowed. All original dynamics remain in place.
    """
    if mode not in ('stopped', 'canonical-stopped'):
        raise ValueError(mode)
    body, _ = dimensions(config)
    gap = max(body + 1, config.get('queue_clearance', body + 1))
    mapping = road_mapping(config, geometry)
    stopped = [f'env.veh___6{i}9___.v = 0' for i in range(config['cars'])]
    if mode == 'stopped':
        return [both([milestone(frame, config, mapping), *stopped]) for frame in frames], dict(
            mode=mode, position_policy='free_within_milestone_cells',
            initial_target_enforced=True, guaranteed_greedy_refinement=False,
            note='A feasible stopped continuation is witnessed, but no canonical '
                 'placement or universally extendible stopped-state set is certified.')
    # Meet the appendix's strict connection-spacing condition before rounding.
    # In particular, length 12 / body 5 has three cells and needs epsilon < 1.
    epsilon = min([Fraction(1)] + [
        Fraction(m['length'] - (CellDomain(m['length'], body).count - 1) * body, 3)
        for m in mapping.values() if m['kind'] == 'connection'])
    cache, predicates, targets = {}, [], []
    for k, frame in enumerate(frames):
        cars, terms = [], [milestone(frame, config, mapping)]
        for i in range(config['cars']):
            r, cell = int(frame[f'road_{i}']), int(frame[f'cell_{i}'])
            item = mapping[r]
            if r not in cache:
                cache[r] = representatives(item['kind'], item['length'], body, epsilon=epsilon)
            positions, exact = cache[r]
            position = positions[cell]
            terms += [f'qpos_{i} = {position}', f'env.veh___6{i}9___.v = 0']
            cars.append(dict(car=i, road=r, cell=cell, kind=item['kind'],
                             position=position, real_canonical=str(exact[cell]),
                             rounded=Fraction(position) != exact[cell]))
        # Same-road clearance and the shared concrete model's joint clearance.
        # Reject unsupported targets; never remove native collision constraints.
        for a, b in combinations(cars, 2):
            ma, mb = mapping[a['road']], mapping[b['road']]
            if a['road'] == b['road'] and abs(a['position'] - b['position']) < gap:
                raise UnsupportedStoppedTarget(f'milestone {k}: stopped cars {a["car"]}, '
                                 f'{b["car"]} violate same-road clearance')
            for joint in set(ma['joints']) & set(mb['joints']):
                da = a['position'] if ma['joints'][0] == joint else ma['length'] - a['position']
                db = b['position'] if mb['joints'][0] == joint else mb['length'] - b['position']
                if da + db <= body:
                    raise UnsupportedStoppedTarget(f'milestone {k}: canonical stopped cars {a["car"]}, '
                                     f'{b["car"]} have clearance {da + db} at joint '
                                     f'{joint}, requiring > {body}; target unsupported')
        predicates.append(both(terms))
        targets.append(dict(milestone=k, scenario_phase=int(frame['scenario_phase']), cars=cars))
    return predicates, dict(
        mode=mode, integer_policy='nearest_integer_ties_up',
        real_connection_epsilon=str(epsilon), initial_target_enforced=True,
        targets=targets, static_clearance_checked=True,
        guaranteed_greedy_refinement=False,
        note='Each query witnesses a stopped terminal continuation; commits remain unrestricted. '
             'Integer representatives and the actual VFM transition relation require a separate '
             'extension argument before asserting the appendix theorem for this implementation.')
