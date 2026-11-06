"""Optional, solver-checked joint braking suffix at a sequential handoff.

This is a sufficient test, not a completeness claim: all cars apply symmetric
maximum braking together. Native dynamics, collision and route constraints stay
active. The suffix is evidence only and is not committed to the output trace.
"""
from .common import both


def rest(cars):
    return both([f'env.veh___6{c}9___.v = 0 & env.veh___6{c}9___.a = 0'
                 for c in range(cars)])


def encoding(cars, braking):
    """Choose the NEXT acceleration, preserving the boundary's committed v+a.

Using next(v) also handles coordinate/sign changes at connector boundaries.
The first transition may accelerate; subsequent ticks brake to zero.
"""
    if braking <= 0 or int(braking) != braking:
        raise ValueError('braking certificate requires a positive integer braking magnitude')
    b = int(braking)
    lines = []
    for car in range(cars):
        p = f'env.veh___6{car}9___'
        lines.append(f'TRANS lookahead_phase = 1 -> next({p}.a) = case\n'
                     f'next({p}.v) > {b} : -{b};\n'
                     f'next({p}.v) < -{b} : {b};\n'
                     f'TRUE : -next({p}.v); esac;\n')
    return ''.join(lines)


def validate(frames, cars, braking):
    """Audit the certificate phase and acceleration policy in a solver witness.

Collision/route feasibility is certified by the query, not reimplemented here.
"""
    cut = next(i for i, s in enumerate(frames) if s.get('lookahead_phase') == '1')
    end = next(i for i in range(cut + 1, len(frames))
               if frames[i].get('lookahead_phase') == '2')
    for i in range(cut, end):
        if frames[i].get('lookahead_phase') != '1':
            raise ValueError('noncontiguous braking certificate')
        for car in range(cars):
            p = f'env.veh___6{car}9___'
            v = int(frames[i + 1][p + '.v'])
            a = int(frames[i + 1][p + '.a'])
            expected = -max(-braking, min(braking, v))
            if a != expected:
                raise ValueError(f'braking certificate tick {i + 1}: car {car} has invalid acceleration')
    for car in range(cars):
        p = f'env.veh___6{car}9___'
        if any(int(frames[end][p + '.' + field]) != 0 for field in ('v', 'a')):
            raise ValueError(f'braking certificate: car {car} did not reach rest')
    return {'status': 'passed', 'handoff_index': cut, 'rest_index': end,
            'braking_ticks': end - cut, 'braking_magnitude': braking,
            'scope': 'joint braking within query routes and native collision constraints; suffix not committed'}
