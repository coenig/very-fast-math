"""Conservative element-entry braking budget, NOT a realizability certificate.

The endpoint model uses one-second ticks. The current acceleration determines
the next displacement; a newly chosen braking command takes effect afterwards.
Clearance here reserves one vehicle length plus an optional margin in *each*
section/connector. It is not the remaining clearance of an arbitrary handoff.
"""
from fractions import Fraction
import json
import math
from pathlib import Path
import re

from .common import ROOT, dump


def stopping_distance(speed, braking, tick=1):
    """Worst-case distance with one committed tick, followed by full braking."""
    v, b, dt = map(Fraction, (speed, braking, tick))
    if v < 0 or b <= 0 or dt <= 0:
        raise ValueError('speed must be nonnegative; braking and tick must be positive')
    n = math.ceil(v / (b * dt))
    return n * v * dt - b * dt * dt * n * (n - 1) / 2


def compatible_speed(clearance, braking):
    """Largest integer symmetric speed cap satisfying the one-second budget."""
    d = Fraction(clearance)
    lo, hi = 0, max(0, math.floor(d))
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if stopping_distance(mid, braking) <= d:
            lo = mid
        else:
            hi = mid - 1
    return lo


def effective_parameters(template_config, overrides, stable=False):
    # Read defaults from the actual template rather than duplicating them.
    source = (ROOT / 'src/templates/EnvModel_Parameters.tpl').read_text()
    keys = ('MAXSPEEDNONEGO', 'MINACCELNONEGO', 'MAXACCELNONEGO', 'VEHICLE_LENGTH')
    values = {}
    for key in keys:
        match = re.search(r'@\{' + key + r'\}@\*+\.defaultValue\[(-?\d+)\]', source)
        if not match:
            raise ValueError(f'cannot resolve template default for {key}')
        values[key] = int(match[1])
    values.update(json.loads(Path(template_config).read_text())['_config'])
    if stable:
        fixed = dict(VEHICLE_LENGTH=5, MINACCELNONEGO=-8, MAXACCELNONEGO=3)
        if fixed.keys() & overrides.keys():
            raise ValueError('stable profile dynamics cannot be overridden')
        values.update(fixed)
    if {'TIMESCALING', 'DISTANCESCALING'} & overrides.keys():
        raise ValueError('endpoint time/distance scaling cannot be overridden')
    values.update(overrides)
    result = {}
    for key in keys:
        value = values[key]
        if isinstance(value, bool) or not re.fullmatch(r'-?\d+', str(value)):
            raise ValueError(f'{key} must be a literal integer for the braking check; got {value!r}')
        result[key] = int(value)
    return result


def check_geometry(geometry, parameters, margin=0):
    """Check all elements, using physical chord-sum bounds for curved connectors.

    MAXSPEEDNONEGO is used even when lane limits are tighter: lane limits alone
    do not bound all connector states. No parameter is changed by this check.
    """
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('braking margin must be finite and nonnegative')
    v = parameters['MAXSPEEDNONEGO']
    amin, amax = parameters['MINACCELNONEGO'], parameters['MAXACCELNONEGO']
    body = parameters['VEHICLE_LENGTH']
    if v <= 0 or body <= 0 or not amin < 0 < amax:
        raise ValueError('need positive speed/vehicle length and MINACCELNONEGO < 0 < MAXACCELNONEGO')
    b = min(-amin, amax)
    distance = stopping_distance(v, b)
    elements = []
    for kind, entries in (('section', geometry['sections']), ('connector', geometry['connections'])):
        for item in entries:
            length = min(item['length'], item.get('arc_length_m', item['length']))
            clearance = Fraction(str(length)) - body - Fraction(str(margin))
            elements.append(dict(kind=kind, id=item['id'], length_m=length,
                                 clearance_m=float(clearance), passed=distance <= clearance))
    if not elements:
        raise ValueError('no geometry to check')
    limiting = min(elements, key=lambda e: e['clearance_m'])
    clearance = Fraction(str(limiting['clearance_m']))
    cap = compatible_speed(clearance, b)
    required_b = None
    # The committed first tick cannot be shortened by choosing stronger braking.
    if clearance >= v:
        lo, hi = 1, v
        while lo < hi:
            mid = (lo + hi) // 2
            if stopping_distance(v, mid) <= clearance:
                hi = mid
            else:
                lo = mid + 1
        required_b = lo
    passed = all(e['passed'] for e in elements)
    message = ('Braking budget passed.' if passed else
               f"Braking budget failed at {limiting['kind']} {limiting['id']}: "
               f"need {float(distance):g} m, available {float(clearance):g} m. "
               f"Largest compatible integer speed cap: {cap} m/s (range [-{cap}, {cap}]). "
               + (f'Minimum symmetric braking magnitude at the current speed: {required_b} m/s^2.'
                  if required_b is not None else
                  'Stronger braking alone cannot help: the committed first tick is already too long.'))
    if not passed and cap == 0:
        message += ' No positive integer speed satisfies this policy; increase the limiting lengths or revise the clearance policy.'
    return dict(schema='endpoint-braking-budget.v1', passed=passed, message=message,
                scope='element-entry longitudinal budget only; not a fleet or handoff feasibility certificate',
                parameters=parameters, tick_seconds=1, reserved_vehicle_length_m=body,
                extra_margin_m=margin, braking_mps2=b, stopping_distance_m=float(distance),
                required_element_length_m=float(distance) + body + margin,
                largest_integer_speed_cap_mps=cap, minimum_symmetric_braking_mps2=required_b,
                suggested_vfm=(dict(MAXSPEEDNONEGO=cap, LANES_MIN_SPEEDS=f'@(-{cap})@',
                                    LANES_MAX_SPEEDS=f'@({cap})@') if cap > 0 else None),
                limiting_element=limiting, elements=elements)


def local_budget(geometry, parameters, margin=0):
    """Largest integer limit per element, under the configured global ceiling.

    This is a speed limit throughout each element, including its entry state.
    It avoids an artificial zero-speed boundary (which would prevent crossing).
    The transition solver must brake before entering a slower element.
    """
    report = check_geometry(geometry, parameters, margin)
    report['schema'] = 'endpoint-braking-budget.v2'
    report['uniform_global_budget_passed'] = report['passed']
    for element in report['elements']:
        cap = min(parameters['MAXSPEEDNONEGO'], compatible_speed(
            str(element['clearance_m']), report['braking_mps2']))
        element.update(speed_limit_mps=cap, passed=cap > 0,
                       stopping_distance_m=float(stopping_distance(cap, report['braking_mps2'])))
    report['policy'] = 'local'
    report['passed'] = all(e['passed'] for e in report['elements'])
    report['message'] = ('Local braking limits computed; global speed ceiling retained.'
                         if report['passed'] else
                         'Local braking budget failed: an element admits no positive integer speed. '
                         'Increase its length or revise the clearance policy.')
    return report


def speed_constraints(geometry, cars, parameters, margin=0, symbolic=False):
    """Native EnvModel constraints; symbolic/fixed compilers share the formula."""
    report = local_budget(geometry, parameters, margin)
    if not report['passed']:
        raise ValueError(report['message'])
    caps = {(e['kind'], e['id']): e['speed_limit_mps'] for e in report['elements']}
    parking_cap = min(parameters['MAXSPEEDNONEGO'], compatible_speed(
        Fraction(str(geometry['parking_length'])) - parameters['VEHICLE_LENGTH'] - Fraction(str(margin)),
        report['braking_mps2'])) if symbolic else None
    if symbolic:
        report['symbolic_parking_speed_limit_mps'] = parking_cap
    lines = ['-- Length-dependent speed limits; signs cover both driving directions.']
    for car in range(cars):
        p = f'veh___6{car}9___'
        allowed = set(geometry.get('car_sections', [range(len(geometry['sections']))]*cars)[car])
        for s in geometry['sections']:
            if s['id'] not in allowed:
                continue
            cap = caps['section', s['id']]
            if symbolic:
                u, v = s['edge']
                cap = f'(case parking_{u} | parking_{v} : {parking_cap}; TRUE : {cap}; esac)'
            lines.append(f'INVAR {p}.on_straight_section = {s["id"]} -> '
                         f'({p}.v >= -({cap}) & {p}.v <= {cap});')
        for c in geometry['connections']:
            if not {c['from'], c['to']} <= allowed:
                continue
            cap = caps['connector', c['id']]
            lines.append(f'INVAR {p}.traversion_from = {c["from"]} & {p}.traversion_to = {c["to"]} -> '
                         f'({p}.v >= -{cap} & {p}.v <= {cap});')
    if symbolic and parking_cap < 1:
        raise ValueError('parking length admits no positive integer speed')
    return '\n'.join(lines) + '\n', report


def preflight(geometry, config, template_config, output, margin=0):
    try:
        parameters = effective_parameters(template_config, config.get('vfm', {}),
                                          config.get('semantics') == 'stable_concurrent')
        report = local_budget(geometry, parameters, margin)
    except (ValueError, KeyError) as exc:
        report = dict(passed=False, message=f'Cannot check braking budget: {exc}')
    dump(Path(output) / 'braking_check.json', report)
    print(report['message'], flush=True)
    return report


def with_speed(config, max_speed):
    if max_speed is None:
        return config
    if type(max_speed) is not int or max_speed <= 0:
        raise ValueError('max-speed must be a positive integer')
    return {**config, 'vfm': {**config.get('vfm', {}), 'MAXSPEEDNONEGO': max_speed,
                            'LANES_MIN_SPEEDS': f'@(-{max_speed})@',
                            'LANES_MAX_SPEEDS': f'@({max_speed})@'}}
