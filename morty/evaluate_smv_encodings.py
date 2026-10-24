import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import statistics
import subprocess
import tempfile
import time
from types import SimpleNamespace

from .evaluate_mc_selection import OUTPUT, REPO, run_trial, write_json


ENCODINGS = ('baseline', 'jump_linear', 'jump_lookup', 'lane_integer', 'lane_range',
             'velocity', 'parked', 'combined', 'combined_range', 'combined_lookup', 'combined_keep_acceleration')


def replace_definition(source, name, expression):
    pattern = r'(?m)^([ \t]*)' + re.escape(name) + r'\s*:=\s*(?:case\b[\s\S]*?\besac|[^;]*);'
    rewritten, count = re.subn(pattern, lambda match: match[1] + name + ' := ' + expression + ';', source)
    if count != 1:
        raise ValueError(f'Expected one definition of {name}, found {count}')
    return rewritten


def compact_jumps(source, count, lookup=False):
    for index in range(count):
        vehicle = f'veh___6{index}9___'
        difference = f'(next({vehicle}.on_normalized_lane) - {vehicle}.on_normalized_lane)'
        distance = f'abs({difference})'
        if lookup:
            permitted = '(case\n' + '\n'.join(
                f'    next({vehicle}.v) <= {47 - 3 * jump} : {jump};'
                for jump in range(7, 0, -1)) + '\n    TRUE : 0;\nesac)'
            condition = f'{distance} <= {permitted}'
        else:
            condition = f'next({vehicle}.v) + 3 * {distance} <= 47'
        for direction, sign, boundary in [('down', '<', 0), ('up', '>', 22)]:
            expression = (f'({difference} {sign} 0 & {distance} <= 7 & {condition})'
                          f' | ({vehicle}.on_normalized_lane = {boundary} & {difference} = 0'
                          f' & next({vehicle}.v) <= 47)')
            source = replace_definition(source, f'{vehicle}.lane_move_{direction}', expression)
    return source


def remove_declaration(source, name):
    pattern = r'(?m)^[ \t]*(?:VAR[ \t]+)?' + re.escape(name) + r'\s*:\s*(?:integer|boolean|0\.\.22);[^\n]*'
    source, count = re.subn(pattern, '', source)
    if count != 1:
        raise ValueError(f'Expected one declaration of {name}, found {count}')
    return source


def scalar_lanes(source, count, bounded_type=False):
    for index in range(count):
        vehicle = f'veh___6{index}9___'
        coordinate = f'{vehicle}.lane_coordinate'
        for bit in range(12):
            source = remove_declaration(source, f'{vehicle}.lane_b{bit}')
            source += (f'\nDEFINE {vehicle}.lane_b{bit} := '
                       f'{coordinate} >= {2 * bit - 1} & {coordinate} <= {2 * bit + 1};\n')
            source = replace_definition(source, f'{vehicle}.lane_{bit}', f'{coordinate} = {2 * bit}')
            if bit < 11:
                source = replace_definition(source, f'{vehicle}.lane_{bit}_{bit + 1}',
                                            f'{coordinate} = {2 * bit + 1}')
        source = replace_definition(source, f'{vehicle}.on_normalized_lane', coordinate)
        source = replace_definition(source, f'{vehicle}.lane_unchanged', f'{coordinate} = next({coordinate})')
        source += f'\nVAR {coordinate} : {"0..22" if bounded_type else "integer"};\n'
        if not bounded_type:
            source += f'INVAR {coordinate} >= 0 & {coordinate} <= 22;\n'
    return source


def project_acceleration(source, count):
    source = remove_declaration(source, 'ego.a')
    source, aliases = re.subn(r'INVAR ego\.a = veh___609___\.a;', '', source)
    if aliases != 1:
        raise ValueError('Missing task-7 acceleration alias')
    for index in range(count):
        vehicle = f'veh___6{index}9___'
        source = remove_declaration(source, f'{vehicle}.a')
        constraint = f'(a_min <= {vehicle}.a & {vehicle}.a <= a_max) &'
        if source.count(constraint) != 1:
            raise ValueError(f'Unexpected acceleration bounds for {vehicle}')
        source = source.replace(constraint, '')
        assignment = f'next({vehicle}.v) := min(max({vehicle}.v + {vehicle}.a, -max_vel), max_vel);'
        if source.count(assignment) != 1:
            raise ValueError(f'Unexpected velocity transition for {vehicle}')
        source = source.replace(assignment, f'TRANS next({vehicle}.v) >= {vehicle}.v - 3'
                                f' & next({vehicle}.v) <= {vehicle}.v + 3;\nASSIGN\n')
    return source


def define_state(source, name, expression):
    source = remove_declaration(source, name)
    pattern = r'(?m)^[ \t]*(init|next)\(' + re.escape(name) + r'\)\s*:=\s*(case\b[\s\S]*?\besac|[^;]*);'

    def replace_assignment(match):
        condition = name if match[1] == 'init' else f'next({name})'
        return f'{"INIT" if match[1] == "init" else "TRANS"} {condition} = ({match[2]});\nASSIGN\n'

    source = re.sub(pattern, replace_assignment, source)
    return source + f'\nDEFINE {name} := {expression};\n'


def freeze_parked(source, main, count, scalar=False, projected=False):
    frozen = []
    for index in range(count):
        vehicle = f'veh___6{index}9___'
        if not re.search(r'INVAR env\.' + re.escape(vehicle) + r'\.v = 0;', main):
            continue
        position = re.search(r'INIT env\.' + re.escape(vehicle) + r'\.abs_pos = (-?\d+);', main)
        if not position or not 0 <= int(position[1]) <= 40000:
            continue
        lane_hold = (r'TRANS\s+' + re.escape(vehicle)
                     + r'\.time_since_last_lc\s*<\s*min_time_between_lcs\s*\|\s*abs\('
                     + re.escape(vehicle) + r'\.v\)\s*<\s*[1-9]\d*\s*->\s*'
                     + re.escape(vehicle) + r'\.lane_unchanged;')
        if not re.search(lane_hold, source):
            raise ValueError('Parked specialization requires the stationary lane-hold constraint')
        frozen.append(index)
        source = define_state(source, f'{vehicle}.v', '0')
        if not projected:
            source = define_state(source, f'{vehicle}.a', '0')
        source = define_state(source, f'{vehicle}.abs_pos', position[1])
        source = define_state(source, f'{vehicle}.prev_abs_pos', f'case cnt = 0 : 0; TRUE : {position[1]}; esac')
        enabled = []
        for bit in range(12):
            if re.search(r'INIT env\.' + re.escape(vehicle) + rf'\.lane_b{bit};', main):
                enabled.append(bit)
            elif not re.search(r'INIT !env\.' + re.escape(vehicle) + rf'\.lane_b{bit};', main):
                raise ValueError(f'Incomplete initial lane bits for {vehicle}')
        if len(enabled) not in (1, 2) or enabled[-1] - enabled[0] != len(enabled) - 1:
            raise ValueError(f'Invalid initial lane bits for {vehicle}')
        if scalar:
            source = define_state(source, f'{vehicle}.lane_coordinate', str(2 * enabled[0] + len(enabled) - 1))
        else:
            for bit in range(12):
                source = define_state(source, f'{vehicle}.lane_b{bit}', 'TRUE' if bit in enabled else 'FALSE')
    return source, frozen


def transformation_key(main):
    vehicles = re.findall(r'INVAR env\.(veh___6\d+9___)\.v = 0;', main)
    return tuple(line.strip() for line in main.splitlines()
                 if re.match(r'^\s*(?:INIT|INVAR)\s', line)
                 and any(f'env.{vehicle}.' in line for vehicle in vehicles))


def transform_model(source, main, encoding):
    if encoding not in ENCODINGS:
        raise ValueError(f'Unknown encoding: {encoding}')
    required = {'m_j': '7', 'vel_min_j': '24', 'vel_max_j': '6', 'max_vel': '140',
                'a_min': '-3', 'a_max': '3'}
    for name, value in required.items():
        if not re.search(r'\b' + name + r'\s*:=\s*' + value + r'\s*;', source):
            raise ValueError(f'Unsupported task-7 parameter {name}')
    count = len(re.findall(r'veh___6\d+9___\.v\s*:\s*integer;', source))
    if (count < 3 or 'UCD experiment7' not in main
            or not re.search(r'section_0_end\s*:\s*40000\s*\.\.\s*40000\s*;', source)
            or re.search(r'\bsection_[1-9]\d*_end\b', source)
            or not re.search(r'INVAR num_technical_lanes = 12;', source)):
        raise ValueError('Only generated task-7 packages are supported')
    frozen = []
    combined = encoding.startswith('combined')
    projected = encoding == 'velocity' or (combined and encoding != 'combined_keep_acceleration')
    if encoding in ('lane_integer', 'lane_range') or combined:
        source = scalar_lanes(source, count, bounded_type=encoding in ('lane_range', 'combined_range'))
    if encoding in ('jump_linear', 'jump_lookup') or combined:
        source = compact_jumps(source, count, lookup=encoding in ('jump_lookup', 'combined_lookup'))
    if projected:
        source = project_acceleration(source, count)
    if encoding == 'parked' or combined:
        source, frozen = freeze_parked(source, main, count, scalar=combined, projected=projected)
    source = re.sub(r'(?m)^VAR[ \t]*\n(?:[ \t]*(?:--[^\n]*)?\n)*(?=(?:VAR|DEFINE|ASSIGN|INIT|INVAR|TRANS)\b)',
                    '', source)
    return source, {'vehicles': count, 'frozen': frozen}


def run_solver(folder, bound=30, timeout=60):
    command_file = folder / 'experiment.cmd'
    command_file.write_text('set on_failure_script_quits 1\ngo_msat\n'
                            f'msat_check_invar_bmc -i -a falsification -k {bound}\nquit\n')
    environment = dict(os.environ)
    libraries = str(REPO / 'external/linux64/lib')
    environment['LD_LIBRARY_PATH'] = libraries + os.pathsep + environment.get('LD_LIBRARY_PATH', '')
    command = [str(REPO / 'external/linux64/nuXmv'), '-int', '-pre', 'cpp',
               '-source', str(command_file), str(folder / 'main.smv')]
    started = time.perf_counter()
    process = subprocess.Popen(command, cwd=folder, env=environment, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, start_new_session=True)
    timed_out = False
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            output, _ = process.communicate()
    elapsed = time.perf_counter() - started
    witness = re.search(r'-> State: \d+\.(\d+) <-', output)
    checked = re.findall(r'no counterexample found[^\n]*?(?:with bound|up to) (\d+)', output)
    if timed_out:
        outcome = 'timeout'
    elif process.returncode != 0 or re.search(r'(?im)^.*(?:error:|undefined|cannot open)', output):
        outcome = 'error'
    elif witness:
        outcome = 'sat'
    elif checked and int(checked[-1]) == bound:
        outcome = 'unsat_bound'
    else:
        outcome = 'error'
    states = re.findall(r'-> State: \d+\.(\d+) <-', output)
    result = {'outcome': outcome, 'seconds': elapsed, 'exit_code': process.returncode,
              'depth': int(states[-1]) - 1 if states else None, 'bound': bound}
    return result, output


def check_local_relations():
    vehicle = 'veh___609___'
    lane_names = [f'{vehicle}.normalized_lane_{index}' for index in range(23)]
    definitions = '\n'.join(f'{name} := {vehicle}.on_normalized_lane = {index};'
                            for index, name in enumerate(lane_names))
    movements = {}
    for direction, step in [('down', -1), ('up', 1)]:
        terms = []
        for current in range(23):
            choices = []
            for jump in range(1, 8):
                target = max(0, min(22, current + step * jump))
                choices.append(f'({abs(target - current)} <= max_jump_lanes_0 & {lane_names[target]})')
            terms.append(f'({lane_names[current]} -> next(' + ' | '.join(choices) + '))')
        movements[direction] = ' &\n'.join(terms)
    original = '\n'.join(f'{vehicle}.lane_move_{direction} := {expression};'
                         for direction, expression in movements.items())
    rewritten = compact_jumps(original, 1).replace(f'{vehicle}.lane_move_', f'{vehicle}.compact_')
    lookup = compact_jumps(original, 1, lookup=True).replace(f'{vehicle}.lane_move_', f'{vehicle}.lookup_')
    relation = ' | '.join([f'{vehicle}.on_normalized_lane = next({vehicle}.on_normalized_lane)',
                           f'{vehicle}.lane_move_down', f'{vehicle}.lane_move_up'])
    source = (f'MODULE main\nVAR {vehicle}.on_normalized_lane : integer; {vehicle}.v : integer;\n'
              f'INVAR {vehicle}.on_normalized_lane >= 0 & {vehicle}.on_normalized_lane <= 22;\n'
              f'INVAR {vehicle}.v >= -140 & {vehicle}.v <= 140;\n'
              f'DEFINE max_jump_lanes_0 := 7 - ({vehicle}.v - 24) * 6 / 18;\n'
              + definitions + '\n' + original + '\n' + rewritten + '\n' + lookup + '\n'
              + f'TRANS !((({relation}) <-> ({relation.replace(".lane_move_", ".compact_")}))'
              + f' & (({relation}) <-> ({relation.replace(".lane_move_", ".lookup_")})));\n'
              + 'VAR step : 0..1;\nASSIGN init(step) := 0; next(step) := 1;\n'
              + 'INVARSPEC step = 0;\n')
    with tempfile.TemporaryDirectory(prefix='vfm-task7-relations-') as directory:
        folder = Path(directory)
        (folder / 'main.smv').write_text(source)
        result, output = run_solver(folder, bound=1)
        if result['outcome'] != 'unsat_bound':
            raise AssertionError(f'Lane-relation check failed: {result}\n{output}')
        projected = 'next(velocity) >= velocity - 3 & next(velocity) <= velocity + 3'
        original_velocity = ' | '.join(
            f'next(velocity) = min(max(velocity + ({acceleration}), -140), 140)'
            for acceleration in range(-3, 4))
        (folder / 'main.smv').write_text(
            'MODULE main\nVAR velocity : integer; step : 0..1;\n'
            'INVAR velocity >= -140 & velocity <= 140;\n'
            'ASSIGN init(step) := 0; next(step) := 1;\n'
            f'TRANS !(({original_velocity}) <-> ({projected}));\nINVARSPEC step = 0;\n')
        velocity_result, output = run_solver(folder, bound=1)
        if velocity_result['outcome'] != 'unsat_bound':
            raise AssertionError(f'Velocity check failed: {velocity_result}\n{output}')
    return {'lane_relations': result, 'velocity_projection': velocity_result}


def smoke_package(package):
    source = (package / 'EnvModel.smv').read_text()
    main = (package / 'main.smv').read_text()
    results = {}
    with tempfile.TemporaryDirectory(prefix='vfm-task7-smoke-') as directory:
        root = Path(directory)
        for encoding in ENCODINGS:
            folder = root / encoding
            folder.mkdir()
            model, metadata = transform_model(source, main, encoding)
            (folder / 'EnvModel.smv').write_text(model)
            for name in ('main.smv', 'planner.cpp_combined.k2.smv'):
                shutil.copy2(package / name, folder / name)
            result, output = run_solver(folder, bound=1)
            if result['outcome'] not in ('sat', 'unsat_bound'):
                raise AssertionError(f'{encoding}: {result}\n{output}')
            if results and result['outcome'] != results['baseline']['outcome']:
                raise AssertionError(f'{encoding} disagrees with baseline: {result}')
            results[encoding] = {**result, **metadata}
    return results


def collect_queries(folder, config_path):
    config = json.loads(config_path.read_text())
    if config['#TEMPLATE']['UCD_EXP_NUM'] != 7:
        raise ValueError('Only task 7 is supported')
    config['#TEMPLATE']['UCD_TASK7_COMPACT'] = False
    config['#TEMPLATE']['UCD_EVALUATION_CAPTURE_SMV'] = True
    write_json(folder / 'source.tpl.json', config)
    trials = []
    for seed, steps in [(0, 3), (1, 4), (5, 1)]:
        args = SimpleNamespace(tag=folder.name + '_capture', configs=None, steps=steps, timeout=180)
        result = run_trial(args, config, 'priority', seed)
        if result['outcome'] in ('timeout', 'error'):
            raise RuntimeError(f'Query collection failed: {result}')
        trials.append(str(OUTPUT / f'{args.tag}_priority_seed{seed}'))
    write_json(folder / 'trials.json', trials)
    return trials


def prepare_cases(folder):
    trials = [Path(path) for path in json.loads((folder / 'trials.json').read_text())]
    cases = []
    for trial in trials:
        seed = json.loads((trial / 'result.json').read_text())['seed']
        queries = [json.loads(line) for line in (trial / 'queries.jsonl').read_text().splitlines()]
        selections = []
        if seed in (0, 1):
            selections.append((f'full_seed{seed}', next(query for query in queries if not query['shortcut'])))
            successful = seed == 1
            selections.append((f'shortcut_{"sat" if successful else "unsat"}_seed{seed}',
                               next(query for query in queries if query['shortcut'] and query['success'] == successful)))
        else:
            for priority in (0, 15):
                selections.append((f'blind_seed{seed}_priority{priority}',
                                   next(query for query in queries if query['priority'] == priority)))
        for name, query in selections:
            case = folder / 'cases' / name
            case.mkdir(parents=True, exist_ok=False)
            package = trial / ('generated' + query['config'])
            for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
                shutil.copy2(package / filename, case / filename)
            (case / 'main.smv').write_text(query['main_smv'])
            shutil.copy2(trial / 'morty/envmodel_config.tpl.json', case / 'envmodel_config.tpl.json')
            hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in case.glob('*.smv')}
            entry = {'name': name, 'seed': seed, 'priority': query['priority'], 'iteration': query['iteration'],
                     'shortcut': query['shortcut'], 'expected_sat': query['success'], 'hashes': hashes}
            write_json(case / 'query.json', entry)
            cases.append(entry)
    write_json(folder / 'cases.json', cases)
    config = json.loads((folder / 'source.tpl.json').read_text())
    config['#TEMPLATE'].update(UCD_SMV_EXPERIMENT_ENCODINGS=';'.join(ENCODINGS),
                               UCD_SMV_EXPERIMENT_REPEATS=3, UCD_SMV_EXPERIMENT_TIMEOUT_SECONDS=30)
    write_json(folder / 'experiment.tpl.json', config)
    return cases


def benchmark(folder):
    config = json.loads((folder / 'experiment.tpl.json').read_text())
    template = config['#TEMPLATE']
    encodings = template['UCD_SMV_EXPERIMENT_ENCODINGS'].split(';')
    repeats = template['UCD_SMV_EXPERIMENT_REPEATS']
    timeout = template['UCD_SMV_EXPERIMENT_TIMEOUT_SECONDS']
    bound = template['BMC_CNT']
    if 'baseline' not in encodings or any(encoding not in ENCODINGS for encoding in encodings):
        raise ValueError('Select known encodings and include baseline')
    cases = json.loads((folder / 'cases.json').read_text())
    results = []
    for case in cases:
        original = folder / 'cases' / case['name']
        for encoding in encodings:
            package = folder / 'benchmarks' / case['name'] / encoding
            package.mkdir(parents=True, exist_ok=False)
            main = (original / 'main.smv').read_text()
            model, metadata = transform_model((original / 'EnvModel.smv').read_text(), main, encoding)
            (package / 'EnvModel.smv').write_text(model)
            for name in ('main.smv', 'planner.cpp_combined.k2.smv'):
                shutil.copy2(original / name, package / name)
            isolated = json.loads((original / 'envmodel_config.tpl.json').read_text())
            isolated['#TEMPLATE'].update(UCD_SMV_EXPERIMENT_ENCODING=encoding,
                                         UCD_SMV_EXPERIMENT_REPEATS=repeats,
                                         UCD_SMV_EXPERIMENT_TIMEOUT_SECONDS=timeout)
            write_json(package / 'envmodel_config.tpl.json', isolated)
            write_json(package / 'transformation.json', metadata)
        for repetition in range(repeats):
            order = encodings[repetition % len(encodings):] + encodings[:repetition % len(encodings)]
            for encoding in order:
                package = folder / 'benchmarks' / case['name'] / encoding
                result, output = run_solver(package, bound=bound, timeout=timeout)
                (package / f'solver_{repetition}.log').write_text(output)
                result.update(case=case['name'], encoding=encoding, repetition=repetition)
                results.append(result)
                write_json(folder / 'results.json', results)
                print(json.dumps(result), flush=True)
                if result['outcome'] == 'error':
                    raise RuntimeError(f'Solver error; see {package / f"solver_{repetition}.log"}')
                if result['outcome'] != 'timeout' and (result['outcome'] == 'sat') != case['expected_sat']:
                    raise AssertionError(f'Encoding changed bounded feasibility: {result}')
    summary = {}
    for encoding in encodings:
        medians = {}
        for case in cases:
            values = [entry for entry in results if entry['case'] == case['name'] and entry['encoding'] == encoding]
            medians[case['name']] = {'seconds': statistics.median(entry['seconds'] for entry in values),
                                    'outcomes': sorted({entry['outcome'] for entry in values}),
                                    'depths': sorted({entry['depth'] for entry in values if entry['depth'] is not None})}
        summary[encoding] = {'cases': medians, 'sum_median_seconds': sum(value['seconds'] for value in medians.values()),
                             'timeouts': sum(entry['outcome'] == 'timeout' for entry in results if entry['encoding'] == encoding)}
    write_json(folder / 'summary.json', summary)
    return summary


def read_trace(output, main):
    initial = {}
    for match in re.finditer(r'(?m)^INIT (!?)(env\.veh___6\d+9___\.[\w]+)(?: = (-?\d+))?;', main):
        initial[match[2]] = int(match[3]) if match[3] else not bool(match[1])
    states = []
    for line in output.splitlines():
        if re.search(r'-> State: \d+\.\d+ <-', line):
            states.append(dict(states[-1] if states else initial))
        elif states:
            match = re.fullmatch(r'\s*(env\.veh___6\d+9___\.[\w]+) = (-?\d+|TRUE|FALSE)\s*', line)
            if match:
                states[-1][match[1]] = match[2] == 'TRUE' if match[2] in ('TRUE', 'FALSE') else int(match[2])
    for state in states:
        for name, value in list(state.items()):
            if name.endswith('.lane_coordinate'):
                vehicle = name.removesuffix('.lane_coordinate')
                for bit in range(12):
                    state[f'{vehicle}.lane_b{bit}'] = 2 * bit - 1 <= value <= 2 * bit + 1
    return states


def pin_trace(main, states):
    suffix = r'\.(?:abs_pos|v|lane_b\d+|on_straight_section|traversion_from|traversion_to)$'
    constraints = []
    for step, state in enumerate(states):
        assignments = [f'{name} = {str(value).upper()}' for name, value in sorted(state.items())
                       if re.search(suffix, name)]
        if not assignments:
            raise ValueError('Trace has no vehicle state')
        constraints.append(f'INVAR env.cnt != {step} | (' + ' & '.join(assignments) + ');')
    return main + '\n' + '\n'.join(constraints) + '\n'


def verify_witnesses(folder):
    results = json.loads((folder / 'results.json').read_text())
    checks = []
    for entry in results:
        if entry['repetition'] != 0 or entry['encoding'] == 'baseline' or entry['outcome'] != 'sat':
            continue
        original = folder / 'benchmarks' / entry['case'] / 'baseline'
        candidate = folder / 'benchmarks' / entry['case'] / entry['encoding']
        for direction, source, destination in [('candidate_to_baseline', candidate, original),
                                                ('baseline_to_candidate', original, candidate)]:
            output = (source / 'solver_0.log').read_text()
            main = (destination / 'main.smv').read_text()
            states = read_trace(output, main)
            if not states:
                raise ValueError(f'Missing trace in {source}')
            check = folder / 'witness_checks' / entry['case'] / entry['encoding'] / direction
            check.mkdir(parents=True, exist_ok=False)
            for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
                shutil.copy2(destination / filename, check / filename)
            (check / 'main.smv').write_text(pin_trace(main, states))
            result, output = run_solver(check, bound=len(states) - 1, timeout=30)
            (check / 'solver.log').write_text(output)
            result.update(case=entry['case'], encoding=entry['encoding'], direction=direction)
            checks.append(result)
            write_json(folder / 'witness_checks.json', checks)
            if result['outcome'] != 'sat' or result['depth'] != len(states) - 1:
                raise AssertionError(f'Witness failed cross-model validation: {result}')
    return checks


def check_live_runs(folder, config_path):
    config = json.loads(config_path.read_text())
    template = config['#TEMPLATE']
    if template['UCD_EXP_NUM'] != 7:
        raise ValueError('Only task 7 is supported')
    encoding = template.get('UCD_EVALUATION_SMV_ENCODING', template.get('UCD_SMV_EXPERIMENT_ENCODING'))
    if encoding not in ENCODINGS:
        raise ValueError('Unknown SMV encoding')
    template['UCD_TASK7_COMPACT'] = False
    template['UCD_EVALUATION_SMV_ENCODING'] = encoding
    template['UCD_EVALUATION_CAPTURE_SMV'] = False
    write_json(folder / 'experiment.tpl.json', config)
    results = []
    for seed in (0, 1):
        args = SimpleNamespace(tag=folder.name, configs=None, steps=50, timeout=180)
        result = run_trial(args, config, 'priority', seed)
        results.append(result)
        write_json(folder / 'results.json', results)
        if result['outcome'] == 'error':
            raise RuntimeError(f'Live check failed: {result}')
    return results


def main():
    parser = argparse.ArgumentParser(description='Isolated task-7 SMV encoding experiments')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--package', type=Path)
    parser.add_argument('--collect', type=Path)
    parser.add_argument('--prepare', type=Path)
    parser.add_argument('--benchmark', type=Path)
    parser.add_argument('--verify', type=Path)
    parser.add_argument('--live-check', type=Path)
    parser.add_argument('--config', type=Path, default=REPO / 'morty/envmodel_config.tpl.json')
    args = parser.parse_args()
    if args.self_test:
        results = check_local_relations()
        if args.package:
            results['packages'] = smoke_package(args.package.resolve())
        print(json.dumps(results, indent=2))
    elif args.collect:
        folder = args.collect.resolve()
        folder.mkdir(parents=True, exist_ok=False)
        print(json.dumps(collect_queries(folder, args.config.resolve()), indent=2))
    elif args.prepare:
        print(json.dumps(prepare_cases(args.prepare.resolve()), indent=2))
    elif args.benchmark:
        print(json.dumps(benchmark(args.benchmark.resolve()), indent=2))
    elif args.verify:
        print(json.dumps(verify_witnesses(args.verify.resolve()), indent=2))
    elif args.live_check:
        folder = args.live_check.resolve()
        folder.mkdir(parents=True, exist_ok=False)
        print(json.dumps(check_live_runs(folder, args.config.resolve()), indent=2))
    else:
        parser.error('Use --self-test, --collect, --prepare, --benchmark, --verify, or --live-check')


if __name__ == '__main__':
    main()