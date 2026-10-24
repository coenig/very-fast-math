import argparse
import ast
from collections import Counter, defaultdict
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

from .morty_helper import MCConfigSelector


REPO = Path(__file__).resolve().parent.parent
OUTPUT = REPO / 'tmp/ucd_live_task7_eval/selection_eval'


class EvaluationSelector(MCConfigSelector):
    def __init__(self, config_count, allowed, policy, observe):
        super().__init__(config_count)
        self.allowed = frozenset(allowed)
        if not self.allowed or min(self.allowed) < 0 or max(self.allowed) >= config_count:
            raise ValueError('Configuration indices must select a nonempty subset of the configured priorities')
        if policy not in ('adaptive', 'priority', 'cascade'):
            raise ValueError('Unknown evaluation policy')
        self.policy = policy
        self.observe = observe
        self.point_index = None

    def ordered_configs(self, shortcut, indices):
        candidates = [index for index in indices if index in self.allowed]
        if self.policy != 'adaptive':
            return sorted(candidates)
        return super().ordered_configs(shortcut, candidates)

    def queries(self, shortcut_count, max_shortcut_priority):
        if self.policy == 'cascade':
            eligible = [index for index in sorted(self.allowed)
                        if max_shortcut_priority is not None and index <= max_shortcut_priority]
            for point_index in range(shortcut_count):
                for config_index in eligible:
                    self.point_index = point_index
                    yield config_index, point_index
            for config_index in sorted(self.allowed):
                self.point_index = None
                yield config_index, None
        else:
            for config_index, point_index in super().queries(shortcut_count, max_shortcut_priority):
                self.point_index = point_index
                yield config_index, point_index

    def record(self, config_index, shortcut, success, seconds):
        super().record(config_index, shortcut, success, seconds)
        self.observe({'priority': config_index, 'shortcut': shortcut,
                      'point_index': self.point_index, 'success': success, 'seconds': seconds})


def summarize_trial(records, exit_code, timed_out):
    blind = max((record['blind_steps'] for record in records), default=0)
    crashed = max((record['crashed_steps'] for record in records), default=0)
    completed = bool(records and records[-1]['driver_completed'])
    if timed_out:
        outcome = 'timeout'
    elif exit_code != 0:
        outcome = 'error'
    elif completed:
        outcome = 'complete'
    elif blind:
        outcome = 'blind'
    elif crashed:
        outcome = 'crash'
    else:
        outcome = 'step_limit'
    return {
        'outcome': outcome,
        'cycles': len(records),
        'blind_events': blind,
        'crashed_steps': crashed,
        'model_finished': bool(records and records[-1]['finished']),
        'continuous_goal': bool(records and records[-1]['success']),
        'solver_seconds': sum(record['solver_seconds'] for record in records),
        'cycle_seconds': sum(record['seconds'] for record in records),
        'solver_jobs': sum(len(record['solver_jobs']) for record in records),
        'exit_code': exit_code,
        'timed_out': timed_out,
    }


def minimum_vehicle_clearance(vehicles, moving_indices):
    from shapely.geometry import Polygon

    footprints = [Polygon(vehicle.polygon()) for vehicle in vehicles]
    closest = None
    for first in range(len(vehicles)):
        for second in range(first + 1, len(vehicles)):
            if first not in moving_indices and second not in moving_indices:
                continue
            distance = float(footprints[first].distance(footprints[second]))
            if closest is None or distance < closest['distance_m']:
                closest = {'distance_m': distance, 'pair': [first, second],
                           'footprints': [list(map(list, footprints[index].exterior.coords))
                                          for index in [first, second]]}
    return closest


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2))
    temporary.replace(path)


def prepare_trial(folder, config):
    folder.mkdir(parents=True, exist_ok=False)
    (folder / 'morty').mkdir()
    (folder / 'src').mkdir()
    shutil.copytree(REPO / 'src/templates', folder / 'src/templates')
    for local in ['src/examples', 'lib']:
        (folder / local).symlink_to(REPO / local, target_is_directory=True)
    binaries = folder / 'external/linux64'
    binaries.mkdir(parents=True)
    for name in ['kratos', 'lib']:
        (binaries / name).symlink_to(REPO / 'external/linux64' / name, target_is_directory=name == 'lib')
    wrapper = binaries / 'nuXmv'
    wrapper.write_text('#!/bin/sh\nfor model; do :; done\n'
                       'exec /usr/bin/time -a -f "%e %U %S %x" -o "${model%/*}/solver_times.tsv" '
                       + shlex.quote(str(REPO / 'external/linux64/nuXmv')) + ' "$@"\n')
    wrapper.chmod(0o755)
    isolated = json.loads(json.dumps(config))
    isolated['#TEMPLATE']['_CACHED_PATH'] = '../cache'
    isolated['#TEMPLATE']['_GENERATED_PATH'] = './generated'
    write_json(folder / 'morty/envmodel_config.tpl.json', isolated)


def evaluate_child(args):
    from . import morty_helper as helper

    folder = args.folder
    template = json.loads((folder / 'morty/envmodel_config.tpl.json').read_text())['#TEMPLATE']
    evaluation_policy = template['UCD_EVALUATION_POLICY']
    configured_indices = template['UCD_EVALUATION_CONFIGS']
    allowed = [int(value) for value in configured_indices.split(';')] if configured_indices else None
    encoding = template.get('UCD_EVALUATION_SMV_ENCODING', 'baseline')
    if encoding != 'baseline':
        from .evaluate_smv_encodings import transform_model, transformation_key
    namespace = {'__name__': 'morty.selection_evaluation', '__package__': 'morty'}
    original_context = helper.morty_script_context
    original_scheduler = helper.RunScheduler.__init__
    original_models = {}
    model_cache = {}
    stages = defaultdict(float)
    records = []
    queries = []
    offsets = {}
    observed_road = None
    moving_indices = set()
    clearance = None
    samples = 0
    started = time.perf_counter()

    def observe_query(event):
        event.update(iteration=namespace['global_counter'],
                     config=namespace['ucd_config_prios_str'][event['priority']],
                     model_input=namespace['mcinput'],
                     target=('INVARSPEC !(TRUE ' + namespace['future_points_str'][event['point_index']] + ');'
                             if event['shortcut'] else namespace['spec_res']))
        if template.get('UCD_EVALUATION_CAPTURE_SMV', False):
            event['main_smv'] = (folder / ('generated' + event['config']) / 'main.smv').read_text()
        queries.append(event)
        with (folder / 'queries.jsonl').open('a') as stream:
            stream.write(json.dumps(event) + '\n')

    helper.MCConfigSelector = lambda count, policy: EvaluationSelector(
        count, allowed if allowed is not None else range(count), evaluation_policy, observe_query)

    def seed_scheduler(scheduler, *parameters, **keywords):
        original_scheduler(scheduler, *parameters, **keywords)
        scheduler.seed = template['UCD_EVALUATION_SEED'] - 1

    helper.RunScheduler.__init__ = seed_scheduler

    class TimedLibrary:
        def __init__(self, library):
            self.library = library

        def expandScript(self, script, result, size):
            selected_query = re.search(rb'\.runMCJob\[([^\]]+)\]', script)
            if encoding != 'baseline' and selected_query:
                transform_started = time.perf_counter()
                package = folder / ('generated' + selected_query[1].decode())
                model = package / 'EnvModel.smv'
                if model not in original_models:
                    original_models[model] = model.read_text()
                main = (package / 'main.smv').read_text()
                cache_key = (model, transformation_key(main))
                if cache_key not in model_cache:
                    rewritten, _ = transform_model(original_models[model], main, encoding)
                    model_cache[cache_key] = rewritten
                model.write_text(model_cache[cache_key])
                stages['smv_transform'] += time.perf_counter() - transform_started
            if b'generateEnvmodels' in script:
                stage = 'generation'
            elif b'prepareInputForMortyUCD' in script or b'.runMCJob' in script:
                stage = 'planning'
            elif b'extractVehPosFromNusmvFile' in script:
                stage = 'positions'
            else:
                stage = 'formulas'
            before = time.perf_counter()
            try:
                return self.library.expandScript(script, result, size)
            finally:
                stages[stage] += time.perf_counter() - before

    @contextmanager
    def timed_context(*parameters, **keywords):
        with original_context(*parameters, **keywords) as library:
            yield TimedLibrary(library)

    helper.morty_script_context = timed_context

    def sample_clearance():
        nonlocal clearance, samples
        current = minimum_vehicle_clearance(namespace['env'].unwrapped.controlled_vehicles, moving_indices)
        samples += 1
        if current is not None and (clearance is None or current['distance_m'] < clearance['distance_m']):
            clearance = current
            clearance['sample'] = samples

    def begin_cycle():
        nonlocal observed_road, moving_indices, clearance, samples
        if not records:
            write_json(folder / 'startup.json', {'seconds': time.perf_counter() - started})
        stages.clear()
        queries.clear()
        offsets.clear()
        clearance = None
        samples = 0
        road = namespace['env'].unwrapped.road
        if observed_road is not road:
            observed_road = road
            vehicles = namespace['env'].unwrapped.controlled_vehicles
            moving_indices = {index for index, vehicle in enumerate(vehicles) if abs(vehicle.speed) > 1e-9}
            if not moving_indices:
                moving_indices = set(range(len(vehicles)))
            original_step = road.step

            def measured_step(dt):
                original_step(dt)
                sample_clearance()

            road.step = measured_step
        sample_clearance()
        for path in folder.glob('generated_config*/solver_times.tsv'):
            offsets[path] = len(path.read_text().splitlines())
        return time.perf_counter()

    def end_cycle(iteration, before):
        jobs = []
        for path in folder.glob('generated_config*/solver_times.tsv'):
            for line in path.read_text().splitlines()[offsets.get(path, 0):]:
                try:
                    wall, user, system, code = map(float, line.split())
                except ValueError:
                    continue
                jobs.append({'config': path.parent.name.removeprefix('generated'),
                             'wall': wall, 'cpu': user + system, 'exit': int(code)})
        vehicles = namespace['env'].unwrapped.controlled_vehicles
        selected = namespace.get('selected_cnt')
        record = {
            'iteration': iteration,
            'seconds': time.perf_counter() - before,
            'stages': dict(stages),
            'selected': selected,
            'selected_config': namespace['ucd_config_prios_str'][selected] if selected is not None else None,
            'shortcut': namespace.get('found_shortcut', False),
            'blind_steps': namespace.get('nocex_count', 0),
            'crashed_steps': namespace.get('crashed_count', 0),
            'finished': namespace.get('res_str') == 'FINISHED',
            'success': bool(namespace.get('successed', False)),
            'driver_completed': namespace['run_id'] in namespace.get('good_ones', []),
            'queries': list(queries),
            'solver_jobs': jobs,
            'solver_seconds': sum(job['wall'] for job in jobs),
            'clearance': clearance,
            'clearance_samples': samples,
            'moving_indices': sorted(moving_indices),
            'actions': [[float(value) for value in action] for action in namespace.get('action', ())],
            'positions': [[float(value) for value in vehicle.position] for vehicle in vehicles],
            'headings': [float(vehicle.heading) for vehicle in vehicles],
        }
        records.append(record)
        write_json(folder / 'metrics.json', records)

    class InstrumentCycles(ast.NodeTransformer):
        def visit_For(self, node):
            self.generic_visit(node)
            if isinstance(node.target, ast.Name) and node.target.id == 'global_counter':
                before = ast.parse('__cycle_started = __begin_cycle()').body
                after = ast.parse('__end_cycle(global_counter, __cycle_started)').body
                node.body = before + [ast.Try(body=node.body, handlers=[], orelse=[], finalbody=after)]
            return node

    namespace.update(__begin_cycle=begin_cycle, __end_cycle=end_cycle)
    module = InstrumentCycles().visit(ast.parse((REPO / 'morty/morty.py').read_text()))
    ast.fix_missing_locations(module)
    sys.argv = ['morty', '--press', '--live']
    os.chdir(folder)
    exec(compile(module, str(REPO / 'morty/morty.py'), 'exec'), namespace)


def run_trial(args, config, policy, seed):
    folder = OUTPUT / f'{args.tag}_{policy}_seed{seed}'
    template = dict(config.get('#TEMPLATE', {}))
    template.update(UCD_MC_SELECTION='priority' if policy == 'priority' else 'adaptive',
                    UCD_NUM_RUNS=1, UCD_STEPS_PER_RUN=args.steps,
                    UCD_MIN_ITERATIONS=0, UCD_ALLOW_BLIND_STEPS=0, UCD_ALLOW_CRASHED_STEPS=0,
                    UCD_EVALUATION_POLICY=policy, UCD_EVALUATION_SEED=seed,
                    UCD_EVALUATION_TIMEOUT_SECONDS=args.timeout,
                    UCD_EVALUATION_CONFIGS=';'.join(map(str, args.configs)) if args.configs is not None else '')
    prepare_trial(folder, {**config, '#TEMPLATE': template})
    command = [sys.executable, '-m', 'morty.evaluate_mc_selection', '--child', '--folder', str(folder),
               '--tag', args.tag]
    print(f'START {policy} seed={seed} folder={folder}', flush=True)
    with (folder / 'run.log').open('w') as log:
        process = subprocess.Popen(command, cwd=REPO, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        timed_out = False
        try:
            process.communicate(timeout=template['UCD_EVALUATION_TIMEOUT_SECONDS'])
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.communicate()
    path = folder / 'metrics.json'
    records = json.loads(path.read_text()) if path.exists() else []
    result = summarize_trial(records, process.returncode, timed_out)
    clearances = [record['clearance']['distance_m'] for record in records if record['clearance'] is not None]
    result.update(policy=policy, seed=seed, configuration_pool=args.configs,
                  minimum_clearance_m=min(clearances) if clearances else None,
                  selected_priorities=dict(Counter(record['selected'] for record in records)),
                  failed_queries=sum(not query['success'] for record in records for query in record['queries']))
    write_json(folder / 'result.json', result)
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description='Isolated, sequential MC selection and safety-margin evaluation')
    parser.add_argument('--config', type=Path, default=REPO / 'morty/envmodel_config.tpl.json')
    parser.add_argument('--policies', nargs='+', choices=['adaptive', 'priority', 'cascade'], default=['adaptive'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[0])
    parser.add_argument('--configs', nargs='+', type=int, help='Allowed original priority indices; default: all')
    parser.add_argument('--shortcuts', choices=['on', 'off'])
    parser.add_argument('--steps', type=int, default=50)
    parser.add_argument('--timeout', type=float, default=300, help='Wall-clock limit per trial, including startup')
    parser.add_argument('--tag', required=True, help='New experiment name; existing trial directories are never overwritten')
    parser.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--folder', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.child:
        evaluate_child(args)
        return
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.tag):
        parser.error('--tag must contain only letters, digits, underscores or hyphens')
    if args.steps < 1 or args.timeout <= 0 or min(args.seeds) < 0:
        parser.error('Steps and timeout must be positive; seeds must be nonnegative')
    if len(set(args.seeds)) != len(args.seeds) or len(set(args.policies)) != len(args.policies):
        parser.error('Repeated seeds or policies would overwrite a trial')
    if sys.platform != 'linux' or not Path('/usr/bin/time').is_file():
        parser.error('This evaluator requires Linux and /usr/bin/time')
    try:
        import shapely
    except ImportError:
        parser.error('Clearance measurements require Shapely: python -m pip install "shapely>=2,<3"')
    config = json.loads(args.config.read_text())
    if args.shortcuts is not None:
        config['#TEMPLATE']['UCD_USE_SHORTCUT'] = args.shortcuts == 'on'
    count = len([name for name in config['#TEMPLATE']['UCD_CONFIG_PRIOS'].split(';') if name])
    if args.configs is not None and (min(args.configs) < 0 or max(args.configs) >= count):
        parser.error('Configuration indices must refer to the original priority list')
    results = []
    for seed in args.seeds:
        for policy in args.policies:
            results.append(run_trial(args, config, policy, seed))
    write_json(OUTPUT / f'{args.tag}_summary.json', results)


if __name__ == '__main__':
    main()