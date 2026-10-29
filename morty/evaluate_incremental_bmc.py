"""Offline experiments with persistent MathSAT contexts; not a driving backend."""

import argparse
from contextlib import contextmanager
import ctypes
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time


REPO = Path(__file__).resolve().parent.parent


class MathsatHandle(ctypes.Structure):
    _fields_ = [('repr', ctypes.c_void_p)]


class MathsatContext:
    def __init__(self):
        self.library = ctypes.CDLL(str(REPO / 'external/linux64/lib/libnuxmv.so'))
        signatures = {
            'msat_create_config': ([], MathsatHandle),
            'msat_destroy_config': ([MathsatHandle], None),
            'msat_set_option': ([MathsatHandle, ctypes.c_char_p, ctypes.c_char_p], ctypes.c_int),
            'msat_create_env': ([MathsatHandle], MathsatHandle),
            'msat_destroy_env': ([MathsatHandle], None),
            'msat_from_smtlib2': ([MathsatHandle, ctypes.c_char_p], MathsatHandle),
            'msat_from_string': ([MathsatHandle, ctypes.c_char_p], MathsatHandle),
            'msat_annotated_list_from_smtlib2': ([MathsatHandle, ctypes.c_char_p,
                ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.POINTER(MathsatHandle)),
                ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))], ctypes.c_int),
            'msat_term_get_type': ([MathsatHandle], MathsatHandle),
            'msat_get_bool_type': ([MathsatHandle], MathsatHandle),
            'msat_find_decl': ([MathsatHandle, ctypes.c_char_p], MathsatHandle),
            'msat_declare_function': ([MathsatHandle, ctypes.c_char_p, MathsatHandle], MathsatHandle),
            'msat_make_constant': ([MathsatHandle, MathsatHandle], MathsatHandle),
            'msat_term_get_decl': ([MathsatHandle], MathsatHandle),
            'msat_decl_get_name': ([MathsatHandle], ctypes.c_void_p),
            'msat_make_not': ([MathsatHandle, MathsatHandle], MathsatHandle),
            'msat_make_or': ([MathsatHandle, MathsatHandle, MathsatHandle], MathsatHandle),
            'msat_apply_substitution': ([MathsatHandle, MathsatHandle, ctypes.c_size_t,
                ctypes.POINTER(MathsatHandle), ctypes.POINTER(MathsatHandle)], MathsatHandle),
            'msat_assert_formula': ([MathsatHandle, MathsatHandle], ctypes.c_int),
            'msat_push_backtrack_point': ([MathsatHandle], ctypes.c_int),
            'msat_pop_backtrack_point': ([MathsatHandle], ctypes.c_int),
            'msat_solve': ([MathsatHandle], ctypes.c_int),
            'msat_solve_with_assumptions': ([MathsatHandle, ctypes.POINTER(MathsatHandle), ctypes.c_size_t], ctypes.c_int),
            'msat_get_model_value': ([MathsatHandle, MathsatHandle], MathsatHandle),
            'msat_term_repr': ([MathsatHandle], ctypes.c_void_p),
            'msat_free': ([ctypes.c_void_p], None),
            'msat_last_error_message': ([MathsatHandle], ctypes.c_char_p),
        }
        for name, (parameters, result) in signatures.items():
            function = getattr(self.library, name)
            function.argtypes = parameters
            function.restype = result
        self.config = self.library.msat_create_config()
        self.env = MathsatHandle()
        if not self.config.repr:
            raise RuntimeError('Cannot create MathSAT configuration')
        try:
            if self.library.msat_set_option(self.config, b'model_generation', b'true'):
                raise RuntimeError('Cannot enable MathSAT model generation')
            self.env = self.library.msat_create_env(self.config)
            if not self.env.repr:
                raise RuntimeError('Cannot create MathSAT environment')
        except BaseException:
            self.close()
            raise

    def check(self, result):
        if result:
            message = self.library.msat_last_error_message(self.env)
            raise RuntimeError(message.decode() if message else 'MathSAT operation failed')

    def term(self, result):
        if not result.repr:
            self.check(1)
        return result

    def parse(self, source):
        return self.term(self.library.msat_from_smtlib2(self.env, source.encode()))

    def expression(self, source):
        return self.term(self.library.msat_from_string(self.env, source.encode()))

    def annotations(self, source):
        count = ctypes.c_size_t()
        terms = ctypes.POINTER(MathsatHandle)()
        annotations = ctypes.POINTER(ctypes.c_void_p)()
        self.check(self.library.msat_annotated_list_from_smtlib2(
            self.env, source.encode(), ctypes.byref(count), ctypes.byref(terms), ctypes.byref(annotations)))
        try:
            return [(MathsatHandle(terms[index].repr),
                     ctypes.string_at(annotations[2 * index]).decode().lstrip(':'),
                     ctypes.string_at(annotations[2 * index + 1]).decode())
                    for index in range(count.value)]
        finally:
            for index in range(2 * count.value):
                self.library.msat_free(annotations[index])
            self.library.msat_free(annotations)
            self.library.msat_free(terms)

    def symbol(self, name, like=None):
        name = name.strip('|')
        declaration = self.library.msat_find_decl(self.env, name.encode())
        if not declaration.repr:
            if like is not None:
                return self.constant(name, like)
            raise ValueError(f'Unknown MathSAT symbol: {name}')
        return self.term(self.library.msat_make_constant(self.env, declaration))

    def constant(self, name, like=None):
        sort = self.library.msat_term_get_type(like) if like is not None else self.library.msat_get_bool_type(self.env)
        declaration = self.term(self.library.msat_declare_function(self.env, name.encode(), sort))
        return self.term(self.library.msat_make_constant(self.env, declaration))

    def name(self, constant):
        declaration = self.term(self.library.msat_term_get_decl(constant))
        return self.read_string(self.library.msat_decl_get_name(declaration))

    def negate(self, formula):
        return self.term(self.library.msat_make_not(self.env, formula))

    def implies(self, condition, formula):
        return self.term(self.library.msat_make_or(self.env, self.negate(condition), formula))

    def substitute(self, formula, substitutions):
        originals = (MathsatHandle * len(substitutions))(*(pair[0] for pair in substitutions))
        replacements = (MathsatHandle * len(substitutions))(*(pair[1] for pair in substitutions))
        return self.term(self.library.msat_apply_substitution(
            self.env, formula, len(substitutions), originals, replacements))

    def assert_term(self, formula):
        self.check(self.library.msat_assert_formula(self.env, formula))

    @contextmanager
    def scope(self):
        self.check(self.library.msat_push_backtrack_point(self.env))
        try:
            yield
        finally:
            self.check(self.library.msat_pop_backtrack_point(self.env))

    def solve(self, timeout=30, assumptions=()):
        if timeout <= 0:
            raise TimeoutError('MathSAT query budget exhausted')
        if assumptions:
            literals = (MathsatHandle * len(assumptions))(*assumptions)
            result = self.library.msat_solve_with_assumptions(self.env, literals, len(assumptions))
        else:
            result = self.library.msat_solve(self.env)
        if result == -1:
            self.check(1)
        return result == 1

    def value(self, formula):
        value = self.term(self.library.msat_get_model_value(self.env, formula))
        return self.read_string(self.library.msat_term_repr(value))

    def read_string(self, text):
        if not text:
            self.check(1)
        try:
            return ctypes.string_at(text).decode()
        finally:
            self.library.msat_free(text)

    def close(self):
        if self.env.repr:
            self.library.msat_destroy_env(self.env)
            self.env = MathsatHandle()
        if self.config.repr:
            self.library.msat_destroy_config(self.config)
            self.config = MathsatHandle()

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()


class IncrementalBmc:
    def __init__(self, solver, source, bound=30):
        if bound < 0:
            raise ValueError('The bound must be nonnegative')
        started = time.perf_counter()
        self.solver = solver
        self.bound = bound
        annotations = solver.annotations(source)
        self.state_variables = [(formula, solver.symbol(value, formula))
                                for formula, kind, value in annotations if kind == 'next']
        initial = [formula for formula, kind, value in annotations if kind == 'init']
        transitions = [formula for formula, kind, value in annotations if kind == 'trans']
        properties = [formula for formula, kind, value in annotations if kind == 'invar-property']
        if len(initial) != 1 or len(transitions) != 1 or len(properties) != 1:
            raise ValueError('Expected one VMT initial relation, transition relation, and invariant property')
        if not self.state_variables:
            raise ValueError('VMT export has no state variables')
        self.frames = [[current for current, following in self.state_variables]]
        for depth in range(1, bound + 1):
            self.frames.append([current if current.repr == following.repr
                                else solver.constant(f'vfm_frame_{depth}_{index}', current)
                                for index, (current, following) in enumerate(self.state_variables)])
        self.names = [solver.name(current) for current, following in self.state_variables]
        self.active = [solver.constant(f'vfm_active_{depth}') for depth in range(bound + 1)]
        solver.assert_term(initial[0])
        solver.assert_term(self.active[0])
        self.transition = transitions[0]
        self.prepared_depth = 0
        self.query_count = 0
        self.goals = [solver.negate(self.at(properties[0], depth)) for depth in range(bound + 1)]
        self.goal_guards = {}
        self.setup_seconds = time.perf_counter() - started

    def prepare_depth(self, depth):
        while self.prepared_depth < depth:
            self.prepared_depth += 1
            current = self.prepared_depth
            self.solver.assert_term(self.solver.implies(self.active[current], self.active[current - 1]))
            self.solver.assert_term(self.solver.implies(self.active[current], self.at(self.transition, current - 1, True)))
        if depth not in self.goal_guards:
            self.goal_guards[depth] = self.solver.constant(f'vfm_goal_{depth}')
            self.solver.assert_term(self.solver.implies(self.goal_guards[depth], self.goals[depth]))

    def at(self, formula, depth, transition=False):
        substitutions = []
        for index, (current, following) in enumerate(self.state_variables):
            substitutions.append((current, self.frames[depth][index]))
            if transition and current.repr != following.repr:
                substitutions.append((following, self.frames[depth + 1][index]))
        return self.solver.substitute(formula, substitutions)

    @contextmanager
    def at_depth(self, depth):
        if not 0 <= depth <= self.bound:
            raise ValueError('Requested depth is outside the unrolling')
        self.prepare_depth(depth)
        with self.solver.scope():
            self.solver.assert_term(self.active[depth])
            if depth < self.bound:
                self.solver.assert_term(self.solver.negate(self.active[depth + 1]))
            self.solver.assert_term(self.goals[depth])
            yield

    def query(self, bindings=(), timeout=30):
        started = time.perf_counter()
        self.query_count += 1
        query_guard = self.solver.constant(f'vfm_query_{self.query_count}')
        try:
            for binding in bindings:
                self.solver.assert_term(self.solver.implies(query_guard, self.solver.expression(binding)))
            for depth in range(self.bound + 1):
                self.prepare_depth(depth)
                assumptions = [query_guard, self.active[depth], self.goal_guards[depth]]
                if depth < self.bound:
                    assumptions.append(self.solver.negate(self.active[depth + 1]))
                if self.solver.solve(timeout - (time.perf_counter() - started), assumptions):
                    states = [{name: self.solver.value(formula)
                               for name, formula in zip(self.names, frame)}
                              for frame in self.frames[:depth + 1]]
                    return {'outcome': 'sat', 'depth': depth, 'states': states,
                            'seconds': time.perf_counter() - started}
        finally:
            self.solver.assert_term(self.solver.negate(query_guard))
        return {'outcome': 'unsat_bound', 'depth': None, 'states': [],
                'seconds': time.perf_counter() - started}


def parameterize_main(main):
    main = re.sub(r'\n[ \t\r\n]*(?=-- INIT CONSTRAINTS\n)', '\n', main)
    initial = re.search(r'-- INIT CONSTRAINTS\n(.*?)-- EO INIT CONSTRAINTS', main, re.DOTALL)
    if initial is None:
        raise ValueError('Missing native observation block')
    values = {}
    declarations = []
    constraints = []
    for line in initial[1].splitlines():
        if not line.strip():
            continue
        assignment = re.fullmatch(r'\s*INIT ([\w.]+) = (-?\d+);\s*', line)
        boolean = re.fullmatch(r'\s*INIT (!?)([\w.]+);\s*', line)
        parameter = f'vfm_observation_{len(values)}'
        if assignment:
            variable, value = assignment[1], int(assignment[2])
            sort = 'integer'
        elif boolean:
            variable, value = boolean[2], not bool(boolean[1])
            sort = 'boolean'
        else:
            raise ValueError(f'Unsupported observation constraint: {line}')
        values[parameter] = value
        declarations.append(f'{parameter} : {sort};')
        constraints.append(f'INIT {variable} = {parameter};')
    main = main[:initial.start(1)] + '\n'.join(constraints) + '\n' + main[initial.end(1):]
    properties = list(re.finditer(r'^INVARSPEC\s+(.+?);', main, re.MULTILINE | re.DOTALL))
    if len(properties) != 1:
        raise ValueError('Expected exactly one invariant goal')
    property_match = properties[0]

    def replace_number(match):
        parameter = f'vfm_goal_parameter_{len(values)}'
        values[parameter] = int(match[0])
        declarations.append(f'{parameter} : integer;')
        return parameter

    property_source = re.sub(r'(?<![\w.])-?\d+(?![\w.])', replace_number, property_match[1])
    main = main[:property_match.start(1)] + property_source + main[property_match.end(1):]
    return main + '\nFROZENVAR\n' + '\n'.join(declarations) + '\n', values


def export_vmt(package, folder, main):
    folder.mkdir(parents=True, exist_ok=True)
    for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
        shutil.copy2(package / filename, folder / filename)
    (folder / 'main.smv').write_text(main)
    environment = dict(os.environ)
    environment['LD_LIBRARY_PATH'] = str(REPO / 'external/linux64/lib') + os.pathsep + environment.get('LD_LIBRARY_PATH', '')
    commands = 'set on_failure_script_quits 1\ngo_msat\nwrite_vmt_model -n 0 -o model.vmt\nquit\n'
    process = subprocess.run([str(REPO / 'external/linux64/nuXmv'), '-int', '-pre', 'cpp', 'main.smv'],
                             input=commands, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             cwd=folder, env=environment, timeout=30)
    (folder / 'export.log').write_text(process.stdout)
    if process.returncode or not (folder / 'model.vmt').is_file():
        raise RuntimeError(f'VMT export failed: {process.stdout[-2000:]}')
    return (folder / 'model.vmt').read_text()


def parameter_bindings(solver, values):
    bindings = []
    for parameter, value in values.items():
        names = [parameter + suffix for suffix in ('__AT0', '__AT-1', '')]
        declared = [name for name in names if solver.library.msat_find_decl(solver.env, name.encode()).repr]
        if len(declared) != 1:
            raise ValueError(f'Cannot identify frozen parameter {parameter}: {declared}')
        literal = str(value).lower() if isinstance(value, bool) or value >= 0 else f'(- {-value})'
        bindings.append(f'(= |{declared[0]}| {literal})')
    return bindings


def model_key(package, parameterized):
    digest = hashlib.sha256(parameterized.encode())
    for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
        digest.update(b'\0' + (package / filename).read_bytes())
    return digest.hexdigest()


def prepare_recording(recording, folder, limit=None):
    from .evaluate_mc_selection import prepare_trial
    from .morty_helper import morty_script_context

    config = json.loads((recording / 'morty/envmodel_config.tpl.json').read_text())
    template = config['#TEMPLATE']
    if (template['UCD_EXP_NUM'] != 7 or not template.get('UCD_TASK7_COMPACT')
            or template['NUMLANES'] + template['LATERAL_LC_GRANULARITY'] != 12):
        raise ValueError('Replay currently supports compact task 7 with twelve technical lanes')
    queries = [json.loads(line) for line in (recording / 'queries.jsonl').read_text().splitlines()]
    queries = [query for query in queries if query['priority'] == 0][:limit]
    if not queries:
        raise ValueError('No highest-priority queries in the recording')
    name = queries[0]['config']
    if any(query['config'] != name for query in queries):
        raise ValueError('The highest-priority configuration changed in this recording')
    template['UCD_CONFIG_PRIOS'] = name
    work = folder / 'native_inputs'
    prepare_trial(work, config)
    package = work / ('generated' + name)
    package.mkdir()
    for filename in ('main.smv', 'EnvModel.smv', 'planner.cpp_combined.k2.smv',
                     'scaling_info.txt', 'debug_trace_array.txt'):
        shutil.copy2(recording / ('generated' + name) / filename, package / filename)
    original_directory = Path.cwd()
    try:
        os.chdir(work)
        with morty_script_context(live=True) as library:
            for index, query in enumerate(queries):
                buffer = ctypes.create_string_buffer(2000)
                script = ('@{ @{./src/templates/}@.stringToHeap[MY_PATH]\n'
                          f'@{{{query["model_input"]}}}@.prepareInputForMortyUCD['
                          f'{template["UCD_HEADING_ADAPTATION"]}, {template["NUMLANES"]}, 12, '
                          f'./generated{name}/debug_trace_array.txt] }}@.nil')
                library.expandScript(script.encode(), buffer, ctypes.sizeof(buffer))
                main, replacements = re.subn(r'^INVARSPEC\s+.+?;', lambda match: query['target'],
                                             (package / 'main.smv').read_text(), flags=re.MULTILINE | re.DOTALL)
                if replacements != 1:
                    raise ValueError('Expected one goal in the prepared native input')
                destination = folder / 'queries' / f'{index:03d}'
                destination.mkdir(parents=True)
                for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
                    shutil.copy2(package / filename, destination / filename)
                (destination / 'main.smv').write_text(main)
                (destination / 'query.json').write_text(json.dumps(query, indent=2))
    finally:
        os.chdir(original_directory)
    return queries


def project_witness(states, main):
    from .evaluate_smv_encodings import read_trace

    lines = []
    for index, state in enumerate(states):
        lines.append(f'-> State: 1.{index + 1} <-')
        for name, value in state.items():
            match = re.fullmatch(r'(env\.veh___6\d+9___\.(?:abs_pos|v|lane_coordinate))__AT0', name)
            if not match:
                continue
            negative = re.fullmatch(r'\(- (\d+)\)', value)
            value = '-' + negative[1] if negative else value
            if not re.fullmatch(r'-?\d+', value):
                raise ValueError(f'Unexpected integer model value: {value}')
            lines.append(f'  {match[1]} = {value}')
    projected = read_trace('\n'.join(lines), main)
    return [{name: value for name, value in state.items()
             if re.search(r'\.(?:abs_pos|v|lane_b\d+)$', name)} for state in projected]


def bmc_worker(connection, source, bound):
    try:
        with MathsatContext() as solver:
            model = IncrementalBmc(solver, source, bound)
            connection.send({'setup_seconds': model.setup_seconds})
            while True:
                values = connection.recv()
                if values is None:
                    break
                result = model.query(parameter_bindings(solver, values), timeout=float('inf'))
                connection.send({'result': result})
    except BaseException as error:
        try:
            connection.send({'error': f'{type(error).__name__}: {error}'})
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class BmcWorker:
    def __init__(self, source, bound=30):
        self.source = source
        self.bound = bound
        self.process = None
        self.connection = None
        self.start()

    def start(self):
        started = time.perf_counter()
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=bmc_worker, args=(child, self.source, self.bound))
        try:
            self.process.start()
            child.close()
            self.receive(30)
        except BaseException:
            child.close()
            self.abort()
            raise
        self.setup_seconds = time.perf_counter() - started

    def receive(self, timeout):
        if timeout <= 0 or not self.connection.poll(timeout):
            self.abort()
            raise TimeoutError('MathSAT worker deadline expired')
        try:
            response = self.connection.recv()
        except (EOFError, OSError) as error:
            self.abort()
            raise RuntimeError('MathSAT worker exited without a result') from error
        if 'error' in response:
            self.abort()
            raise RuntimeError(response['error'])
        return response

    def query(self, values=None, timeout=30):
        started = time.perf_counter()
        try:
            if self.process is None:
                self.start()
            self.connection.send(values or {})
            response = self.receive(timeout - (time.perf_counter() - started))
        except TimeoutError:
            return {'outcome': 'timeout', 'depth': None, 'states': [],
                    'seconds': time.perf_counter() - started}
        result = response['result']
        result['seconds'] = time.perf_counter() - started
        return result

    def abort(self):
        if self.process is not None:
            if self.process.is_alive():
                self.process.kill()
            if self.process.pid is not None:
                self.process.join()
            self.process = None
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def close(self):
        if self.process is not None and self.process.is_alive():
            try:
                self.connection.send(None)
            except (BrokenPipeError, EOFError, OSError):
                pass
            self.process.join(timeout=1)
        self.abort()

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()


def replay_recording(recording, folder, bound=30, limit=None):
    from .evaluate_mc_selection import write_json
    from .evaluate_smv_encodings import pin_trace, run_solver

    queries = prepare_recording(recording, folder, limit)
    contexts = {}
    results = []
    try:
        for index, query in enumerate(queries):
            package = folder / 'queries' / f'{index:03d}'
            main = (package / 'main.smv').read_text()
            parameterized, values = parameterize_main(main)
            key = model_key(package, parameterized)
            native, output = run_solver(package, bound=bound, timeout=30)
            (package / 'native.log').write_text(output)
            if native['outcome'] not in ('sat', 'unsat_bound'):
                raise AssertionError(f'Native reference failed: {native}')
            if (native['outcome'] == 'sat') != query['success']:
                raise AssertionError(f'Recorded input replay disagrees with the original query: {index}')
            started = time.perf_counter()
            cache_hit = key in contexts and contexts[key].process is not None
            if key not in contexts:
                source = export_vmt(package, folder / 'exports' / key, parameterized)
                contexts[key] = BmcWorker(source, bound)
            worker = contexts[key]
            warm = worker.query(values)
            warm['total_seconds'] = time.perf_counter() - started
            started = time.perf_counter()
            with BmcWorker(worker.source, bound) as cold_worker:
                cold = cold_worker.query(values)
            cold['total_seconds'] = time.perf_counter() - started
            for label, result in (('warm', warm), ('cold', cold)):
                if result['outcome'] != 'timeout' and (result['outcome'], result['depth']) != (native['outcome'], native['depth']):
                    raise AssertionError(f'{label} query {index} disagrees: {result["outcome"]}, {result["depth"]} vs {native}')
            witness = None
            if warm['outcome'] == 'sat':
                replay = package / 'witness'
                replay.mkdir()
                for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
                    shutil.copy2(package / filename, replay / filename)
                (replay / 'main.smv').write_text(pin_trace(main, project_witness(warm['states'], main)))
                witness, output = run_solver(replay, bound=warm['depth'], timeout=30)
                (replay / 'solver.log').write_text(output)
                if (witness['outcome'], witness['depth']) != ('sat', warm['depth']):
                    raise AssertionError(f'Reused witness failed validation: {witness}')
            warm.pop('states')
            cold.pop('states')
            entry = {'index': index, 'iteration': query['iteration'], 'shortcut': query['shortcut'],
                     'key': key, 'cache_hit': cache_hit, 'native': native, 'cold': cold, 'warm': warm,
                     'witness': witness}
            results.append(entry)
            write_json(folder / 'results.json', results)
            print(f'Query {index}, cycle {query["iteration"] + 1}, {"shortcut" if query["shortcut"] else "full"}, '
                  f'{native["outcome"]} depth {native["depth"]}: native {native["seconds"]:.3f}s, '
                  f'cold {cold["total_seconds"]:.3f}s, warm {warm["total_seconds"]:.3f}s '
                  f'({"hit" if cache_hit else "miss"})', flush=True)
    finally:
        for worker in contexts.values():
            worker.close()
    return results


def check_context_isolation():
    with MathsatContext() as solver:
        solver.assert_term(solver.parse('''
(declare-fun position () Int)
(declare-fun destination () Int)
(assert (= destination (+ position 1)))
(assert (>= position 0))
'''))
        for position, destination, expected in ((1, 2, True), (1, 3, False), (2, 3, True)):
            with solver.scope():
                solver.assert_term(solver.expression(f'(= position {position})'))
                solver.assert_term(solver.expression(f'(= destination {destination})'))
                assert solver.solve() == expected
                if expected:
                    assert solver.value(solver.expression('destination')) == str(destination)
        with solver.scope():
            solver.assert_term(solver.expression('false'))
            try:
                solver.solve(timeout=0)
            except TimeoutError:
                pass
            else:
                raise AssertionError('An exhausted budget must not produce a result')
        assert solver.solve()
    print('MathSAT context isolation, model extraction, and exhausted-budget cleanup pass.')


def check_bmc_prefixes():
    source = '''
(declare-fun position () Int)
(declare-fun following () Int)
(declare-fun unused_state () Bool)
(define-fun state () Int (! position :next following))
(define-fun unused () Bool (! unused_state :next unused_following))
(define-fun initial () Bool (! (= position 0) :init true))
(define-fun transition () Bool (! (and (< position 1) (= following (+ position 1))) :trans true))
(define-fun property () Bool (! (< position 1) :invar-property 0))
(assert true)
'''
    with MathsatContext() as solver:
        model = IncrementalBmc(solver, source, bound=2)
        assert model.query()['depth'] == 1
        for depth, expected in ((1, True), (2, False), (1, True)):
            with model.at_depth(depth):
                assert solver.solve() == expected
    print('Guarded BMC unrolling preserves short witnesses after deeper dead ends.')


def check_parameterization():
    source = '''MODULE main
VAR position : integer;
INVARSPEC !(position >= 12);
-- INIT CONSTRAINTS
INIT position = -3;
INIT !lane_b1;
-- EO INIT CONSTRAINTS
'''
    transformed, values = parameterize_main(source)
    changed, other_values = parameterize_main(source.replace('>= 12', '>= 15').replace('= -3;', '= 2;')
                                             .replace('\n-- INIT CONSTRAINTS', '\n\n-- INIT CONSTRAINTS'))
    assert transformed == changed and values != other_values
    assert values == {'vfm_observation_0': -3, 'vfm_observation_1': False, 'vfm_goal_parameter_2': 12}
    assert 'INIT lane_b1 = vfm_observation_1;' in transformed
    with tempfile.TemporaryDirectory(prefix='vfm-key-test-') as temporary:
        package = Path(temporary)
        for filename in ('EnvModel.smv', 'planner.cpp_combined.k2.smv'):
            (package / filename).write_text('original')
        original_key = model_key(package, transformed)
        assert original_key == model_key(package, changed)
        assert original_key != model_key(package, transformed + '\nINVAR position < 10;')
        (package / 'EnvModel.smv').write_text('changed geometry or configuration')
        assert original_key != model_key(package, transformed)
    print('Observation and goal changes preserve the parameterized model structure.')


def check_query_isolation():
    source = '''
(declare-fun position () Int)
(declare-fun following () Int)
(declare-fun observation () Int)
(declare-fun target () Int)
(define-fun state () Int (! position :next following))
(define-fun observed () Int (! observation :next observation))
(define-fun goal () Int (! target :next target))
(define-fun initial () Bool (! (= position observation) :init true))
(define-fun transition () Bool (! (and (< position 1) (= following (+ position 1))) :trans true))
(define-fun property () Bool (! (< position target) :invar-property 0))
(assert true)
'''
    with MathsatContext() as solver:
        model = IncrementalBmc(solver, source, bound=2)
        for observation, target, outcome, depth in [(0, 1, 'sat', 1), (0, 3, 'unsat_bound', None),
                                                     (1, 1, 'sat', 0), (0, 1, 'sat', 1)]:
            result = model.query([f'(= observation {observation})', f'(= target {target})'])
            assert (result['outcome'], result['depth']) == (outcome, depth), result
        try:
            model.query(['false'], timeout=0)
        except TimeoutError:
            pass
        else:
            raise AssertionError('An exhausted query must time out')
        assert model.query(['(= observation 0)', '(= target 1)'])['depth'] == 1
    print('Changing observations/goals and timeouts do not leak across queries.')
    with BmcWorker(source, bound=100) as worker:
        original_pid = worker.process.pid
        first = worker.query({'observation': 0, 'target': 1})
        assert (first['outcome'], first['depth']) == ('sat', 1), first
        second = worker.query({'observation': 1, 'target': 1})
        assert (second['outcome'], second['depth']) == ('sat', 0), second
        assert worker.process.pid == original_pid
        interrupted = worker.query({'observation': 0, 'target': 1000}, timeout=0.000001)
        assert interrupted['outcome'] == 'timeout' and worker.process is None, interrupted
        recovered = worker.query({'observation': 0, 'target': 1})
        assert (recovered['outcome'], recovered['depth']) == ('sat', 1), recovered
        assert worker.process.pid != original_pid
    print('Worker reuse, hard timeout disposal, and clean restart pass.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--vmt', type=Path)
    parser.add_argument('--package', type=Path)
    parser.add_argument('--recording', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--limit', type=int)
    parser.add_argument('--bound', type=int, default=30)
    args = parser.parse_args()
    if args.self_test:
        check_context_isolation()
        check_bmc_prefixes()
        check_parameterization()
        check_query_isolation()
    elif args.recording:
        if not args.output:
            parser.error('--recording requires a new --output folder')
        replay_recording(args.recording.resolve(), args.output.resolve(), args.bound, args.limit)
    elif args.package:
        package = args.package.resolve()
        parameterized, values = parameterize_main((package / 'main.smv').read_text())
        with tempfile.TemporaryDirectory(prefix='vfm-incremental-') as temporary:
            source = export_vmt(package, Path(temporary), parameterized)
            with BmcWorker(source, args.bound) as worker:
                result = worker.query(values)
                result.pop('states')
                result['setup_seconds'] = worker.setup_seconds
                print(json.dumps(result, indent=2))
    elif args.vmt:
        with BmcWorker(args.vmt.read_text(), args.bound) as worker:
            result = worker.query()
            result.pop('states')
            result['setup_seconds'] = worker.setup_seconds
            print(json.dumps(result, indent=2))
    else:
        parser.error('Specify --self-test, --recording, --package, or --vmt')


if __name__ == '__main__':
    main()