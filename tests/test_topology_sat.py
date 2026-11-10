"""gamma-SAT topology enumeration for the queue pipeline.

Run from the repository root:  python -m unittest tests.test_topology_sat
The oracle tests need nuXmv on PATH; set VFM_SAT_SOLVERS=z3,msat to choose backends.

The static constraints come from the shared concrete compiler, with vehicle
statements dropped, so they over-approximate the topologies whose queue model
has an initial state. The nuXmv oracle checks that no such topology is missed.
"""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from pysmt.shortcuts import Int, substitute

from vfm_grid_python.approaches import external_enumeration
from vfm_grid_python.approaches.abstraction_based.abstract import render
from vfm_grid_python.common import check
from vfm_grid_python.geometry import build
from vfm_grid_python.benchmark_timing import JOURNAL
from vfm_grid_python.concrete.trace_model import parse
from vfm_grid_python.scenarios.families import scenario
from vfm_grid_python.scenarios.topologies import (layouts, placement_constraints, sat_layouts,
                                                  static_problem)
from vfm_grid_python.scenarios.topology_sat import StaticProblem, Unsupported
from vfm_grid_python.scenarios.validation import normalize

SOLVERS = os.environ.get('VFM_SAT_SOLVERS', 'z3,msat').split(',')
CASES = [('parking', 2, 2, 1), ('row_reversal', 2, 2, 2),
         ('shared_gate', 2, 3, 2), ('shared_bay', 2, 3, 2)]


def config_for(family, rows, columns, cars):
    return normalize(scenario(family, rows, columns, cars))


def key(graph):
    return (frozenset(tuple(e) for e in graph['edges']),
            frozenset(n['id'] for n in graph['nodes'] if n['kind'] == 'P'))


def consistent(config, graph, directory, eta=None):
    """nuXmv bound-0 check: does the queue abstraction have an initial state here?"""
    fixed = dict(config, scenario_stages=[], edges=graph['edges'],
                 labels={str(n['id']): n['kind'] for n in graph['nodes']})
    if eta is not None:
        fixed['initial_constraints'] = fixed.get('initial_constraints', []) + placement_constraints(eta)
    text = render(fixed)
    assert text.count('INVARSPEC !all_parked;') == 1
    Path(directory).mkdir(parents=True)
    model = Path(directory)/'static.smv'
    model.write_text(text.replace('INVARSPEC !all_parked;', 'INVARSPEC FALSE;'))
    result = check(model, 0, 120)
    if result['status'] not in ('witness', 'no_witness_within_bound'):
        raise RuntimeError(f'oracle failed: {result} in {directory}')
    return result['status'] == 'witness'


class Translation(unittest.TestCase):
    def test_arithmetic(self):
        p = StaticProblem({'x': (0, 20)}, {'d': 'case x mod 3 = 0 : x / 3; TRUE : abs(x - 7); esac'}, [], parse)
        for x in range(21):
            term, _ = p.term(parse('d + min(x, 4)'))
            values = {p.symbols['x']: Int(x)}
            values.update({q: Int(x // c) for (t, c), (q, r) in p._divisions.items()})
            values.update({r: Int(x % c) for (t, c), (q, r) in p._divisions.items()})
            expected = (x // 3 if x % 3 == 0 else abs(x - 7)) + min(x, 4)
            self.assertEqual(substitute(term, values).simplify().constant_value(), expected)
        with self.assertRaises(Unsupported):
            StaticProblem({'x': (0, 9)}, {}, [('INIT', '(x - 30) mod 3 = 0')], parse)

    def test_static_fragment(self):
        p = static_problem(config_for('shared_gate', 3, 3, 2))
        self.assertTrue(p.dropped, 'vehicle statements should be dropped')
        for statement in p.dropped:
            self.assertTrue(any(n.startswith(('env.veh', 'env.section', 'road_', 'qpos_', 'joint_',
                                              'loc_', 'scenario_phase', 'rank_'))
                                for n in statement['nonstatic']), statement)
        self.assertIn('edge_0_1', p.declarations)
        self.assertEqual(p.declarations['start_0'], (0, 8))


class OracleAgreement(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('nuXmv'):
            raise unittest.SkipTest('nuXmv not on PATH')
        cls.root = Path(tempfile.mkdtemp(prefix='sat-oracle-queue-'))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root, ignore_errors=True)

    def oracle(self, config, name):
        graphs = list(layouts(config['rows'], config['columns'], config['parkings'], 0))
        with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
            verdicts = list(pool.map(lambda ig: consistent(config, ig[1], self.root/name/f'{ig[0]:05d}'),
                                     enumerate(graphs)))
        return {key(g) for g, ok in zip(graphs, verdicts) if ok}

    def test_topology_projection_is_complete(self):
        for family, rows, columns, cars in CASES:
            config = config_for(family, rows, columns, cars)
            expected = self.oracle(config, f'{family}_{rows}x{columns}')
            self.assertTrue(expected, f'{family}: oracle found no consistent topology')
            for solver in SOLVERS:
                with self.subTest(family=family, solver=solver):
                    found = [key(g) for g in sat_layouts(config, seed=3, solver=solver)]
                    self.assertEqual(len(found), len(set(found)), 'repeated topology')
                    self.assertLessEqual(expected, set(found), 'missed a consistent topology')
                    extra = len(set(found) - expected)
                    print(f'\n{family} {rows}x{columns} {solver}: {len(expected)} consistent, '
                          f'{extra} extra SAT candidates', file=sys.stderr)

    def test_placement_projection(self):
        config = config_for('shared_gate', 2, 3, 2)
        for solver in SOLVERS:
            with self.subTest(solver=solver):
                graphs = list(sat_layouts(config, solver=solver, projection='placement'))
                points = [(key(g), repr(g['eta'])) for g in graphs]
                self.assertEqual(len(points), len(set(points)), 'repeated placement')
                self.assertEqual({p[0] for p in points},
                                 {key(g) for g in sat_layouts(config, solver=solver)})

    def test_unsatisfiable_order_and_geometry(self):
        config = dict(config_for('fleet_yield', 2, 3, 2), parkings=4)
        self.assertEqual(list(sat_layouts(config)), [])
        program = ('import itertools; from vfm_grid_python.scenarios.families import scenario; '
                   'from vfm_grid_python.scenarios.validation import normalize; '
                   'from vfm_grid_python.scenarios.topologies import sat_layouts; '
                   "print([g['edges'] for g in itertools.islice(sat_layouts("
                   "normalize(scenario('shared_bay', 3, 3, 2)), seed=11, solver=%r), 10)])")
        for solver in SOLVERS:
            runs = {subprocess.run([sys.executable, '-c', program % solver], check=True,
                                   capture_output=True, text=True).stdout for _ in range(2)}
            self.assertEqual(len(runs), 1, f'{solver}: seeded order differs between processes')
        for graph in sat_layouts(config_for('shared_bay', 2, 3, 2)):
            build(graph)


class ApproachIntegration(unittest.TestCase):
    """external_enumeration.run with the SAT enumerator; concrete checks stubbed."""

    def run_approach(self, config, projection, verdicts, **options):
        root = Path(tempfile.mkdtemp(prefix='sat-approach-'))
        self.addCleanup(shutil.rmtree, root, ignore_errors=True)
        calls = []

        def fake_fixed(case, graph, directory, bound, timeout):
            calls.append((case, graph))
            return dict(status=verdicts[min(len(calls)-1, len(verdicts)-1)])
        with mock.patch.object(external_enumeration, 'fixed', fake_fixed), \
             mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(JOURNAL, None)
            result = external_enumeration.run(config, root/'run', 10, 60, 0, options.get('limit', 100),
                                              'sat', projection, 'z3')
        return result, calls, root/'run'

    def test_first_witness_and_timing(self):
        config = config_for('shared_gate', 2, 3, 2)
        result, calls, out = self.run_approach(config, 'topology', ['no_witness_within_bound', 'witness'])
        self.assertEqual((result['status'], result['topologies_checked']), ('witness', 2))
        self.assertGreater(result['timings_seconds'].get('enumeration', 0), 0)
        self.assertTrue((out/'enumeration/static_constraints.json').exists())
        self.assertTrue(all(case is config for case, _ in calls))

    def test_placement_fixes_eta(self):
        config = config_for('shared_gate', 2, 3, 2)
        _, calls, _ = self.run_approach(config, 'placement', ['witness'])
        case, graph = calls[0]
        fixed = placement_constraints(graph['eta'])
        self.assertEqual(case['initial_constraints'][-len(fixed):], fixed)

    def test_exhaustion_unsatisfiable_and_limit(self):
        config = config_for('parking', 2, 2, 1)
        result, calls, _ = self.run_approach(config, 'topology', ['no_witness_within_bound'])
        self.assertEqual(result['status'], 'all_layouts_exhausted_within_bound')
        result, calls, _ = self.run_approach(config, 'topology', ['no_witness_within_bound'], limit=2)
        self.assertEqual((result['status'], len(calls)), ('topology_limit', 2))
        config = dict(config_for('fleet_yield', 2, 3, 2), parkings=4)
        result, calls, _ = self.run_approach(config, 'topology', ['witness'])
        self.assertEqual((result['status'], calls), ('gamma_unsatisfiable', []))
        with self.assertRaises(ValueError):
            external_enumeration.run(config, Path(tempfile.mkdtemp())/'x', 10, 60, 0, 1, 'blind', 'placement')


if __name__ == '__main__':
    unittest.main()
