"""Enumerate grid topologies satisfying the static scenario constraints (gamma).

The constraints are NOT re-encoded here. A caller passes the static part of a
model it already generates (declarations, DEFINEs and INIT/INVAR statements as
SMV text) together with its own SMV expression parser. This module keeps the
statements whose variables are all static (topology and placement variables,
possibly via DEFINEs and aliases), translates them to PySMT, and enumerates
solutions with blocking clauses. It has no other dependency on the package,
so the same file can be used by different versions of the pipeline.

Projection:
  * ``topology``:  distinct (edges, parking labels); placements stay symbolic.
  * ``placement``: distinct (edges, labels, starts, goals, roles).

Integer ``/`` and ``mod`` by a positive constant are encoded exactly with
auxiliary quotient/remainder variables. Only non-negative dividends are
accepted (node IDs), where SMV's and floor semantics coincide.
"""
from __future__ import annotations

import contextlib
import re

from pysmt.shortcuts import (And, Bool, Equals, FreshSymbol, GE, GT, Iff, Implies, Int, Ite,
                             LE, LT, Max, Min, Minus, Not, Or, Plus, Solver, Symbol, Times)
from pysmt.typing import BOOL, INT

TOPOLOGY = re.compile(r'(?:edge_\d+_\d+|parking_\d+)$')
PLACEMENT = re.compile(r'(?:start_\d+|goal_\d+|role_\w+)$')
PROJECTIONS = ('topology', 'placement')


class Unsupported(ValueError):
    """A static statement uses SMV syntax outside the supported fragment."""


def declaration(domain):
    """Map an SMV domain string to 'boolean' or an integer interval."""
    domain = domain.strip()
    if domain == 'boolean':
        return 'boolean'
    match = re.fullmatch(r'(-?\d+)\s*\.\.\s*(-?\d+)', domain)
    if not match:
        raise Unsupported(f'unsupported domain: {domain}')
    lo, hi = map(int, match.groups())
    return (lo, hi)


class StaticProblem:
    """Static constraints of one scenario, ready to be enumerated.

    declarations: {name: 'boolean' | (lo, hi)}; only these are free variables.
    defines:      {name: SMV expression text}
    statements:   [(kind, SMV expression text)], kind in INIT/INVAR
    parse:        the caller's SMV expression parser (text -> AST tuples)
    aliases:      {name: declared name}, e.g. loc_i -> start_i (INIT loc_i = start_i)
    """

    def __init__(self, declarations, defines, statements, parse, aliases=None):
        self.declarations = dict(declarations)
        self.aliases = dict(aliases or {})
        self.parse = parse
        self.define_sources = dict(defines)
        self._define_ast, self._define_names, self._define_terms = {}, {}, {}
        self._divisions, self.side = {}, []
        self.symbols = {name: Symbol(name, BOOL if kind == 'boolean' else INT)
                        for name, kind in self.declarations.items()}
        self.kept, self.dropped = [], []
        for kind, source in statements:
            if kind not in ('INIT', 'INVAR'):
                raise ValueError(f'unexpected statement kind {kind}')
            ast = parse(source)
            unknown = sorted(n for n in self.names(ast) if n not in self.declarations)
            if unknown:
                self.dropped.append(dict(kind=kind, source=source, nonstatic=unknown[:10]))
            else:
                self.kept.append(dict(kind=kind, source=source, ast=ast))
        # Translate eagerly so unsupported syntax fails before any solving.
        self.formulas = [self.boolean(s['ast']) for s in self.kept]
        for name, kind in self.declarations.items():
            if kind != 'boolean':
                self.formulas.append(And(LE(Int(kind[0]), self.symbols[name]),
                                         LE(self.symbols[name], Int(kind[1]))))

    # -- free names -------------------------------------------------------
    def _define(self, name):
        if name not in self._define_ast:
            self._define_ast[name] = self.parse(self.define_sources[name])
        return self._define_ast[name]

    def names(self, ast):
        tag = ast[0]
        if tag == 'literal':
            return set()
        if tag == 'name':
            name = self.aliases.get(ast[1], ast[1])
            if name in self.declarations:
                return {name}
            if name in self.define_sources:
                if name not in self._define_names:
                    self._define_names[name] = set()  # guards against cycles
                    self._define_names[name] = self.names(self._define(name))
                return self._define_names[name]
            return {name}
        if tag == 'unary':
            return self.names(ast[2])
        if tag == 'binary':
            return self.names(ast[2]) | self.names(ast[3])
        if tag == 'case':
            return set().union(*(self.names(c) | self.names(v) for c, v in ast[1]))
        if tag in ('call', 'set'):
            return set().union(*(self.names(a) for a in ast[-1]))
        raise Unsupported(f'unknown AST node {tag}')

    # -- translation ------------------------------------------------------
    def boolean(self, ast):
        term, kind = self.term(ast)
        if kind != 'boolean':
            raise Unsupported(f'expected a Boolean expression: {ast}')
        return term

    def integer(self, ast):
        term, kind = self.term(ast)
        if kind == 'boolean':
            raise Unsupported(f'expected an integer expression: {ast}')
        return term, kind

    def term(self, ast):
        """Return (PySMT term, 'boolean' | (lo, hi))."""
        tag = ast[0]
        if tag == 'literal':
            value = ast[1]
            return (Bool(value), 'boolean') if isinstance(value, bool) else (Int(value), (value, value))
        if tag == 'name':
            name = self.aliases.get(ast[1], ast[1])
            if name in self.declarations:
                return self.symbols[name], self.declarations[name]
            if name in self.define_sources:
                if name not in self._define_terms:
                    self._define_terms[name] = self.term(self._define(name))
                return self._define_terms[name]
            raise Unsupported(f'non-static name {name}')
        if tag == 'unary':
            op, operand = ast[1], ast[2]
            if op == '!':
                return Not(self.boolean(operand)), 'boolean'
            term, (lo, hi) = self.integer(operand)
            return (term, (lo, hi)) if op == '+' else (Minus(Int(0), term), (-hi, -lo))
        if tag == 'binary':
            return self.binary(*ast[1:])
        if tag == 'case':
            return self.case(ast[1])
        if tag == 'call':
            return self.call(ast[1], ast[2])
        raise Unsupported(f'unsupported SMV construct: {tag}')

    def binary(self, op, left, right):
        logic = {'&': And, '|': Or, '->': Implies, '<->': Iff,
                 'xor': lambda a, b: Not(Iff(a, b)), 'xnor': Iff}
        if op in logic:
            return logic[op](self.boolean(left), self.boolean(right)), 'boolean'
        if op in ('=', '!='):
            (a, ka), (b, kb) = self.term(left), self.term(right)
            if (ka == 'boolean') != (kb == 'boolean'):
                raise Unsupported(f'mixed Boolean/integer comparison: {op}')
            equal = Iff(a, b) if ka == 'boolean' else Equals(a, b)
            return (equal if op == '=' else Not(equal)), 'boolean'
        if op in ('<', '<=', '>', '>='):
            (a, _), (b, _) = self.integer(left), self.integer(right)
            return {'<': LT, '<=': LE, '>': GT, '>=': GE}[op](a, b), 'boolean'
        if op in ('+', '-'):
            (a, (alo, ahi)), (b, (blo, bhi)) = self.integer(left), self.integer(right)
            if op == '+':
                return Plus(a, b), (alo + blo, ahi + bhi)
            return Minus(a, b), (alo - bhi, ahi - blo)
        if op == '*':
            (a, (alo, ahi)), (b, (blo, bhi)) = self.integer(left), self.integer(right)
            if not (alo == ahi or blo == bhi):
                raise Unsupported('nonlinear multiplication')
            corners = [x * y for x in (alo, ahi) for y in (blo, bhi)]
            return Times(a, b), (min(corners), max(corners))
        if op in ('/', 'mod', '%'):
            return self.division(op, left, right)
        raise Unsupported(f'unsupported operator {op}')

    def division(self, op, left, right):
        x, (lo, hi) = self.integer(left)
        if right[0] != 'literal' or isinstance(right[1], bool) or right[1] <= 0:
            raise Unsupported('division/modulo only by a positive integer constant')
        if lo < 0:
            raise Unsupported('division/modulo of a possibly negative dividend')
        c = right[1]
        key = (x, c)
        if key not in self._divisions:
            q, r = FreshSymbol(INT), FreshSymbol(INT)
            self.side += [Equals(x, Plus(Times(Int(c), q), r)), LE(Int(0), r), LT(r, Int(c))]
            self._divisions[key] = (q, r)
        q, r = self._divisions[key]
        if op == '/':
            return q, (lo // c, hi // c)
        return r, (0, min(c - 1, hi))

    def case(self, branches):
        condition, value = branches[-1]
        if condition != ('literal', True):
            raise Unsupported('case without a final TRUE branch')
        result, kind = self.term(value)
        for condition, value in reversed(branches[:-1]):
            term, other = self.term(value)
            if (kind == 'boolean') != (other == 'boolean'):
                raise Unsupported('case branches of different types')
            if kind != 'boolean':
                kind = (min(kind[0], other[0]), max(kind[1], other[1]))
            result = Ite(self.boolean(condition), term, result)
        return result, kind

    def call(self, function, args):
        if function == 'abs' and len(args) == 1:
            x, (lo, hi) = self.integer(args[0])
            bound = max(abs(lo), abs(hi))
            return Ite(GE(x, Int(0)), x, Minus(Int(0), x)), (0 if lo <= 0 <= hi else min(abs(lo), abs(hi)), bound)
        if function in ('min', 'max') and len(args) == 2:
            (a, (alo, ahi)), (b, (blo, bhi)) = self.integer(args[0]), self.integer(args[1])
            if function == 'min':
                return Min(a, b), (min(alo, blo), min(ahi, bhi))
            return Max(a, b), (max(alo, blo), max(ahi, bhi))
        if function == 'toint' and len(args) == 1:
            return Ite(self.boolean(args[0]), Int(1), Int(0)), (0, 1)
        raise Unsupported(f'unsupported function {function}')

    # -- enumeration --------------------------------------------------------
    def assertions(self):
        return self.formulas + self.side

    def report(self):
        return dict(declarations={k: v if v == 'boolean' else list(v) for k, v in self.declarations.items()},
                    aliases=self.aliases, kept=[dict(kind=s['kind'], source=s['source']) for s in self.kept],
                    dropped=self.dropped)


def projected(problem, projection):
    if projection not in PROJECTIONS:
        raise ValueError(f'projection must be one of {PROJECTIONS}')
    names = [n for n in problem.declarations if TOPOLOGY.match(n)]
    if projection == 'placement':
        names += [n for n in problem.declarations if PLACEMENT.match(n)]
    return sorted(names)


def solutions(problem, projection='topology', solver='z3', seed=0, timer=None):
    """Yield {name: value} for the projected variables, without repetitions.

    ``timer`` is a zero-argument callable returning a context manager that wraps
    every solver call (e.g. a benchmark timing interval). The generator simply
    stops when no further solution exists; a caller that received nothing knows
    that the static constraints are unsatisfiable.
    """
    names = projected(problem, projection)
    timer = timer or contextlib.nullcontext
    if type(seed) is not int or seed < 0:
        raise ValueError('seed must be a non-negative integer')
    # Seed 0 means the backend default. MathSAT rejects random_seed=0 and PySMT
    # does not detect the failed environment creation (it segfaults later).
    options = dict(random_seed=seed) if seed else {}
    with Solver(name=solver, logic='QF_LIA', **options) as engine:
        for formula in problem.assertions():
            engine.add_assertion(formula)
        while True:
            with timer():
                satisfiable = engine.solve()
            if not satisfiable:
                return
            values = {n: engine.get_py_value(problem.symbols[n]) for n in names}
            yield values
            engine.add_assertion(Or([
                Not(problem.symbols[n] if v else Not(problem.symbols[n])) if isinstance(v, bool)
                else Not(Equals(problem.symbols[n], Int(v))) for n, v in values.items()]))


def backend_version(solver):
    try:
        if solver == 'z3':
            import z3
            return 'z3 ' + z3.get_version_string()
        if solver == 'msat':
            import mathsat
            return mathsat.msat_get_version()
    except Exception as exc:  # version metadata must never break enumeration
        return f'{solver} (version unavailable: {exc})'
    return solver
