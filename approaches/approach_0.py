from __future__ import annotations

from benchmarking import BenchmarkResult
from model_checking import Morty

from .planarity_loop import run_planarity_loop


def run(
    morty: Morty,
    *,
    max_iterations: int = 10_000,
    iteration_timeout_seconds: float | None = 60.0,
) -> BenchmarkResult:
    """Enumerate counterexample topologies produced by nuXmv.

    Each iteration asks nuXmv for a counterexample, converts its road topology
    into a graph, checks whether that graph is planar, and writes an HTML
    visualization into the Morty scenario directory. The observed topology is
    then blocked in ``main.smv`` so the next iteration searches for a different
    one. A timed-out iteration is recorded and skipped.

    The returned benchmark result reports elapsed time and completed/time-out
    iteration counts; it does not treat planarity as the definition of a
    successful benchmark iteration.
    """
    return run_planarity_loop(
        morty,
        approach_name="approach_0",
        max_iterations=max_iterations,
        iteration_timeout_seconds=iteration_timeout_seconds,
    )


approach_0 = run
