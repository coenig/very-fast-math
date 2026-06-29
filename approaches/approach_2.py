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
    """Run the legacy planarity-counterexample loop used by approach 2.

    This currently delegates to the same topology extraction, planarity check,
    visualization, blocking, timeout, and benchmark accounting implementation
    as approach 0. It remains available for compatibility but is not included
    in the default parallel benchmark set.
    """
    return run_planarity_loop(
        morty,
        approach_name="approach_2",
        max_iterations=max_iterations,
        iteration_timeout_seconds=iteration_timeout_seconds,
    )


approach_2 = run
