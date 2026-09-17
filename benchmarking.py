from __future__ import annotations

import json
import signal
import time
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterator


class IterationTimeoutError(TimeoutError):
    pass


@dataclass(frozen=True)
class BenchmarkResult:
    approach: str
    elapsed_seconds: float
    attempted_runs: int
    successful_runs: int
    timed_out_runs: int
    completed: bool

    def as_dict(self) -> dict:
        return asdict(self)


class BenchmarkCounter:
    def __init__(self, approach: str) -> None:
        self.approach = approach
        self.started_at = time.perf_counter()
        self.attempted_runs = 0
        self.successful_runs = 0
        self.timed_out_runs = 0

    def attempted(self) -> None:
        self.attempted_runs += 1

    def succeeded(self) -> None:
        self.successful_runs += 1

    def timed_out(self) -> None:
        self.timed_out_runs += 1

    def result(self, *, completed: bool) -> BenchmarkResult:
        return BenchmarkResult(
            approach=self.approach,
            elapsed_seconds=time.perf_counter() - self.started_at,
            attempted_runs=self.attempted_runs,
            successful_runs=self.successful_runs,
            timed_out_runs=self.timed_out_runs,
            completed=completed,
        )


def save_benchmark_result(result: BenchmarkResult, output_path: Path) -> None:
    output_path.mkdir(parents=True, exist_ok=True)
    (output_path / "benchmark_result.json").write_text(
        json.dumps(result.as_dict(), indent=2) + "\n"
    )


@contextmanager
def iteration_timeout(seconds: float | None) -> Iterator[None]:
    if seconds is None:
        yield
        return
    if seconds <= 0:
        raise ValueError("iteration timeout must be positive")

    def raise_timeout(signum, frame) -> None:
        del signum, frame
        raise IterationTimeoutError(
            f"iteration exceeded its {seconds:g}-second timeout"
        )

    previous_handler = signal.getsignal(signal.SIGALRM)
    previous_timer = signal.getitimer(signal.ITIMER_REAL)
    signal.signal(signal.SIGALRM, raise_timeout)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, *previous_timer)
        signal.signal(signal.SIGALRM, previous_handler)
