"""Process-local benchmark accounting, shared with native child processes.

The journal survives timeouts. Setup, solver commands and trace export are
separate intervals; neither flattening nor XML import is counted as solving.
Only the benchmark worker sets these environment variables.
"""
from __future__ import annotations

import contextlib
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time
import uuid

JOURNAL = "VFM_BENCHMARK_JOURNAL"
DEADLINE = "VFM_BENCHMARK_DEADLINE"
PHASE = "VFM_BENCHMARK_PHASE"


def remaining(timeout):
    deadline = os.environ.get(DEADLINE)
    return max(0., min(timeout, float(deadline) - time.monotonic())) if deadline else timeout


def event(**values):
    path = os.environ.get(JOURNAL)
    if path:
        record = dict(values, monotonic=time.monotonic(), pid=os.getpid(), phase=os.environ.get(PHASE,'concrete'))
        # One append per event, also safe for native-generation subprocesses.
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        try:
            os.write(fd, (json.dumps(record) + "\n").encode())
        finally:
            os.close(fd)


@contextlib.contextmanager
def interval(category, **metadata):
    identifier = uuid.uuid4().hex
    event(kind="start", id=identifier, category=category, **metadata)
    try:
        yield
    finally:
        event(kind="end", id=identifier)


def totals(path, stopped_at=None, phase=None):
    """Include elapsed portions of intervals interrupted by a killed worker."""
    starts, ends, result = {}, {}, {}
    if not Path(path).exists():
        return result
    for line in Path(path).read_text().splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue  # A final append may have been interrupted.
        if record.get("kind") == "start":
            starts[record["id"]] = record
        elif record.get("kind") == "end":
            ends[record["id"]] = record["monotonic"]
    for identifier, record in starts.items():
        if phase is not None and record.get('phase','concrete') != phase:
            continue
        end = ends.get(identifier, stopped_at)
        if end is not None:
            category = record["category"]
            result[category] = result.get(category, 0.) + max(0., end-record["monotonic"])
    return result


def interactive_check(model, command, timeout, *, setup_command="go_msat"):
    """Time actual interactive solver commands, not the whole nuXmv process."""
    started = time.monotonic()
    deadline = started + remaining(timeout)
    model = Path(model).resolve()
    process = subprocess.Popen(["nuXmv", "-int", "-quiet", "-pre", "cpp", model.name],
                               cwd=model.parent, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               start_new_session=True)
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    durations = {}
    status = "finished"
    with (model.parent / "checking.log").open("wb") as log:
        def prompt():
            tail = b""
            while True:
                wait = deadline - time.monotonic()
                if wait <= 0:
                    raise TimeoutError("whole-run deadline reached")
                if not selector.select(min(wait, 1.)):
                    if process.poll() is not None:
                        raise RuntimeError("nuXmv exited before its next prompt")
                    continue
                chunk = os.read(process.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError("nuXmv closed its output before its next prompt")
                log.write(chunk)
                log.flush()
                tail = (tail + chunk)[-32:]
                if tail.endswith(b"nuXmv > "):
                    return

        def phase(category, commands, initial=False):
            begin = time.monotonic()
            try:
                with interval(category, model=str(model)):
                    if initial:
                        prompt()
                    for instruction in commands:
                        process.stdin.write((instruction + "\n").encode())
                        process.stdin.flush()
                        prompt()
            finally:
                durations[category + "_seconds"] = time.monotonic() - begin

        try:
            phase("setup", [setup_command], initial=True)
            phase("model_checking", [command])
            phase("trace_export", ["show_traces -p 6 -o trace.xml 1",
                                    "show_traces -o debug_trace_array.txt 1"])
            process.stdin.write(b"quit\n")
            process.stdin.flush()
            process.wait(timeout=max(.01, deadline-time.monotonic()))
            if process.returncode:
                status = "error"
        except (TimeoutError, subprocess.TimeoutExpired):
            status = "timeout"
        except (RuntimeError, BrokenPipeError) as exc:
            log.write(("\nDriver error: " + str(exc)).encode())
            status = "error"
        finally:
            selector.close()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            process.stdin.close()
            process.stdout.close()
    return dict(status=status, seconds=time.monotonic()-started, **durations)
