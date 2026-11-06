"""Configuration, trace IO and bounded nuXmv execution."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + "\n")


def states(path):
    result, state = [], {}
    for node in ET.parse(path).getroot().findall("node"):
        element = node.find("state")
        if element is None:
            continue
        state = state.copy()
        for value in element.findall("value"):
            state[value.attrib["variable"]] = value.text.strip()
        result.append(state)
    return result


def either(terms):
    return "(" + " | ".join(f"({term})" for term in terms) + ")" if terms else "FALSE"


def both(terms):
    return "(" + " & ".join(f"({term})" for term in terms) + ")" if terms else "TRUE"


def execute(command, cwd, log, timeout):
    """Bound the whole subprocess tree and keep native expansion output off stdout."""
    from .benchmark_timing import remaining, event
    import uuid
    timeout = remaining(timeout)
    if timeout <= 0:
        return {"status": "timeout", "seconds": 0.0}
    started = time.monotonic()
    identifier = uuid.uuid4().hex
    category = "generation" if Path(log).name == "generation.log" else "preprocessing"
    event(kind="start", id=identifier, category=category, log=str(log))
    with Path(log).open("w") as stream:
        process = subprocess.Popen(command, cwd=cwd, stdout=stream,
                                   stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                   start_new_session=True)
        try:
            while True:
                remaining=timeout-(time.monotonic()-started)
                try:
                    returncode = process.wait(timeout=max(0,min(15,remaining)))
                    break
                except subprocess.TimeoutExpired:
                    elapsed=time.monotonic()-started
                    if elapsed>=timeout:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                        event(kind="end", id=identifier)
                        return {"status": "timeout", "seconds": elapsed}
                    print(f"[{Path(log).name}] still running: {elapsed:.0f}s / {timeout:g}s",flush=True)
        except BaseException:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            event(kind="end", id=identifier)
            raise
    event(kind="end", id=identifier)
    return {"status": "finished" if returncode == 0 else "error",
            "returncode": returncode, "seconds": time.monotonic() - started}


def check(model, bound=100, timeout=1800, temporal=False, *, backend="smt"):
    if bound < 0 or timeout <= 0:
        raise ValueError("bound must be non-negative and timeout must be positive")
    if backend not in ("smt", "sat"):
        raise ValueError("backend must be smt or sat")
    if backend == "sat" and temporal:
        raise ValueError("SAT backend is currently supported for finite invariant models only")
    model = Path(model).resolve()
    xml = model.parent / "trace.xml"
    txt = model.parent / "debug_trace_array.txt"
    # A run directory is a single attempt. Never mistake an old witness for this run.
    if xml.exists() or txt.exists():
        raise FileExistsError(f"trace outputs already exist in {model.parent}; use a new directory")
    command = (f"msat_check_ltlspec_sbmc_inc -k {bound} -n 0" if temporal else
               f"msat_check_invar_bmc -a falsification -k {bound} -n 0")
    setup_command = "go_msat"
    if backend == "sat":
        setup_command = "go_bmc"
        command = f"check_invar_bmc_inc -a falsification -k {bound} -n 0"
    script = model.parent / "check.cmd"
    script.write_text(f'{setup_command}\n{command}\nshow_traces -p 6 -o trace.xml 1\n'
                      'show_traces -o debug_trace_array.txt 1\nquit\n')
    from .benchmark_timing import JOURNAL, interactive_check
    if os.environ.get(JOURNAL):
        result = interactive_check(model, command, timeout, setup_command=setup_command)
    else:
        result = execute(["nuXmv", "-quiet", "-pre", "cpp", "-source", str(script),
                          model.name], model.parent, model.parent / "checking.log", timeout)
    log_path = model.parent / "checking.log"
    log = log_path.read_text() if log_path.exists() else ''
    invalid = re.search(r'"[^"\n]+" undefined|undefined symbol|syntax error|type error|invalid preprocessing directive|Division by non word expression',log,re.I)
    if invalid:
        result["status"] = "error"
    elif result["status"] != "timeout" and xml.exists() and xml.stat().st_size and states(xml):
        result["status"] = "witness"
    elif result["status"] != "timeout":
        result["status"] = ("no_witness_within_bound" if
                            re.search(r"no (?:proof or )?counterexample found with bound", log, re.I) else "error")
    result["bound"] = bound
    dump(model.parent / "check_result.json", result)
    print(f"{model.parent.name}: {result['status']} ({result['seconds']:.2f}s)", flush=True)
    return result


def grid_edges(rows, columns):
    return [(u, v) for u in range(rows * columns)
            for v in (u + 1, u + columns)
            if v < rows * columns and (v == u + columns or u // columns == v // columns)]
