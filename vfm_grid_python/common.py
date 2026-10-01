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
    started = time.monotonic()
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
                        return {"status": "timeout", "seconds": elapsed}
                    print(f"[{Path(log).name}] still running: {elapsed:.0f}s / {timeout:g}s",flush=True)
        except BaseException:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise
    return {"status": "finished" if returncode == 0 else "error",
            "returncode": returncode, "seconds": time.monotonic() - started}


def check(model, bound=100, timeout=1800, temporal=False):
    if bound < 0 or timeout <= 0:
        raise ValueError("bound must be non-negative and timeout must be positive")
    model = Path(model).resolve()
    xml = model.parent / "trace.xml"
    txt = model.parent / "debug_trace_array.txt"
    # A run directory is a single attempt. Never mistake an old witness for this run.
    if xml.exists() or txt.exists():
        raise FileExistsError(f"trace outputs already exist in {model.parent}; use a new directory")
    command = (f"msat_check_ltlspec_sbmc_inc -k {bound} -n 0" if temporal else
               f"msat_check_invar_bmc -a falsification -k {bound} -n 0")
    script = model.parent / "check.cmd"
    script.write_text(f'go_msat\n{command}\nshow_traces -p 6 -o trace.xml 1\n'
                      'show_traces -o debug_trace_array.txt 1\nquit\n')
    result = execute(["nuXmv", "-quiet", "-pre", "cpp", "-source", str(script),
                      model.name], model.parent, model.parent / "checking.log", timeout)
    log = (model.parent / "checking.log").read_text()
    invalid = re.search(r'"[^"\n]+" undefined|undefined symbol|syntax error|type error|invalid preprocessing directive',log,re.I)
    if invalid:
        result["status"] = "error"
    elif xml.exists() and xml.stat().st_size and states(xml):
        result["status"] = "witness"
    elif result["status"] != "timeout":
        result["status"] = ("no_witness_within_bound" if
                            "no counterexample found with bound" in log.lower() else "error")
    result["bound"] = bound
    dump(model.parent / "check_result.json", result)
    print(f"{model.parent.name}: {result['status']} ({result['seconds']:.2f}s)", flush=True)
    return result


def grid_edges(rows, columns):
    return [(u, v) for u in range(rows * columns)
            for v in (u + 1, u + columns)
            if v < rows * columns and (v == u + columns or u // columns == v // columns)]


def config_args(parser):
    parser.add_argument("--config", type=Path, help="JSON configuration (optional)")
    parser.add_argument("--rows", type=int)
    parser.add_argument("--columns", type=int)
    parser.add_argument("--cars", type=int)
    parser.add_argument("--parkings", type=int, help="exact number of symbolic parking nodes")
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=1800)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--no-gif", action="store_true")


def load_config(args):
    config = json.loads(args.config.read_text()) if args.config else {}
    for key in ("rows", "columns", "cars", "parkings"):
        if getattr(args, key) is not None:
            config[key] = getattr(args, key)
    for key, default in (("rows", 2), ("columns", 3), ("cars", 2)):
        config.setdefault(key, default)
    return config
