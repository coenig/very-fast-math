#!/usr/bin/env python3
"""Render a concrete VFM XML counterexample as GIFs.

This converts the XML counterexample to NuSMV's textual trace format and
invokes the native renderer through libvfm.so (without running the VFM binary).
"""
from __future__ import annotations
import argparse
import shutil
import xml.etree.ElementTree as ET
from ctypes import CDLL, c_bool, c_char_p
from pathlib import Path

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("xml", type=Path, help="XML counterexample")
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--library", type=Path, default=Path("lib/libvfm.so"))
    args = p.parse_args()

    root = ET.parse(args.xml).getroot()
    states = list(root.iter("state"))
    if not states:
        p.error("XML contains no <state> elements")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    trace = args.output_dir / "debug_trace_array.txt"
    lines = ["Trace Description: MSAT BMC counterexample", "Trace Type: Counterexample"]
    for i, state in enumerate(states, 1):
        lines.append(f"  -> State: 1.{i} <-")
        for value in state.findall("value"):
            lines.append(f"    {value.attrib['variable']} = {value.text or ''}")
    trace.write_text("\n".join(lines) + "\n")

    lib = CDLL(str(args.library.resolve()))
    lib.processCEX.argtypes = [c_char_p, c_char_p, c_bool, c_bool]
    lib.processCEX.restype = c_bool
    ok = bool(lib.processCEX(str(args.output_dir.resolve()).encode(), b"smv", True, False))
    print(f"trace: {trace}")
    print(f"renderer returned: {ok}")
    for gif in sorted(args.output_dir.glob("**/*.gif")):
        print(f"gif: {gif}")
    return 0 if ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
