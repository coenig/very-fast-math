#!/usr/bin/env python3
"""Make a VFM text trace exercise every connector in a fixed topology.

VFM's native CEX renderer intentionally drops unused connector variables.
For topology inspection this utility marks every legal ``from -> to`` pair as
traversed in the initial trace state, causing the renderer to retain the full
fixed road graph.  It is a visualization trace, not a physically executable
vehicle trajectory.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def make_topology_trace(trace: Path, geometry: Path, output: Path) -> None:
    text = trace.read_text()
    manifest = json.loads(geometry.read_text())
    pairs = {
        (int(source), int(target))
        for source, targets in manifest["outgoing"].items()
        for target in targets
    }
    if not pairs:
        raise ValueError("geometry manifest has no outgoing connections")

    state_start = text.find("-> State:")
    if state_start < 0:
        raise ValueError("trace has no NuSMV state")
    next_state = text.find("-> State:", state_start + 1)
    state_end = len(text) if next_state < 0 else next_state
    prefix, state, suffix = text[:state_start], text[state_start:state_end], text[state_end:]

    def mark(match: re.Match[str]) -> str:
        source, target = int(match.group(1)), int(match.group(2))
        return match.group(0).replace(" = 0", " = 1") if (source, target) in pairs else match.group(0)

    state = re.sub(
        r"is_traversing_from_sec_(\d+)_to_sec_(\d+) = 0",
        mark,
        state,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(prefix + state + suffix)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("geometry", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    make_topology_trace(args.trace, args.geometry, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
