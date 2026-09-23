#!/usr/bin/env python3
"""Run the abstract-grid and concrete-refinement parking workflow."""
from __future__ import annotations
import argparse, subprocess, sys
from pathlib import Path

from parking_geometry import GeometryConfig, write_geometry


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--rows", type=int, default=4)
    p.add_argument("--columns", type=int, default=5)
    p.add_argument("--cars", type=int, default=2)
    p.add_argument("--bound", type=int, default=100)
    p.add_argument("--output-dir", type=Path, default=Path("vfm-with-parking/pipeline"))
    p.add_argument("--concrete-bound", type=int, default=100)
    p.add_argument("--section-length", type=int, default=100)
    p.add_argument("--lane-offset", type=int, default=10,
                    help="perpendicular offset for opposite directed road sections")
    p.add_argument("--junction-radius", type=int, default=20,
                   help="radius of the connector area around each abstract grid node")
    p.add_argument("--parking-lane-offset", type=int, default=5,
                   help="half-separation of inbound/outbound parking sections")
    p.add_argument("--generate-only", action="store_true")
    p.add_argument("--false-property", action="store_true",
                   help="use INVARSPEC FALSE to produce a one-state smoke-test trace")
    args = p.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    abstract_cmd = [sys.executable, "scalable_grid_parking.py", "--rows", str(args.rows),
                    "--columns", str(args.columns), "--cars", str(args.cars),
                    "--bound", str(args.bound), "--output-dir", str(args.output_dir / "abstract")]
    result = subprocess.run(abstract_cmd)
    if result.returncode:
        return result.returncode
    runs = sorted((args.output_dir / "abstract").glob("scenario*_grid_*"))
    if not runs:
        raise RuntimeError("abstract stage produced no scenario directory")
    abstract = runs[-1]
    grid = abstract / "selected_grid.json"
    if not grid.exists():
        raise RuntimeError(f"abstract stage produced no selected grid: {grid}")
    write_geometry(
        grid, abstract / "concrete_geometry.json",
        GeometryConfig(
            grid_pitch=args.section_length + 2 * args.junction_radius,
            road_length=args.section_length,
            junction_radius=args.junction_radius,
            lane_offset=args.lane_offset,
            parking_lane_offset=args.parking_lane_offset,
        ),
    )
    concrete_cmd = [sys.executable, "concretizer.py", str(abstract), "--bound",
                    str(args.concrete_bound), "--section-length", str(args.section_length),
                    "--lane-offset", str(args.lane_offset),
                    "--junction-radius", str(args.junction_radius),
                    "--parking-lane-offset", str(args.parking_lane_offset),
                    "--output-dir", str(args.output_dir / "concrete")]
    if args.generate_only:
        concrete_cmd.append("--generate-only")
    if args.false_property:
        concrete_cmd.append("--false-property")
    return subprocess.run(concrete_cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main())
