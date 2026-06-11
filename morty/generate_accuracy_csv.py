#!/usr/bin/env python3
"""
Generate CSV tables comparing prediction accuracy across experiments.

Usage:
    python morty/generate_accuracy_csv.py
    python morty/generate_accuracy_csv.py -o results.csv

Reads raw MC output (debug_trace_array.txt) from the archived experiment
folders and produces a CSV file with position and velocity errors for all
cars, steps, and experiments.

Output: morty/accuracy_results.csv  (one row per car x step x experiment)
"""

import argparse
import csv
import os
import re
import sys

import numpy as np

EXPERIMENTS = {
    "A: Baseline (1/1)": {
        "root": "examples/exp4_time_1000_dist_1000___the_94_percent_setup/detailed_archive/run_0",
        "time_scale": 1.0,
    },
    "B: Scaled, no lat (2/4, MT=3)": {
        "root": "examples/exp4_time_2000_dist_4000_mintimeBetweenLC_3",
        "time_scale": 2.0,
    },
    "C: Scaled, with lat (2/4, MT=1)": {
        "root": "examples/exp4_time_2000_dist_4000_mintimeBetweenLC_1/detailed_archive/run_0",
        "time_scale": 2.0,
    },
}

CARS = ["609", "619", "629", "639", "649"]
STEPS = [2, 3, 5]
CONFIG_PRIOS = ["_config_vehwidth=8", "_config_vehwidth=7", "_config_vehwidth=6"]


def parse_debug_trace(filepath):
    """Parse a debug_trace_array.txt file.

    Returns a dict mapping state labels (e.g. "1.1", "1.2") to a dict of
    {variable_name: float_value}.  Only variables whose values look numeric
    are included.
    """
    states = {}
    current_state = None

    state_re = re.compile(r"^\s*-> State:\s*(\S+)\s*<-")
    assign_re = re.compile(r"^\s+(\S+)\s*=\s*(\S+)")

    with open(filepath, "r") as f:
        for line in f:
            m = state_re.match(line)
            if m:
                current_state = m.group(1)
                if current_state not in states:
                    states[current_state] = {}
                continue

            if current_state is not None:
                m = assign_re.match(line)
                if m:
                    var, val = m.group(1), m.group(2)
                    try:
                        states[current_state][var] = float(val)
                    except ValueError:
                        pass

    # Forward-fill: each state inherits values from the previous state
    ordered = sorted(states.keys(), key=lambda s: list(map(int, s.split("."))))
    prev = {}
    for s in ordered:
        merged = {**prev, **states[s]}
        states[s] = merged
        prev = merged

    return states


def compute_diffs(root, step, time_scale, variables):
    """Compute prediction-vs-actual diffs for the given step and variables."""
    target_offset = round((step - 1) * 2 / time_scale)

    all_iterations = set()
    for entry in os.listdir(root):
        m = re.match(r"^iteration_(\d+)$", entry)
        if m:
            all_iterations.add(int(m.group(1)))

    base_iterations = sorted(
        m for m in all_iterations if (m + target_offset) in all_iterations
    )

    diffs = {var: [] for var in variables}

    for m in base_iterations:
        target_iter = m + target_offset
        step_key = f"1.{step}"

        # Find trace for base iteration (predicted state at step_key)
        base_values = None
        for cfg in CONFIG_PRIOS:
            path = os.path.join(root, f"iteration_{m}", cfg, "debug_trace_array.txt")
            if not os.path.isfile(path):
                continue
            trace = parse_debug_trace(path)
            if trace and step_key in trace:
                base_values = trace[step_key]
                break

        if base_values is None:
            continue

        # Find trace for target iteration (actual state at 1.1)
        target_values = None
        for cfg in CONFIG_PRIOS:
            path = os.path.join(
                root, f"iteration_{target_iter}", cfg, "debug_trace_array.txt"
            )
            if not os.path.isfile(path):
                continue
            trace = parse_debug_trace(path)
            if trace and "1.1" in trace:
                target_values = trace["1.1"]
                break

        if target_values is None:
            continue

        for var in variables:
            if var in base_values and var in target_values:
                diffs[var].append(base_values[var] - target_values[var])
            else:
                diffs[var].append(float("nan"))

    return diffs


def main():
    parser = argparse.ArgumentParser(description="Generate accuracy comparison CSV")
    parser.add_argument(
        "--output", "-o", default="morty/accuracy_results.csv",
        help="Output CSV path (default: morty/accuracy_results.csv)",
    )
    args = parser.parse_args()

    rows = []

    for exp_name, exp_cfg in EXPERIMENTS.items():
        root = exp_cfg["root"]
        ts = exp_cfg["time_scale"]

        if not os.path.isdir(root):
            print(f"Warning: {root} not found, skipping {exp_name}")
            continue

        # Count iterations
        iterations = [d for d in os.listdir(root) if d.startswith("iteration_")]
        n_iters = len(iterations)

        for step in STEPS:
            variables = []
            for car in CARS:
                variables.append(f"env.veh___{car}___.abs_pos")
                variables.append(f"env.veh___{car}___.v")

            diffs = compute_diffs(root, step, ts, variables)

            for car in CARS:
                pos_var = f"env.veh___{car}___.abs_pos"
                vel_var = f"env.veh___{car}___.v"

                pos_vals = [v for v in diffs[pos_var] if v == v]
                vel_vals = [v for v in diffs[vel_var] if v == v]

                pos_arr = np.array(pos_vals) if pos_vals else np.array([])
                vel_arr = np.array(vel_vals) if vel_vals else np.array([])

                rows.append({
                    "experiment": exp_name,
                    "iterations": n_iters,
                    "step": step,
                    "car": car,
                    "pos_mean": f"{pos_arr.mean():.2f}" if len(pos_arr) else "N/A",
                    "pos_std": f"{pos_arr.std():.2f}" if len(pos_arr) else "N/A",
                    "pos_min": f"{pos_arr.min():.2f}" if len(pos_arr) else "N/A",
                    "pos_max": f"{pos_arr.max():.2f}" if len(pos_arr) else "N/A",
                    "vel_mean": f"{vel_arr.mean():.2f}" if len(vel_arr) else "N/A",
                    "vel_std": f"{vel_arr.std():.2f}" if len(vel_arr) else "N/A",
                    "vel_min": f"{vel_arr.min():.2f}" if len(vel_arr) else "N/A",
                    "vel_max": f"{vel_arr.max():.2f}" if len(vel_arr) else "N/A",
                    "n_samples": len(pos_vals),
                })

    # Write CSV
    fieldnames = [
        "experiment", "iterations", "step", "car",
        "pos_mean", "pos_std", "pos_min", "pos_max",
        "vel_mean", "vel_std", "vel_min", "vel_max",
        "n_samples",
    ]

    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"CSV written to {args.output} ({len(rows)} rows)")

    # Also print a readable summary to stdout
    print(f"\n{'Experiment':<38} {'Step':>4} {'Car':>4} "
          f"{'Pos Mean±Std':>16} {'Vel Mean±Std':>16} {'N':>4}")
    print("-" * 90)
    for r in rows:
        print(f"{r['experiment']:<38} {r['step']:>4} {r['car']:>4} "
              f"{r['pos_mean']:>7} ± {r['pos_std']:<6} "
              f"{r['vel_mean']:>7} ± {r['vel_std']:<6} {r['n_samples']:>4}")


if __name__ == "__main__":
    main()
