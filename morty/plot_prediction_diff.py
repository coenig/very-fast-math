#!/usr/bin/env python3
"""Plot differences of variables between a base prediction step and actual
initial states across iterations from debug_trace_array.txt files.

Usage:
    python plot_prediction_diff.py --root examples/gp/detailed_archive/run_0 \
        --base-iteration 0 --base-step 5 --config _config_vehwidth=6

Example with explicit variables:
    python plot_prediction_diff.py --root examples/gp/detailed_archive/run_0 \
        --base-iteration 0 --base-step 5 --config _config_vehwidth=6 \
        --variables "env.veh___609___.abs_pos,env.veh___609___.v"
"""

import argparse
import os
import re
import sys

import matplotlib.pyplot as plt


def parse_debug_trace(filepath):
    """Parse a debug_trace_array.txt file.

    Returns a dict {step_number: {var_name: numeric_value}} where each step
    contains the full cumulative state (unchanged variables carried forward).
    Returns None if no counterexample (no states) was found.
    """
    with open(filepath, "r") as f:
        content = f.read()

    state_pattern = re.compile(r"-> State: 1\.(\d+) <-")
    parts = state_pattern.split(content)

    # parts layout: [header, step1_num, step1_body, step2_num, step2_body, ...]
    if len(parts) < 3:
        return None  # No counterexample found

    var_pattern = re.compile(r"^\s+(\S+)\s*=\s*(.+)$", re.MULTILINE)
    current_state = {}
    states = {}

    for i in range(1, len(parts), 2):
        step_num = int(parts[i])
        block = parts[i + 1]

        for match in var_pattern.finditer(block):
            var_name = match.group(1)
            value_str = match.group(2).strip()

            if value_str in ("TRUE", "FALSE"):
                continue

            try:
                value = int(value_str)
            except ValueError:
                try:
                    value = float(value_str)
                except ValueError:
                    continue

            current_state[var_name] = value

        states[step_num] = dict(current_state)

    return states


DEFAULT_VARIABLES = (
    "env.veh___609___.abs_pos,"
    "env.veh___609___.v,"
    "env.veh___609___.on_normalized_lane,"
    "env.veh___619___.abs_pos,"
    "env.veh___619___.v,"
    "env.veh___619___.on_normalized_lane"
)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot variable differences between a base prediction step "
            "and actual initial states across iterations."
        )
    )
    parser.add_argument(
        "--root", required=True,
        help="Root run folder (e.g., examples/gp/detailed_archive/run_0)",
    )
    parser.add_argument(
        "--base-iteration", type=int, required=True,
        help="Base iteration number (m)",
    )
    parser.add_argument(
        "--base-step", type=int, required=True,
        help="Step number (1-based) in the base iteration trace (n)",
    )
    parser.add_argument(
        "--config", required=True,
        help="Config folder name (e.g., _config_vehwidth=6)",
    )
    parser.add_argument(
        "--variables", default=DEFAULT_VARIABLES,
        help="Comma-separated list of variables (default: %(default)s)",
    )
    args = parser.parse_args()

    variables = [v.strip() for v in args.variables.split(",")]

    # --- Load base ---
    base_dir = os.path.join(args.root, f"iteration_{args.base_iteration}", args.config)
    base_file = os.path.join(base_dir, "debug_trace_array.txt")

    if not os.path.isdir(base_dir):
        print(f"Error: Base config folder not found: {base_dir}", file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(base_file):
        print(f"Error: Base trace file not found: {base_file}", file=sys.stderr)
        sys.exit(1)

    base_states = parse_debug_trace(base_file)
    if base_states is None:
        print(
            f"Error: No counterexample found in base iteration {args.base_iteration}",
            file=sys.stderr,
        )
        sys.exit(1)

    if args.base_step not in base_states:
        print(
            f"Error: Step 1.{args.base_step} not found in base iteration {args.base_iteration} "
            f"(available steps: {sorted(base_states.keys())})",
            file=sys.stderr,
        )
        sys.exit(1)

    base_values = base_states[args.base_step]

    for var in variables:
        if var not in base_values:
            print(
                f"Error: Variable '{var}' not found in base state 1.{args.base_step} "
                f"of iteration {args.base_iteration}",
                file=sys.stderr,
            )
            print(
                f"Available numeric variables: {sorted(base_values.keys())}",
                file=sys.stderr,
            )
            sys.exit(1)

    # --- Discover target iterations ---
    iteration_re = re.compile(r"^iteration_(\d+)$")
    iterations = []
    for entry in os.listdir(args.root):
        match = iteration_re.match(entry)
        if match:
            iter_num = int(match.group(1))
            if iter_num >= args.base_iteration:
                iterations.append(iter_num)
    iterations.sort()

    if not iterations:
        print(f"Error: No iterations found from iteration {args.base_iteration} onward", file=sys.stderr)
        sys.exit(1)

    # --- Collect differences ---
    diffs = {var: [] for var in variables}
    x_values = []

    for iter_num in iterations:
        target_dir = os.path.join(
            args.root, f"iteration_{iter_num}", args.config
        )
        target_file = os.path.join(target_dir, "debug_trace_array.txt")

        if not os.path.isdir(target_dir):
            print(
                f"Error: Config folder not found for iteration {iter_num}: {target_dir}",
                file=sys.stderr,
            )
            sys.exit(1)
        if not os.path.isfile(target_file):
            print(
                f"Error: Trace file not found for iteration {iter_num}: {target_file}",
                file=sys.stderr,
            )
            sys.exit(1)

        target_states = parse_debug_trace(target_file)
        x_values.append(iter_num)

        if target_states is None or 1 not in target_states:
            # No counterexample — insert NaN to break the line
            print(
                f"Warning: No counterexample in iteration {iter_num}, "
                "line will be broken at this point.",
                file=sys.stderr,
            )
            for var in variables:
                diffs[var].append(float("nan"))
            continue

        target_values = target_states[1]

        for var in variables:
            if var in target_values:
                diffs[var].append(base_values[var] - target_values[var])
            else:
                diffs[var].append(float("nan"))

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(12, 6))

    for var in variables:
        ax.plot(x_values, diffs[var], marker="o", label=var, markersize=4)

    ax.set_xlabel("Iteration")
    ax.set_ylabel("Difference (base − target)")
    ax.set_title(
        f"Variable differences: base = iteration {args.base_iteration} step 1.{args.base_step}, "
        f"config = {args.config}"
    )
    ax.legend(loc="best", fontsize="small")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="black", linewidth=0.5)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
