#!/usr/bin/env python3
"""Plot prediction accuracy across iterations for a fixed prediction step.

For each iteration m (x-axis), computes the difference between the predicted
state at step i in iteration m's trace and the actual initial state (1.1) in
iteration m + 2*i. This validates prediction accuracy across the entire run.

Usage:
    python plot_prediction_accuracy.py --root examples/gp/detailed_archive/run_0 \
        --step 5 --config _config_vehwidth=6

Example with explicit variables:
    python plot_prediction_accuracy.py --root examples/gp/detailed_archive/run_0 \
        --step 5 --config _config_vehwidth=6 \
        --variables "env.veh___609___.abs_pos,env.veh___609___.v"
"""

import argparse
import os
import re
import sys

import matplotlib.pyplot as plt


DEFAULT_VARIABLES = (
    "env.veh___609___.abs_pos,"
    "env.veh___609___.v,"
    "env.veh___609___.on_normalized_lane,"
    "env.veh___619___.abs_pos,"
    "env.veh___619___.v,"
    "env.veh___619___.on_normalized_lane"
)


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

    if len(parts) < 3:
        return None

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


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot prediction accuracy across iterations for a fixed step. "
            "For each iteration m, computes diff between step i in m's trace "
            "and state 1.1 in iteration m + 2*i."
        )
    )
    parser.add_argument(
        "--root", required=True,
        help="Root run folder (e.g., examples/gp/detailed_archive/run_0)",
    )
    parser.add_argument(
        "--step", type=int, required=True,
        help="Prediction step number i (1-based) to evaluate across iterations",
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
    step = args.step
    target_offset = (step - 1) * 2

    # --- Discover all iterations ---
    iteration_re = re.compile(r"^iteration_(\d+)$")
    all_iterations = set()
    for entry in os.listdir(args.root):
        match = iteration_re.match(entry)
        if match:
            all_iterations.add(int(match.group(1)))

    if not all_iterations:
        print("Error: No iterations found in root folder", file=sys.stderr)
        sys.exit(1)

    # We need both m and m + target_offset to exist
    base_iterations = sorted(
        m for m in all_iterations if (m + target_offset) in all_iterations
    )

    if not base_iterations:
        print(
            f"Error: No iteration pair (m, m+{target_offset}) available",
            file=sys.stderr,
        )
        sys.exit(1)

    # --- Collect differences ---
    diffs = {var: [] for var in variables}
    x_values = []

    for m in base_iterations:
        base_dir = os.path.join(args.root, f"iteration_{m}", args.config)
        base_file = os.path.join(base_dir, "debug_trace_array.txt")
        target_iter = m + target_offset
        target_dir = os.path.join(args.root, f"iteration_{target_iter}", args.config)
        target_file = os.path.join(target_dir, "debug_trace_array.txt")

        if not os.path.isfile(base_file):
            print(f"Error: Trace file not found: {base_file}", file=sys.stderr)
            sys.exit(1)
        if not os.path.isfile(target_file):
            print(f"Error: Trace file not found: {target_file}", file=sys.stderr)
            sys.exit(1)

        base_states = parse_debug_trace(base_file)
        if base_states is None or step not in base_states:
            # No prediction available at this step — break the line
            print(
                f"Warning: No prediction at step 1.{step} in iteration {m}, "
                "line will be broken.",
                file=sys.stderr,
            )
            x_values.append(m)
            for var in variables:
                diffs[var].append(float("nan"))
            continue

        target_states = parse_debug_trace(target_file)
        if target_states is None or 1 not in target_states:
            print(
                f"Warning: No counterexample in target iteration {target_iter}, "
                "line will be broken.",
                file=sys.stderr,
            )
            x_values.append(m)
            for var in variables:
                diffs[var].append(float("nan"))
            continue

        base_values = base_states[step]
        target_values = target_states[1]

        x_values.append(m)
        for var in variables:
            if var in base_values and var in target_values:
                diffs[var].append(base_values[var] - target_values[var])
            else:
                diffs[var].append(float("nan"))

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(12, 6))

    for var in variables:
        ax.plot(x_values, diffs[var], marker="o", label=var, markersize=4)

    ax.set_xlabel("Base iteration (m)")
    ax.set_ylabel("Difference (predicted − actual)")
    ax.set_title(
        f"Prediction accuracy: step {step} vs iteration m+{target_offset}, "
        f"config = {args.config}"
    )
    ax.legend(loc="best", fontsize="small")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="black", linewidth=0.5)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
