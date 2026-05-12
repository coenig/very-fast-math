#!/usr/bin/env python3
"""Plot prediction accuracy across iterations for a fixed prediction step.

For each iteration m (x-axis), computes the difference between the predicted
state at step i in iteration m's trace and the actual initial state (1.1) in
iteration m + (i-1)*2. Step 1.1 is the current state; step i predicts (i-1)
seconds ahead, and at 2 Hz policy frequency that maps to (i-1)*2 iterations.

Usage:
    python plot_prediction_accuracy.py --root examples/gp/detailed_archive/run_0 \
        --step 5 --config "_config_vehwidth=8;_config_vehwidth=7;_config_vehwidth=6"

The --config argument accepts a semicolon-separated priority list. For each
iteration, the script picks the highest-priority config that has a valid
counterexample (matching morty's UCD_CONFIG_PRIOS behavior).

Example with explicit variables:
    python plot_prediction_accuracy.py --root examples/gp/detailed_archive/run_0 \
        --step 5 --config "_config_vehwidth=8;_config_vehwidth=7;_config_vehwidth=6" \
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
            "and state 1.1 in iteration m + (i-1)*2."
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
        help=(
            "Semicolon-separated priority list of config folder names "
            "(e.g., '_config_vehwidth=8;_config_vehwidth=7;_config_vehwidth=6'). "
            "For each iteration the first config with a valid counterexample is used."
        ),
    )
    parser.add_argument(
        "--variables", default=DEFAULT_VARIABLES,
        help="Comma-separated list of variables (default: %(default)s)",
    )
    parser.add_argument(
        "--save", default=None,
        help="Save plot to file instead of showing (e.g., output.png)",
    )
    args = parser.parse_args()

    variables = [v.strip() for v in args.variables.split(",")]
    step = args.step
    target_offset = (step - 1) * 2
    config_prios = [c.strip() for c in args.config.split(";") if c.strip()]

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

    def find_trace_with_step(iteration, required_step):
        """Return parsed states from the highest-priority config that has
        a valid counterexample containing the required step."""
        for cfg in config_prios:
            path = os.path.join(
                args.root, f"iteration_{iteration}", cfg, "debug_trace_array.txt"
            )
            if not os.path.isfile(path):
                continue
            states = parse_debug_trace(path)
            if states and required_step in states:
                return states, cfg
        return None, None

    def find_trace_with_state1(iteration):
        """Return parsed states from any config that has state 1.1."""
        for cfg in config_prios:
            path = os.path.join(
                args.root, f"iteration_{iteration}", cfg, "debug_trace_array.txt"
            )
            if not os.path.isfile(path):
                continue
            states = parse_debug_trace(path)
            if states and 1 in states:
                return states, cfg
        return None, None

    # --- Collect differences ---
    diffs = {var: [] for var in variables}
    x_values = []
    configs_used = []

    for m in base_iterations:
        target_iter = m + target_offset

        base_states, base_cfg = find_trace_with_step(m, step)
        if base_states is None:
            print(
                f"Warning: No valid prediction at step 1.{step} in iteration {m} "
                f"(tried configs: {config_prios}), line will be broken.",
                file=sys.stderr,
            )
            x_values.append(m)
            configs_used.append(None)
            for var in variables:
                diffs[var].append(float("nan"))
            continue

        target_states, target_cfg = find_trace_with_state1(target_iter)
        if target_states is None:
            print(
                f"Warning: No state 1.1 in target iteration {target_iter} "
                f"(tried configs: {config_prios}), line will be broken.",
                file=sys.stderr,
            )
            x_values.append(m)
            configs_used.append(None)
            for var in variables:
                diffs[var].append(float("nan"))
            continue

        base_values = base_states[step]
        target_values = target_states[1]

        x_values.append(m)
        configs_used.append(base_cfg)
        for var in variables:
            if var in base_values and var in target_values:
                diffs[var].append(base_values[var] - target_values[var])
            else:
                diffs[var].append(float("nan"))

    # --- Print numeric summary ---
    import numpy as np

    print(f"\n{'Variable':<45} {'Mean':>8} {'Std':>8} {'Min':>8} {'Max':>8}")
    print("-" * 81)
    for var in variables:
        vals = [v for v in diffs[var] if v == v]  # filter NaN
        if vals:
            arr = np.array(vals)
            print(
                f"{var:<45} {arr.mean():>8.2f} {arr.std():>8.2f} "
                f"{arr.min():>8.2f} {arr.max():>8.2f}"
            )
        else:
            print(f"{var:<45} {'N/A':>8} {'N/A':>8} {'N/A':>8} {'N/A':>8}")

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(12, 6))

    for var in variables:
        ax.plot(x_values, diffs[var], marker="o", label=var, markersize=4)

    ax.set_xlabel("Base iteration (m)")
    ax.set_ylabel("Difference (predicted − actual)")
    config_label = " > ".join(config_prios) if len(config_prios) > 1 else config_prios[0]
    ax.set_title(
        f"Prediction accuracy: step {step} vs iteration m+{target_offset}, "
        f"config priority = {config_label}"
    )
    ax.legend(loc="best", fontsize="small")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="black", linewidth=0.5)

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f"\nPlot saved to: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
