import matplotlib
matplotlib.use('Agg')  # Non-interactive backend — no X11/display required

from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import os


def plot_cex_lengths(cex_lengths, output_path, cnt_history=None, point_colors=None):
    """Plot actual CEX lengths vs. the ideal (decrement-by-1) baseline, with MC priority annotation.

    Args:
        cex_lengths: List of CEX lengths recorded at each iteration where a CEX was found.
        output_path: File path for the output PDF (overwritten each call).
        cnt_history: List of MC priorities (cnt) for each CEX found (optional, default None).
        point_colors: List of colors for individual points (optional, default None).
    """
    if not cex_lengths:
        return

    steps = list(range(len(cex_lengths)))
    ideal = [cex_lengths[0] - i for i in steps]

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(steps, cex_lengths, color='tab:blue', linewidth=1, label='Actual CEX length')
    if point_colors is not None and len(point_colors) == len(cex_lengths):
        ax.scatter(steps, cex_lengths, c=point_colors, s=18, zorder=3)
    else:
        ax.scatter(steps, cex_lengths, color='tab:blue', s=18, zorder=3)
    ax.plot(steps, ideal, linestyle='--', color='gray', label='Ideal (first − step)')

    # Annotate MC priorities if provided
    if cnt_history is not None and len(cnt_history) == len(cex_lengths):
        for x, y, cnt in zip(steps, cex_lengths, cnt_history):
            ax.annotate(str(cnt), (x, y), textcoords="offset points", xytext=(0, 8), ha='center', fontsize=8, color='darkred')

    ax.set_xlabel('Sim time (iteration)')
    ax.set_ylabel('CEX length')
    ax.set_title('CEX Length Progression')
    ax.set_ylim(bottom=0)
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with PdfPages(output_path) as pdf:
        pdf.savefig(fig)
    plt.close(fig)


def plot_cex_lengths_cumulative(all_histories, output_path):
    """Plot all per-seed CEX length curves plus their average.

    Args:
        all_histories: Dict mapping seed -> list of CEX lengths.
        output_path: File path for the output PDF (overwritten each call).
    """
    if not all_histories:
        return

    fig, ax = plt.subplots(figsize=(8, 4))

    # Individual seed traces in light color.
    max_len = 0
    for seed, lengths in all_histories.items():
        steps = list(range(len(lengths)))
        ax.plot(steps, lengths, color='steelblue', alpha=0.2, linewidth=0.8)
        max_len = max(max_len, len(lengths))

    # Compute average (ragged arrays — average only over seeds that have data at each step).
    if max_len > 0:
        import numpy as np
        avg = np.full(max_len, np.nan)
        for step in range(max_len):
            vals = [h[step] for h in all_histories.values() if len(h) > step]
            if vals:
                avg[step] = np.mean(vals)
        ax.plot(range(max_len), avg, color='darkblue', linewidth=2, label='Average')

    # Ideal line from average first value.
    first_vals = [h[0] for h in all_histories.values() if h]
    if first_vals:
        import numpy as np
        avg_first = np.mean(first_vals)
        ideal = [avg_first - i for i in range(max_len)]
        ax.plot(range(max_len), ideal, linestyle='--', color='gray', label='Ideal (avg first − step)')

    ax.set_xlabel('Sim time (iteration)')
    ax.set_ylabel('CEX length')
    ax.set_title(f'CEX Length Cumulative ({len(all_histories)} seeds)')
    ax.set_ylim(bottom=0)
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with PdfPages(output_path) as pdf:
        pdf.savefig(fig)
    plt.close(fig)


def plot_mc_runtimes(mc_runtime_histories, output_path, selected_cnt_history=None):
    """Plot per-priority nuXmv runtime evolution across iterations.

    Args:
        mc_runtime_histories: Dict mapping config/priority label -> list of runtime values in seconds.
        output_path: File path for the output PDF (overwritten each call).
        selected_cnt_history: List of selected priority indices per iteration (None for blind/no-CEX).
    """
    if not mc_runtime_histories:
        return

    fig, ax = plt.subplots(figsize=(8, 4))

    have_any = False
    for idx, (label, runtimes) in enumerate(mc_runtime_histories.items()):
        if not runtimes:
            continue
        steps = list(range(len(runtimes)))
        ax.plot(steps, runtimes, marker='o', markersize=2.5, linewidth=1, label=f"prio {idx}: {label}")
        have_any = True

    # Mark the selected priority in each iteration with an 'X'.
    if selected_cnt_history is not None:
        labels = list(mc_runtime_histories.keys())
        for step, selected_idx in enumerate(selected_cnt_history):
            if selected_idx is None:
                continue
            if selected_idx < 0 or selected_idx >= len(labels):
                continue
            label = labels[selected_idx]
            runtimes = mc_runtime_histories.get(label, [])
            if step >= len(runtimes):
                continue
            y = runtimes[step]
            if y != y:  # NaN check
                continue
            ax.scatter([step], [y], marker='x', s=48, linewidths=1.5, color='black', zorder=4)

    if not have_any:
        plt.close(fig)
        return

    ax.set_xlabel('Sim time (iteration)')
    ax.set_ylabel('nuXmv runtime [s]')
    ax.set_title('MC Runtime Progression by Priority')
    ax.set_ylim(bottom=0)
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with PdfPages(output_path) as pdf:
        pdf.savefig(fig)
    plt.close(fig)
