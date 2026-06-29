from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt

from .model import Problem


def plot_coordinates(
    problem: Problem,
    coordinates: dict[str, tuple[float, float]],
    output_path: str | Path,
    report: dict | None = None,
    tolerance: float | None = None,
    title: str | None = None,
) -> None:
    edge_errors = {tuple(item["edge"]): item for item in (report or {}).get("edge_errors", [])}
    figure, (graph_axis, error_axis) = plt.subplots(
        1, 2, figsize=(12, 5), gridspec_kw={"width_ratios": [3, 2]}
    )

    labels = []
    errors = []
    for edge in problem.edges:
        a, b = coordinates[edge.u], coordinates[edge.v]
        item = edge_errors.get((edge.u, edge.v))
        actual = item["actual"] if item else math.dist(a, b)
        error = abs(actual - edge.length)
        labels.append(f"{edge.u}-{edge.v}")
        errors.append(error)
        graph_axis.plot([a[0], b[0]], [a[1], b[1]], "k-", zorder=1)
        graph_axis.text(
            (a[0] + b[0]) / 2,
            (a[1] + b[1]) / 2,
            f"target {edge.length:.2f}\nactual {actual:.2f}\nerr {error:.3g}",
            fontsize=7,
            ha="center",
            va="center",
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
        )

    for node, point in coordinates.items():
        graph_axis.plot(point[0], point[1], "o", zorder=2)
        graph_axis.annotate(node, point, xytext=(5, 5), textcoords="offset points", fontweight="bold")

    graph_axis.set_title(title or "Graph layout")
    graph_axis.set_aspect("equal", adjustable="datalim")
    graph_axis.axis("off")

    positions = range(len(errors))
    colors = ["tab:green" if tolerance is None or error <= tolerance else "tab:red" for error in errors]
    error_axis.bar(positions, errors, color=colors)
    error_axis.set_xticks(list(positions), labels, rotation=45, ha="right")
    error_axis.set_ylabel("Absolute edge-length error")
    error_axis.set_title(f"Edge errors (max {max(errors, default=0.0):.3g})")
    if tolerance is not None:
        error_axis.axhline(tolerance, color="tab:orange", linestyle="--", label=f"tolerance = {tolerance:g}")
        error_axis.legend()
    error_axis.grid(axis="y", alpha=0.25)

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def plot_solution(problem: Problem, solution_path: str, output_path: str) -> None:
    data = json.loads(Path(solution_path).read_text())
    coordinates = {
        node: (float(point[0]), float(point[1]))
        for node, point in data["coordinates"].items()
    }
    plot_coordinates(problem, coordinates, output_path, report=data.get("verification"))
