#!/usr/bin/env python3
"""Create timing plots and an HTML table for three-way parking benchmarks."""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path


def load_results(run: Path) -> list[dict]:
    source = run / "results.jsonl"
    return [json.loads(line) for line in source.read_text().splitlines() if line.strip()]


def visualize(run: Path) -> None:
    results = load_results(run)
    plots = run / "plots"
    plots.mkdir(exist_ok=True)
    try:
        from matplotlib import pyplot as plt

        methods = ("enumerated_vfm", "abstract_concretized", "symbolic_vfm")
        colors = {"enumerated_vfm": "#f58518", "abstract_concretized": "#4c78a8",
                  "symbolic_vfm": "#54a24b"}
        figure, axes = plt.subplots(figsize=(max(12, len(results) / 2.5), 6))
        for method in methods:
            subset = [item for item in results if item["method"] == method]
            x = [item["case"] for item in subset]
            y = [float(item.get("total_time_to_concrete_xml_seconds") or 0)
                 for item in subset]
            axes.plot(x, y, marker="o", label=method, color=colors[method])
        axes.set_yscale("symlog", linthresh=1)
        axes.set_ylabel("Seconds until concrete XML (log scale)")
        axes.tick_params(axis="x", rotation=70)
        axes.legend()
        figure.tight_layout()
        figure.savefig(plots / "time_to_concrete_xml.png", dpi=170)
        plt.close(figure)

        abstract = [item for item in results if item["method"] == "abstract_concretized"]
        figure, axes = plt.subplots(figsize=(max(10, len(abstract) * .7), 5))
        x = [item["case"] for item in abstract]
        first = [float(item.get("abstract_time_to_xml_seconds") or 0) for item in abstract]
        second = [float(item.get("concrete_generation_seconds") or 0) +
                  float(item.get("concrete_checking_seconds") or 0) for item in abstract]
        axes.bar(x, first, label="abstract XML")
        axes.bar(x, second, bottom=first, label="VFM generation + XML")
        axes.set_ylabel("Seconds")
        axes.tick_params(axis="x", rotation=70)
        axes.legend()
        figure.tight_layout()
        figure.savefig(plots / "abstraction_breakdown.png", dpi=170)
        plt.close(figure)
    except ImportError:
        pass

    columns = ["case", "tier", "method", "status", "rows", "columns", "cars",
               "bound", "maximum_edges", "total_time_to_concrete_xml_seconds",
               "abstract_time_to_xml_seconds", "concrete_generation_seconds",
               "concrete_checking_seconds", "abstract_visualization_seconds",
               "visualization_seconds", "candidates_checked", "concrete_states"]
    rows = []
    for item in results:
        cells = "".join(f"<td>{html.escape(str(item.get(column, '')))}</td>"
                        for column in columns)
        rows.append(f"<tr>{cells}</tr>")
    document = """<!doctype html><meta charset='utf-8'>
<title>Parking strategy benchmark</title>
<style>body{font-family:sans-serif;margin:2rem}table{border-collapse:collapse;font-size:12px}
th,td{border:1px solid #bbb;padding:.35rem}th{position:sticky;top:0;background:#eee}
img{max-width:100%}</style>
<h1>Parking strategy benchmark</h1>
<img src='plots/time_to_concrete_xml.png'><img src='plots/abstraction_breakdown.png'>
<table><thead><tr>""" + "".join(f"<th>{html.escape(column)}</th>" for column in columns) + \
        "</tr></thead><tbody>" + "".join(rows) + "</tbody></table>"
    (run / "dashboard.html").write_text(document)
    with (run / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows({key: item.get(key, "") for key in columns} for item in results)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    visualize(args.run)
    print(f"Dashboard: {(args.run / 'dashboard.html').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
