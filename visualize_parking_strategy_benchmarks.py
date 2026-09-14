#!/usr/bin/env python3
"""Create timing plots and an HTML table for three-way parking benchmarks."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
from pathlib import Path


def load_results(run: Path) -> list[dict]:
    source = run / "results.jsonl"
    results = [json.loads(line) for line in source.read_text().splitlines() if line.strip()]
    for item in results:
        item.setdefault("source_run", run.name)
    return results


def discover_runs(inputs: list[Path]) -> list[Path]:
    runs: list[Path] = []
    for path in inputs:
        if (path / "results.jsonl").is_file():
            runs.append(path)
        else:
            runs.extend(sorted(child.parent for child in path.glob("run_*/results.jsonl")))
    unique = {run.resolve(): run for run in runs}
    if not unique:
        raise FileNotFoundError("no benchmark results.jsonl found in the supplied paths")
    return list(unique.values())


def visualize(run: Path, input_runs: list[Path] | None = None) -> None:
    source_runs = input_runs or [run]
    results = [item for source in source_runs for item in load_results(source)]
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

    columns = ["source_run", "case", "tier", "method", "status", "rows", "columns", "cars",
               "bound", "maximum_edges", "total_time_to_concrete_xml_seconds",
               "abstract_time_to_xml_seconds", "concrete_generation_seconds",
               "concrete_checking_seconds", "abstract_visualization_seconds",
               "visualization_seconds", "candidates_checked", "concrete_states"]
    rows = []
    for item in results:
        cells = "".join(f"<td>{html.escape(str(item.get(column, '')))}</td>"
                        for column in columns)
        rows.append(f"<tr>{cells}</tr>")

    # Compact scenario-by-approach pivot, using milliseconds until concrete XML.
    pivot: dict[tuple[str, str], dict[str, float | None]] = {}
    for item in results:
        approach = item.get("method", "")
        if approach == "abstract_concretized" and item.get("refinement"):
            approach = f"{approach}/{item['refinement']}"
        scenario = f"{item.get('source_run', '')}/{item.get('case', '')}"
        raw = item.get("total_time_to_concrete_xml_seconds")
        try:
            seconds = float(raw) if raw not in (None, "") else None
        except (TypeError, ValueError):
            seconds = None
        milliseconds = (seconds * 1000
                        if seconds is not None and math.isfinite(seconds) else None)
        values = pivot.setdefault((scenario, item.get("case", "")), {})
        # A catalog can contain repeated attempts for one approach. Preserve a
        # successful witness if another record for the same cell is a timeout.
        previous = values.get(approach)
        if milliseconds is not None or approach not in values:
            values[approach] = (milliseconds if previous is None else
                                min(previous, milliseconds))
    preferred = ("enumerated_vfm", "abstract_concretized/monitor",
                 "abstract_concretized/ltl", "abstract_concretized/both", "symbolic_vfm")
    approaches = [name for name in preferred if any(name in values for values in pivot.values())]
    approaches.extend(sorted({name for values in pivot.values() for name in values
                               if name not in approaches}))
    max_time = max((value for values in pivot.values() for value in values.values()
                    if value is not None), default=0.0)

    def timing_cell(value: float | None) -> str:
        if value is None:
            return "<td class='missing'>—</td>"
        intensity = 0.0 if max_time <= 0 else min(1.0, value / max_time)
        green = round(245 - 155 * intensity)
        blue = round(210 - 150 * intensity)
        return (f"<td style='background:rgb(255,{green},{blue})' "
                f"title='{value:.3f} ms'>{value:.1f}</td>")

    pivot_rows = []
    for (scenario, _case), values in sorted(pivot.items()):
        pivot_rows.append("<tr><th class='scenario'>" + html.escape(scenario) + "</th>" +
                          "".join(timing_cell(values.get(name)) for name in approaches) + "</tr>")
    pivot_table = ("<h2>Runtime until concrete XML (milliseconds)</h2>"
                   "<p class='table-note'>Darker cells indicate longer runtimes.</p>"
                   "<table class='timing-pivot'><thead><tr><th>scenario</th>" +
                   "".join(f"<th>{html.escape(name)}</th>" for name in approaches) +
                   "</tr></thead><tbody>" + "".join(pivot_rows) + "</tbody></table>")
    document = """<!doctype html><meta charset='utf-8'>
<title>Parking strategy benchmark</title>
<style>body{font-family:sans-serif;margin:2rem}table{border-collapse:collapse;font-size:12px}
th,td{border:1px solid #bbb;padding:.35rem}th{position:sticky;top:0;background:#eee}
.timing-pivot{font-variant-numeric:tabular-nums}.timing-pivot th,.timing-pivot td{text-align:right}
.timing-pivot th.scenario{text-align:left;white-space:nowrap}.timing-pivot .missing{color:#888;background:#f7f7f7}
.table-note{color:#555;font-size:13px}
img{max-width:100%}</style>
<h1>Parking strategy benchmark</h1>
<img src='run_average_times.png'>
<img src='plots/time_to_concrete_xml.png'><img src='plots/abstraction_breakdown.png'>""" + pivot_table + \
        "<h2>Detailed results</h2><table><thead><tr>" + "".join(f"<th>{html.escape(column)}</th>" for column in columns) + \
        "</tr></thead><tbody>" + "".join(rows) + "</tbody></table>"
    (run / "dashboard.html").write_text(document)
    with (run / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows({key: item.get(key, "") for key in columns} for item in results)

    with (run / "scenario_runtime_ms.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["scenario", *approaches])
        writer.writeheader()
        for (scenario, _case), values in sorted(pivot.items()):
            writer.writerow({"scenario": scenario, **{
                name: "" if values.get(name) is None else f"{values[name]:.3f}"
                for name in approaches
            }})

    # Compact run-level view: one point per source run and approach, using the
    # mean time of successful concrete XML witnesses in that run.
    grouped: dict[tuple[str, str], list[float]] = {}
    for item in results:
        value = item.get("total_time_to_concrete_xml_seconds")
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            approach = item["method"]
            refinement = item.get("refinement")
            if approach == "abstract_concretized" and refinement:
                approach = f"{approach}/{refinement}"
            grouped.setdefault((item.get("source_run", "run"), approach), []).append(float(value))
    averages = []
    for (source, approach), values in sorted(grouped.items()):
        averages.append({
            "run": source, "approach": approach, "witnesses": len(values),
            "average_seconds": sum(values) / len(values),
            "minimum_seconds": min(values), "maximum_seconds": max(values),
        })
    compact_columns = ["run", "approach", "witnesses", "average_seconds",
                       "minimum_seconds", "maximum_seconds"]
    with (run / "average_times.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=compact_columns)
        writer.writeheader()
        writer.writerows(averages)
    compact_rows = "".join(
        "<tr>" + "".join(f"<td>{html.escape(str(item[column]))}</td>"
                          for column in compact_columns) + "</tr>"
        for item in averages
    )
    compact_table = (
        "<h2>Average time per run and approach</h2><table><thead><tr>" +
        "".join(f"<th>{html.escape(column)}</th>" for column in compact_columns) +
        f"</tr></thead><tbody>{compact_rows}</tbody></table>"
    )
    dashboard = run / "dashboard.html"
    dashboard.write_text(dashboard.read_text().replace("</table>", "</table>" + compact_table))
    try:
        from matplotlib import pyplot as plt

        approaches = sorted({item["approach"] for item in averages})
        colors = {approach: plt.cm.tab10(index % 10)
                  for index, approach in enumerate(approaches)}
        figure, axes = plt.subplots(figsize=(max(8, len(source_runs) * 2), 5))
        run_names = [source.name for source in source_runs]
        x_positions = {name: index for index, name in enumerate(run_names)}
        for approach in approaches:
            subset = [item for item in averages if item["approach"] == approach]
            axes.scatter([x_positions[item["run"]] for item in subset],
                         [item["average_seconds"] for item in subset],
                         s=70, color=colors[approach], label=approach)
        axes.set_xticks(range(len(run_names)), run_names, rotation=45, ha="right")
        axes.set_ylabel("Average seconds to concrete XML")
        axes.set_title("Average benchmark time by run and approach")
        axes.legend()
        figure.tight_layout()
        figure.savefig(run / "run_average_times.png", dpi=170)
        plt.close(figure)
    except ImportError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, nargs="+",
                        help="run directory, all_runs directory, or multiple run directories")
    parser.add_argument("--output-dir", type=Path,
                        help="directory for the combined dashboard")
    args = parser.parse_args()
    source_runs = discover_runs(args.run)
    if args.output_dir:
        output = args.output_dir
    elif len(source_runs) == 1 and (args.run[0] / "results.jsonl").is_file():
        output = source_runs[0]
    else:
        output = args.run[0] / "combined"
    output.mkdir(parents=True, exist_ok=True)
    visualize(output, source_runs)
    print(f"Included runs: {len(source_runs)}")
    print(f"Dashboard: {(output / 'dashboard.html').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
