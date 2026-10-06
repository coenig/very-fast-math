"""Tables and runtime plots for endpoint benchmark dumps (never legacy dumps)."""
import argparse
import csv
from collections import defaultdict
import html
import json
import math
from pathlib import Path
import statistics

from .benchmark import METHODS, SCHEMA


def records(paths):
    found = {}
    for supplied in paths:
        path = Path(supplied).resolve()
        files = [path] if path.is_file() else list(path.rglob("result.json"))
        # Read individual checkpoints: this also recovers a run that completed
        # immediately before its supervisor could append results.jsonl.
        for source in files:
            if source.suffix == ".jsonl":
                values = [json.loads(line) for line in source.read_text().splitlines() if line.strip()]
            else:
                values = [json.loads(source.read_text())]
            for value in values:
                if value.get("schema") != SCHEMA:
                    continue
                key = str(Path(value["directory"]).resolve())
                if key in found and found[key] != value:
                    raise ValueError(f"conflicting result records for {key}")
                found[key] = value
    if not found:
        raise ValueError("no endpoint-benchmark.v1 results found")
    return list(found.values())


def summarize(data, metric):
    groups = defaultdict(list)
    for record in data:
        identity = (record["instance"]+' ['+record.get('contract_version','legacy')+']',record["family"],record["rows"],record["columns"],
                    record["cars"],record["repetitions"])
        groups[identity,record["method"]].append(record)
    rows = []
    for identity in sorted({key[0] for key in groups}):
        name,family,r,c,cars,repeats=identity
        row=dict(scenario=name,family=family,rows=r,columns=c,cars=cars,repetitions=repeats)
        for method in METHODS:
            trials=groups[identity,method]
            successful=[trial[metric]*1000 for trial in trials
                        if trial.get("validated") and trial["status"]=="witness" and trial.get(metric) is not None]
            row[method+"_ms"]=statistics.mean(successful) if successful else None
            row[method+"_successes"]=len(successful)
            row[method+"_attempts"]=len(trials)
            row[method+"_statuses"]="; ".join(f"{status}: {sum(t['status']==status for t in trials)}"
                                              for status in sorted({t["status"] for t in trials})) or "not run"
        rows.append(row)
    return rows


def write_report(data, output, plot=True):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    raw_fields=["instance","method","semantics","contract_version","rows","columns","cars","repetitions","seed","parallel",
                "status","validated","wall_seconds","time_to_validated_xml_seconds",
                "abstract_witness_seconds","model_checking_seconds","abstract_model_checking_seconds",
                "concrete_model_checking_seconds","generation_seconds","setup_seconds","validation_seconds",
                "preprocessing_seconds","trace_export_seconds","directory"]
    with (output/"runs.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=raw_fields,extrasaction="ignore")
        writer.writeheader(); writer.writerows(data)
    pages=[]
    for metric, title in (("time_to_validated_xml_seconds","Time to validated XML"),
                          ("model_checking_seconds","Total model-checking commands")):
        rows=summarize(data,metric)
        with (output/f"{metric}.csv").open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        values=[row[m+"_ms"] for row in rows for m in METHODS if row[m+"_ms"] is not None]
        maximum=max([math.log1p(v) for v in values] or [1.])
        table=["<h2>"+title+" (ms)</h2><table><tr><th>Scenario / grid / cars / repetitions</th>"]
        table += ["<th>"+html.escape(m)+"</th>" for m in METHODS]
        table.append("</tr>")
        for row in rows:
            table.append(f"<tr><th>{html.escape(row['scenario'])}<br>{row['rows']}×{row['columns']} / {row['cars']} / {row['repetitions']}</th>")
            for method in METHODS:
                value=row[method+"_ms"]
                text="-" if value is None else f"{value:,.1f}"
                count=row[method+"_attempts"]
                if count>1: text+=f" ({row[method+'_successes']}/{count})"
                shade=1 if value is None else 1-.35*math.log1p(value)/maximum
                color=f"rgb({int(255*shade)}, {int(255*shade)}, 255)"
                table.append(f'<td style="background:{color}" title="{html.escape(row[method+"_statuses"])}">{text}</td>')
            table.append("</tr>")
        table.append("</table>")
        pages.append("".join(table))
    body="""<!doctype html><meta charset="utf-8"><title>Endpoint parking benchmarks</title>
<style>body{font:16px sans-serif;margin:2em}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:.6em;text-align:right}th:first-child{text-align:left}</style>
<h1>Endpoint parking benchmarks</h1><p>Numbers are means of successful, validated runs only.
Parentheses show successful/total attempts. '-' means no validated success (hover for status).
Timeouts, errors, bounded failures and not-run approaches are not zero runtimes.
Plots/GIF generation is excluded. Pure checking includes all solver calls, including initial-state checks.
Parallel jobs compete for resources; consult runs.csv before comparing experiments.</p>"""
    (output/"index.html").write_text(body+"\n".join(pages))
    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(max(8,len(data)*.2),5))
        colors=dict(zip(METHODS,("#276fbf","#e08e00","#268545")))
        for method in METHODS:
            points=[(index,row) for index,row in enumerate(data) if row["method"]==method]
            successes=[(index,row) for index,row in points if row.get("validated") and row["status"]=="witness"]
            failures=[(index,row) for index,row in points if not(row.get("validated") and row["status"]=="witness")]
            ax.scatter([p[0] for p in successes],[p[1]["time_to_validated_xml_seconds"] for p in successes],
                       color=colors[method],label=method)
            ax.scatter([p[0] for p in failures],[p[1]["wall_seconds"] for p in failures],
                       color=colors[method],marker="x")
        ax.set(xlabel="Run index (runs.csv order)",ylabel="Seconds (× = failure/timeout elapsed time)")
        ax.set_yscale("symlog",linthresh=1)
        ax.legend(); fig.tight_layout(); fig.savefig(output/"runtimes.png",dpi=150); plt.close(fig)
        families=sorted({row['family'] for row in data})
        for parameter,label in (('cars','Cars'),('cells','Grid cells'),('repetitions','Stage repetitions')):
            fig,axes=plt.subplots(len(families),1,figsize=(8,max(4,3*len(families))),squeeze=False)
            for axis,family in zip(axes[:,0],families):
                for method in METHODS:
                    points=[row for row in data if row['family']==family and row['method']==method
                            and row.get('validated') and row['status']=='witness']
                    x=[row['rows']*row['columns'] if parameter=='cells' else row[parameter] for row in points]
                    axis.scatter(x,[row['model_checking_seconds'] for row in points],color=colors[method],label=method)
                axis.set(title=family,xlabel=label,ylabel='Total MC seconds')
                axis.set_yscale('symlog',linthresh=1)
            axes[0,0].legend(); fig.tight_layout(); fig.savefig(output/f'scaling_{parameter}.png',dpi=120); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs",type=Path,nargs="+")
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--no-plot",action="store_true")
    parser.add_argument("--gifs",action="store_true",help="render successful witnesses after reporting, outside benchmark timings")
    args=parser.parse_args()
    data=records(args.runs)
    write_report(data,args.output_dir,not args.no_plot)
    if args.gifs:
        from .visualize import concrete_plot, abstract_plot
        reports=[]
        for row in data:
            if row['status']!='witness' or not row.get('validated'): continue
            try:
                trace=Path(row['trace']); directory=trace.parent
                concrete_plot(json.loads((directory/'geometry.json').read_text()),directory,trace,True)
                abstract=Path(row['directory'])/'abstract'
                if (abstract/'selected_graph.json').exists():
                    abstract_plot(json.loads((abstract/'selected_graph.json').read_text()),abstract,True)
                reports.append(dict(directory=str(directory),status='rendered'))
            except Exception as exc:
                reports.append(dict(directory=row['directory'],status='error',error=str(exc)))
        (args.output_dir/'render_results.json').write_text(json.dumps(reports,indent=2)+'\n')
    print(args.output_dir/"index.html")


if __name__=="__main__": main()
