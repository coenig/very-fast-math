"""Search a catalog of symbolic scenarios, checkpointing after each stage."""
import argparse
import json
from pathlib import Path
from .abstract import run as abstract_run
from .concretize import run as concrete_run
from .common import dump


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--only", nargs="+")
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--concrete-bound", type=int, default=300)
    parser.add_argument("--timeout", type=float, default=1800)
    parser.add_argument("--concretize", action="store_true", help="refine successful abstract witnesses after all abstract searches")
    parser.add_argument('--refinement',choices=('monitor','sequential-local'),default='monitor')
    args = parser.parse_args()
    catalog = json.loads(args.catalog.read_text())
    if args.only and set(args.only) - catalog.keys():
        parser.error("unknown scenario name in --only")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "summary.json").exists():
        parser.error("use a new output directory")
    results = {}
    for name, config in catalog.items():
        if args.only and name not in args.only:
            continue
        if not name.replace("_", "").isalnum():
            raise ValueError("scenario names must be alphanumeric with underscores")
        print(f"Searching {name}: {config['rows']}x{config['columns']}, {config['cars']} cars, {config['parkings']} parking nodes", flush=True)
        results[name] = {"abstract": {"status": "running"}}
        dump(output / "summary.json", results)
        try:
            result = abstract_run(config, output / name / "abstract", args.bound, args.timeout)
            results[name] = {"abstract": result}
            if result["status"] == "witness":
                graph = json.loads((output / name / "abstract" / "selected_graph.json").read_text())
                results[name].update(states=len(graph["milestones"]), edges=len(graph["edges"]), roles=graph["roles"])
        except Exception as exc:
            results[name] = {"abstract": {"status": "error", "message": str(exc)}}
        dump(output / "summary.json", results)
    if args.concretize:
        from .sequential import run as sequential_run
        refine=sequential_run if args.refinement=='sequential-local' else concrete_run
        for name, record in results.items():
            if record["abstract"]["status"] != "witness":
                continue
            record["concrete"] = {"status": "running"}
            dump(output / "summary.json", results)
            try:
                record["concrete"] = refine(output / name / "abstract", output / name / "concrete",
                                                  args.concrete_bound, args.timeout)
            except Exception as exc:
                record["concrete"] = {"status": "error", "message": str(exc)}
            dump(output / "summary.json", results)
    print(f"Results: {output / 'summary.json'}", flush=True)
    return 0 if all(r.get("concrete", r["abstract"])["status"] == "witness" for r in results.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
