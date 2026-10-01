"""Run abstract synthesis followed by VFM monitor refinement."""
import argparse
from . import abstract, concretize
from .common import config_args, load_config, dump


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    config_args(parser)
    concretize.arguments(parser)
    args=parser.parse_args()
    output=args.output_dir.resolve()
    result=abstract.run(load_config(args),output / "abstract",args.bound,args.timeout,not args.no_gif)
    if result["status"] == "witness":
        result=concretize.run(output / "abstract",output / "concrete",args.concrete_bound,
                             args.timeout,args.road_length,args.parking_length,args.junction_radius,
                             args.template_config,not args.no_gif)
    dump(output / "result.json",result)
    return 0 if result["status"] == "witness" else 2


if __name__ == "__main__":
    raise SystemExit(main())
