"""Render selected edges and the actual endpoint-aware connection curves."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from .geometry import point
from .common import states


def abstract_plot(graph,output,gif=True):
    fig,ax=plt.subplots(figsize=(8,5))
    positions={n["id"]:(n["column"],-n["row"]) for n in graph["nodes"]}
    for u,v in graph["edges"]:
        ax.plot(*zip(positions[u],positions[v]),color="black",zorder=0)
    for n in graph["nodes"]:
        x,y=positions[n["id"]]
        ax.scatter([x],[y],marker="s" if n["kind"]=="P" else "o",c="lightgreen" if n["kind"]=="P" else "lightgray",s=160)
        roles = [name for name, node in graph.get("roles", {}).items() if node == n["id"]]
        label = f"{n['kind']}{n['id']}" + (" [" + ", ".join(roles) + "]" if roles else "")
        ax.annotate(label,(x,y),xytext=(6,8),textcoords="offset points",fontsize=8)
    ax.set_aspect("equal"); ax.axis("off")
    fig.savefig(Path(output)/"topology.png",dpi=150,bbox_inches="tight")
    if gif:
        dots=[ax.plot([],[],"o",label=f"car {i}")[0] for i in range(len(graph["milestones"][0]))]
        ax.legend()
        def update(frame):
            for i,dot in enumerate(dots):
                x,y=positions[graph["milestones"][frame][i]]
                if graph.get('approaches'):
                    from .junctions import DELTA
                    dy,dx=DELTA[graph['approaches'][frame][i]]
                    x,y=x+0.18*dx,y-0.18*dy
                dot.set_data([x],[y])
            ax.set_title(f"Abstract step {frame}; goals: {graph['goals']}")
            return dots
        FuncAnimation(fig,update,frames=len(graph["milestones"]),interval=500).save(Path(output)/"abstract.gif",writer=PillowWriter(fps=2))
    plt.close(fig)


def car_point(state,car,geometry):
    prefix=f"env.veh___6{car}9___"
    sec=int(state[f"{prefix}.on_straight_section"])
    pos=int(state[f"{prefix}.abs_pos"])
    if sec>=0:
        # A symbolic model retains candidate slot IDs; its exported geometry
        # contains only active sections, so IDs need not be dense.
        s=next(s for s in geometry["sections"] if s['id']==sec)
        return point([s["source"],s["drain"]],pos/s["length"])
    a,b=int(state[f"{prefix}.traversion_from"]),int(state[f"{prefix}.traversion_to"])
    c=next(c for c in geometry["connections"] if (c["from"],c["to"])==(a,b))
    # Interpolate by sampled arclength, not Bezier parameter, for uniform speed.
    samples=[point(c["curve"],i/256) for i in range(257)]
    import math
    lengths=[math.dist(a,b) for a,b in zip(samples,samples[1:])]
    distance=pos/c["length"]*sum(lengths)
    for index,length in enumerate(lengths):
        if distance<=length:
            return point([samples[index],samples[index+1]],distance/length if length else 0)
        distance-=length
    return samples[-1]


def concrete_plot(geometry,output,trace=None,gif=True):
    fig,ax=plt.subplots(figsize=(9,6))
    for s in geometry["sections"]:
        ax.plot(*zip(s["source"],s["drain"]),color="black",linewidth=3)
        midpoint=point([s["source"],s["drain"]],0.5)
        ax.annotate(f"s{s['id']} ({s['length']}m)",midpoint,xytext=(4,4),textcoords="offset points",fontsize=8)
        if s["kind"]=="parking":
            ax.scatter(*s["drain"],marker="s",c="lightgreen",s=120,zorder=3)
    for c in geometry["connections"]:
        ax.plot(*zip(*(point(c["curve"],i/100) for i in range(101))),color="royalblue",linewidth=1)
    ax.set_aspect("equal"); ax.set_xlabel("metres"); ax.set_ylabel("metres")
    ax.set_title("Black: sections; blue: connections; green: parking ends")
    fig.savefig(Path(output)/"topology.png",dpi=150,bbox_inches="tight")
    if gif and trace:
        frames=states(trace)
        cars=len([k for k in frames[0] if k.startswith("env.veh___6") and k.endswith(".on_straight_section")])
        dots=[ax.plot([],[],"o",label=f"car {i}",zorder=5)[0] for i in range(cars)]
        ax.legend()
        def update(index):
            for i,dot in enumerate(dots):
                x,y=car_point(frames[index],i,geometry); dot.set_data([x],[y])
            ax.set_title(f"Concrete tick {index}; milestone {frames[index].get('refinement_phase','?')}")
            return dots
        FuncAnimation(fig,update,frames=len(frames),interval=170).save(Path(output)/"concrete.gif",writer=PillowWriter(fps=6))
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path,help="abstract or concrete output directory")
    parser.add_argument("--trace",type=Path)
    args=parser.parse_args()
    geometry=args.run/"geometry.json"
    if geometry.exists():
        trace=args.trace
        if trace is None:
            trace=next((args.run/name for name in ('trace.xml','partial_trace.xml') if (args.run/name).exists()),None)
        if trace is None and (args.run/"model.json").exists():
            trace=Path(json.loads((args.run/"model.json").read_text())["model"]).parent/"trace.xml"
            if not trace.exists(): trace=None
        concrete_plot(json.loads(geometry.read_text()),args.run,trace)
    else:
        abstract_plot(json.loads((args.run/"selected_graph.json").read_text()),args.run)


if __name__=="__main__":
    main()
