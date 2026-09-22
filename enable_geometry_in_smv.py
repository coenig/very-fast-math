#!/usr/bin/env python3
"""Create a geometry-enabled copy of a generated VFM EnvModel.smv.

The generated parking model omits geometry variables for speed.  This tool
adds frozen grid coordinates and the same angle/sine/cosine definitions used
by ``EnvModel_Sections.tpl``.  The source model is never modified.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path

def generate(model: Path, grid: Path, output: Path, pitch: int = 100, granularity: int = 45) -> None:
    text = model.read_text()
    if "-- GEOMETRY ENABLED BY enable_geometry_in_smv.py" in text:
        output.write_text(text); return
    topo = json.loads(grid.read_text())
    nodes = {int(n["id"]):(int(n["row"]), int(n["column"])) for n in topo["nodes"]}
    outgoing = {}
    for e in topo.get("edges",[]): outgoing.setdefault(int(e["source"]), int(e["target"]))
    lines = ["", "-- GEOMETRY ENABLED BY enable_geometry_in_smv.py", "FROZENVAR"]
    for sec,(row,col) in sorted(nodes.items()):
        target = outgoing.get(sec)
        if target in nodes:
            tr,tc = nodes[target]
            angle = round(math.degrees(math.atan2(tr-row, tc-col))/granularity)*granularity % 360
        else: angle = 0
        raw = angle // granularity
        lines += [f"   section_{sec}.source.x : {col*pitch} .. {col*pitch};",
                  f"   section_{sec}.source.y : {-row*pitch} .. {-row*pitch};",
                  f"   section_{sec}.angle_raw : {raw} .. {raw};"]
    lines += ["", "DEFINE"]
    for sec in sorted(nodes):
        lines += [f"   section_{sec}.angle := section_{sec}.angle_raw * {granularity};",
                  f"   sin_of_section_{sec}_angle := {int(round(math.sin(math.radians(0))*100))};",
                  f"   cos_of_section_{sec}_angle := {int(round(math.cos(math.radians(0))*100))};"]
        # Replace constants with exact quantized values while keeping syntax simple.
        target = outgoing.get(sec); angle = 0
        if target in nodes:
            r,c=nodes[sec]; tr,tc=nodes[target]; angle=round(math.degrees(math.atan2(tr-r,tc-c))/granularity)*granularity%360
        lines[-2] = f"   sin_of_section_{sec}_angle := {int(round(math.sin(math.radians(angle))*100))};"
        lines[-1] = f"   cos_of_section_{sec}_angle := {int(round(math.cos(math.radians(angle))*100))};"
        lines += [f"   section_{sec}.drain.x := section_{sec}.source.x + (section_{sec}_end * cos_of_section_{sec}_angle) / 100;",
                  f"   section_{sec}.drain.y := section_{sec}.source.y + (section_{sec}_end * sin_of_section_{sec}_angle) / 100;"]
    marker = "\n".join(lines) + "\n\n"
    pos = text.find("\nDEFINE")
    if pos < 0: raise ValueError("could not find EnvModel DEFINE section")
    text = text[:pos] + marker + text[pos:]
    output.parent.mkdir(parents=True, exist_ok=True); output.write_text(text)

if __name__ == "__main__":
    p=argparse.ArgumentParser(); p.add_argument("model",type=Path); p.add_argument("grid",type=Path); p.add_argument("-o","--output",required=True,type=Path); p.add_argument("--pitch",type=int,default=100)
    a=p.parse_args(); generate(a.model,a.grid,a.output,a.pitch); print(a.output)
