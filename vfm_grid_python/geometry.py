"""One straight section per edge; one reversible connector per incident pair."""
from __future__ import annotations

from itertools import combinations
import math


def point(curve, t):
    if len(curve) == 2:
        return [(1-t)*curve[0][i] + t*curve[1][i] for i in range(2)]
    return [sum(w * p[i] for w, p in zip(((1-t)**3, 3*(1-t)**2*t,
                                        3*(1-t)*t*t, t**3), curve)) for i in range(2)]


def arc_length(curve, samples=256):
    return sum(math.dist(point(curve, i/samples), point(curve, (i+1)/samples))
               for i in range(samples))


def build(graph, road_length=20, parking_length=7, radius=6):
    if any(type(v) is not int or v <= 0 for v in (road_length, parking_length, radius)):
        raise ValueError("road length, parking length and junction radius must be positive integer metres")
    nodes = {n["id"]: dict(n) for n in graph["nodes"]}
    edges = sorted({tuple(sorted(e)) for e in graph["edges"]})
    incident = {n: [] for n in nodes}
    for u, v in edges:
        if u == v or u not in nodes or v not in nodes:
            raise ValueError("invalid edge")
        if abs(nodes[u]["row"]-nodes[v]["row"]) + abs(nodes[u]["column"]-nodes[v]["column"]) != 1:
            raise ValueError("non-grid edge")
        incident[u].append(v)
        incident[v].append(u)
    for u, node in nodes.items():
        if node["kind"] not in ("N", "P"):
            raise ValueError("unknown node kind")
        if node["kind"] == "P" and (len(incident[u]) != 1 or nodes[incident[u][0]]["kind"] != "N"):
            raise ValueError("a P must have exactly one N neighbour")
    pitch = road_length + 2*radius
    centers = {u: [n["column"]*pitch, -n["row"]*pitch] for u, n in nodes.items()}
    sections = []
    for u, v in edges:
        if nodes[u]["kind"] == "P":
            u, v = v, u
        dx = nodes[v]["column"]-nodes[u]["column"]
        dy = nodes[u]["row"]-nodes[v]["row"]
        parking = nodes[v]["kind"] == "P"
        length = parking_length if parking else road_length
        source = [centers[u][0] + radius*dx, centers[u][1] + radius*dy]
        drain = [source[0]+length*dx, source[1]+length*dy]
        if parking:
            centers[v] = drain
        sections.append({"id": len(sections), "edge": [u,v], "kind": "parking" if parking else "road",
                         "source": source, "drain": drain, "source_x": source[0], "source_y": source[1],
                         "length": length, "angle": int(round(math.degrees(math.atan2(dy, dx)))) % 360})
    section_incidence = {n: [] for n in nodes}
    for s in sections:
        for node in s["edge"]:
            section_incidence[node].append(s)
    connections = []
    for node in nodes:
        neighbours = section_incidence[node]
        for a, b in combinations(neighbours, 2):
            ae, be = a["edge"][1] == node, b["edge"][1] == node
            p0, p3 = a["drain"] if ae else a["source"], b["drain"] if be else b["source"]
            da = [(a["drain"][i]-a["source"][i])/a["length"] * (1 if ae else -1) for i in range(2)]
            db = [(b["drain"][i]-b["source"][i])/b["length"] * (-1 if be else 1) for i in range(2)]
            if da == db:
                curve = [p0, p3]
            else:
                tangent = math.dist(p0,p3)/3
                curve = [p0, [p0[i]+tangent*da[i] for i in range(2)],
                         [p3[i]-tangent*db[i] for i in range(2)], p3]
            length = arc_length(curve)
            connections.append({"id": len(connections), "from": a["id"], "to": b["id"],
                                "from_end": ae, "to_end": be, "node": node,
                                "curve": curve, "arc_length_m": length,
                                "length": max(1, math.ceil(length))})
    if not sections:
        raise ValueError("the selected graph has no roads")
    # VFM's root section contract is source=(0,0), angle=0. A rigid transform
    # preserves every length, turn and grid adjacency, including parking leaves.
    origin = sections[0]["source"][:]
    rotation = sections[0]["angle"]
    co, si = round(math.cos(math.radians(rotation))), round(math.sin(math.radians(rotation)))
    def transform(p):
        x,y=p[0]-origin[0],p[1]-origin[1]
        return [co*x+si*y,-si*x+co*y]
    centers={u:transform(p) for u,p in centers.items()}
    for s in sections:
        s["source"],s["drain"]=transform(s["source"]),transform(s["drain"])
        s["source_x"],s["source_y"]=s["source"]
        s["angle"]=(s["angle"]-rotation)%360
    for c in connections:
        c["curve"]=[transform(p) for p in c["curve"]]
    return {"schema": "vfm_grid_python.endpoint.v1", "road_length": road_length,
            "parking_length": parking_length, "junction_radius": radius,
            "nodes": list(nodes.values()), "centers": centers,
            "sections": sections, "connections": connections}


def location_expression(geometry, car, node, stopped=False):
    from .common import either
    prefix = f"env.veh___6{car}9___"
    parts = []
    kind = next(n["kind"] for n in geometry["nodes"] if n["id"] == node)
    for s in geometry["sections"]:
        if node in s["edge"]:
            position = s["length"] if node == s["edge"][1] else 0
            parts.append(f"{prefix}.on_straight_section = {s['id']} & {prefix}.abs_pos = {position}")
    for c in geometry["connections"]:
        if c["node"] == node:
            parts.append(f"{prefix}.traversion_from = {c['from']} & {prefix}.traversion_to = {c['to']}")
    result = either(parts)
    if stopped or kind == "P":
        result += f" & {prefix}.v = 0"
    return f"({result})"
