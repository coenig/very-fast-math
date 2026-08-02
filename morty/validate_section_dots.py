#!/usr/bin/env python3
"""
Offline validator for the red "future-position" dots on straight sections.

It mirrors, in pure Python, the exact transform used in
include/vfmacro/script.h  ->  extractVehPosFromNusmvFile, i.e.

    lat_pos = y_max_tech - lane * 2*num_actual/num_tech
    y_max_tech = lane_width_he * (num_actual*(1 - 1/(2*num_tech)) - 1/2)
    global  = source
            + R(angle) * (long_pos, lat_pos)
            + (I - R(angle)) * (0, lat_center)
    lat_center = lane_width_he * (num_actual - 1) / 2

and plots the resulting dots on top of a road polygon that is reconstructed
from each section's (source, angle, end) using the *same* transform. If the
transform is correct, every dot must land inside its section's road band, and
low lane numbers (lane 0 = left-of-travel) must hug the highlighted lane-0 edge.

Usage:
    python3 -m morty.validate_section_dots <trace.txt> [--out out.png]
    python3 -m morty.validate_section_dots <trace.txt> --sweep-angle 45,90,135,225

The --sweep-angle option re-renders section 1 at each given angle (reusing the
same per-step lane/long data) so sign/pivot bugs that a single angle can hide
become visible.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

STATE_RE = re.compile(r"->\s*State:\s*[\d.]+\s*<-")
ASSIGN_RE = re.compile(r"^\s*([\w.\"'\[\]$!{}]+)\s*=\s*(.+?)\s*$")
VEH_ABSPOS_RE = re.compile(r"^env\.veh___(6\d+9)___\.abs_pos$")


def parse_trace(path: str) -> list[dict[str, str]]:
    """Parse a nuXmv trace into a list of cumulative state dicts (one per State)."""
    states: list[dict[str, str]] = []
    cur: dict[str, str] | None = None
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            if STATE_RE.search(raw):
                cur = dict(states[-1]) if states else {}
                states.append(cur)
                continue
            if cur is None:
                continue
            m = ASSIGN_RE.match(raw)
            if m:
                cur[m.group(1)] = m.group(2)
    return states


def fnum(state: dict[str, str], key: str, default: float | None = None) -> float | None:
    v = state.get(key)
    if v is None:
        return default
    try:
        return float(v)
    except ValueError:
        return default


@dataclass
class Section:
    idx: int
    source: tuple[float, float]
    angle_deg: float
    end: float  # in long_pos units (already scaled like abs_pos)


@dataclass
class Dot:
    x: float
    y: float
    veh: str
    lane: float
    step: int
    section: int  # >=0 straight section, -1 = connector/traversion


@dataclass
class Frame:
    lane_width_he: float
    num_tech: float
    num_actual: float
    dist_scale: float
    sections: dict[int, Section] = field(default_factory=dict)


def build_frame(states: list[dict[str, str]]) -> Frame:
    last = states[-1]
    lane_width_he = fnum(last, "env.lane_width", 400.0) / 100.0
    num_tech = fnum(last, "env.num_technical_lanes", 4.0)
    num_actual = fnum(last, "env.num_lanes", 4.0)
    dist_scale = fnum(last, "planner.distance_scaling", 1000.0) or 1000.0

    sections: dict[int, Section] = {}
    for key in last:
        m = re.match(r"env\.section_(\d+)\.source\.x$", key)
        if not m:
            continue
        n = int(m.group(1))
        sx = fnum(last, f"env.section_{n}.source.x")
        sy = fnum(last, f"env.section_{n}.source.y")
        ang = fnum(last, f"env.section_{n}.angle")
        end = fnum(last, f"env.section_{n}_end", 200.0)
        if sx is None or sy is None or ang is None:
            continue
        sections[n] = Section(n, (sx, sy), ang, end / dist_scale * 1000.0)
    return Frame(lane_width_he, num_tech, num_actual, dist_scale, sections)


def y_max_tech(fr: Frame) -> float:
    return fr.lane_width_he * (fr.num_actual * (1.0 - 1.0 / (2.0 * fr.num_tech)) - 0.5)


def lat_center(fr: Frame) -> float:
    return fr.lane_width_he * (fr.num_actual - 1.0) / 2.0


def to_global(fr: Frame, long_pos: float, lat_pos: float, sec: Section) -> tuple[float, float]:
    """Exact replica of the script.h transform for a straight section."""
    ang = math.radians(sec.angle_deg)
    ca, sa = math.cos(ang), math.sin(ang)
    lc = lat_center(fr)
    rx = long_pos * ca - lat_pos * sa
    ry = long_pos * sa + lat_pos * ca
    return (sec.source[0] + rx + lc * sa, sec.source[1] + ry + lc * (1.0 - ca))


def lat_pos_for_lane(fr: Frame, lane: float) -> float:
    return y_max_tech(fr) - lane * 2.0 * fr.num_actual / fr.num_tech


# --- Bezier helpers (exact replicas of include/geometry/bezier_functions.h) ---

def _v_add(a, b): return (a[0] + b[0], a[1] + b[1])
def _v_sub(a, b): return (a[0] - b[0], a[1] - b[1])
def _v_mul(a, s): return (a[0] * s, a[1] * s)
def _v_len(a): return math.hypot(a[0], a[1])
def _v_dist(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])


def _v_set_length(a, length):
    l = _v_len(a)
    if l == 0:
        return (0.0, 0.0)
    return (a[0] / l * length, a[1] / l * length)


def _b_prime(t, p0, p1, p2, p3):
    return _v_add(_v_add(
        _v_mul(_v_sub(p1, p0), 3 * (1 - t) ** 2),
        _v_mul(_v_sub(p2, p1), 6 * (1 - t) * t)),
        _v_mul(_v_sub(p3, p2), 3 * t ** 2))


def _cubic_bezier(t, p0, p1, p2, p3):
    return _v_add(_v_add(_v_add(
        _v_mul(p0, (1 - t) ** 3),
        _v_mul(p1, 3 * (1 - t) ** 2 * t)),
        _v_mul(p2, 3 * (1 - t) * t ** 2)),
        _v_mul(p3, t ** 3))


def _arc_length(t, s, v, w, e, n=100):
    dt = t / n
    length = 0.0
    for i in range(n):
        p1 = _b_prime(i * dt, s, v, w, e)
        p2 = _b_prime((i + 1) * dt, s, v, w, e)
        length += 0.5 * (_v_len(p1) + _v_len(p2)) * dt
    return length


def _find_t(l, s, v, w, e, tol=1e-6):
    a, b = 0.0, 1.0
    while (b - a) > tol:
        mid = (a + b) / 2.0
        if _arc_length(mid, s, v, w, e) < l:
            a = mid
        else:
            b = mid
    return (a + b) / 2.0


def _point_at_ratio(l, s, v, w, e):
    total = _arc_length(1.0, s, v, w, e)
    t = _find_t(l * total, s, v, w, e)
    return _cubic_bezier(t, s, v, w, e)


def _nice_between_points(point_orig, dir_orig, point_targ, dir_targ):
    d = _v_dist(point_orig, point_targ)
    b1_dir = _v_set_length(_v_sub(point_orig, dir_orig), d / 3)
    between1 = _v_add(point_orig, b1_dir)
    b2_dir = _v_set_length(_v_sub(point_targ, dir_targ), d / 3)
    between2 = _v_add(point_targ, b2_dir)
    return [between1, b1_dir, between2, b2_dir]


def connector_point(fr: Frame, long_pos: float, lat_pos: float,
                    sec_from: Section, sec_to: Section) -> tuple[float, float]:
    """Replica of the connector branch in script.h: reconstruct the Bezier arc and
    place the car at arc-length ratio long_pos / arc_length."""
    arc_origin = to_global(fr, sec_from.end, lat_pos, sec_from)   # drain of "from"
    arc_origin_from = to_global(fr, 0.0, lat_pos, sec_from)       # source of "from"
    arc_target = to_global(fr, 0.0, lat_pos, sec_to)             # source of "to"
    arc_target_from = to_global(fr, sec_to.end, lat_pos, sec_to)  # drain of "to"
    nice = _nice_between_points(arc_origin, arc_origin_from, arc_target, arc_target_from)
    arc_length = _arc_length(1.0, arc_origin, nice[0], nice[2], arc_target)
    rel = long_pos / arc_length if arc_length > 0 else 0.0
    rel = max(0.0, min(1.0, rel))
    return _point_at_ratio(rel, arc_origin, nice[0], nice[2], arc_target)


def collect_dots(states: list[dict[str, str]], fr: Frame,
                 angle_override: dict[int, float] | None = None) -> list[Dot]:
    dots: list[Dot] = []
    # discover vehicle tokens
    veh_tokens: list[str] = []
    for key in states[-1]:
        m = VEH_ABSPOS_RE.match(key)
        if m:
            veh_tokens.append(m.group(1))
    veh_tokens = sorted(set(veh_tokens))

    for step, st in enumerate(states):
        for veh in veh_tokens:
            lane = fnum(st, f"env.veh___{veh}___.on_normalized_lane")
            abspos = fnum(st, f"env.veh___{veh}___.abs_pos")
            on_sec = fnum(st, f"env.veh___{veh}___.on_straight_section")
            if lane is None or abspos is None or on_sec is None:
                continue
            if lane < 0 or abspos < 0:
                continue
            long_pos = abspos / fr.dist_scale * 1000.0
            lp = lat_pos_for_lane(fr, lane)
            sec_idx = int(on_sec)
            if sec_idx >= 0 and sec_idx in fr.sections:
                sec = fr.sections[sec_idx]
                if angle_override and sec_idx in angle_override:
                    sec = Section(sec.idx, sec.source, angle_override[sec_idx], sec.end)
                x, y = to_global(fr, long_pos, lp, sec)
            else:
                # connector / traversion: reconstruct the Bezier arc between the two sections.
                t_from = fnum(st, f"env.veh___{veh}___.traversion_from")
                t_to = fnum(st, f"env.veh___{veh}___.traversion_to")
                if (t_from is not None and t_to is not None
                        and int(t_from) in fr.sections and int(t_to) in fr.sections):
                    sec_from = fr.sections[int(t_from)]
                    sec_to = fr.sections[int(t_to)]
                    if angle_override:
                        if sec_from.idx in angle_override:
                            sec_from = Section(sec_from.idx, sec_from.source,
                                               angle_override[sec_from.idx], sec_from.end)
                        if sec_to.idx in angle_override:
                            sec_to = Section(sec_to.idx, sec_to.source,
                                             angle_override[sec_to.idx], sec_to.end)
                    x, y = connector_point(fr, long_pos, lp, sec_from, sec_to)
                else:
                    x, y = long_pos, lp
            dots.append(Dot(x, y, veh, lane, step, sec_idx))
    return dots


def road_polygon(fr: Frame, sec: Section):
    """Corners of the drivable band of a section, in global coords."""
    half_w = fr.num_actual * fr.lane_width_he / 2.0
    lc = lat_center(fr)
    lo, hi = lc - half_w, lc + half_w
    corners_local = [(0.0, lo), (sec.end, lo), (sec.end, hi), (0.0, hi)]
    return [to_global(fr, x, y, sec) for x, y in corners_local]


def lane0_edge(fr: Frame, sec: Section):
    """The lane-0 (left-of-travel) edge line of a section, in global coords."""
    hi = lat_center(fr) + fr.num_actual * fr.lane_width_he / 2.0
    return [to_global(fr, 0.0, hi, sec), to_global(fr, sec.end, hi, sec)]


def render(fr: Frame, dots: list[Dot], out: str, title: str,
           angle_override: dict[int, float] | None = None, flip_y: bool = False):
    fig, ax = plt.subplots(figsize=(11, 9))
    step_lat = 2.0 * fr.num_actual / fr.num_tech          # lat per normalized-lane step
    n_norm = int(round(2 * fr.num_tech - 1))               # number of normalized positions
    for n, sec in sorted(fr.sections.items()):
        if angle_override and n in angle_override:
            sec = Section(sec.idx, sec.source, angle_override[n], sec.end)
        poly = road_polygon(fr, sec)
        ax.add_patch(MplPolygon(poly, closed=True, facecolor="0.85",
                                edgecolor="0.4", zorder=1))
        # normalized-lane grid: even = actual lane center (solid), odd = half position (dotted)
        for k in range(n_norm):
            lat = y_max_tech(fr) - k * step_lat
            g0 = to_global(fr, 0.0, lat, sec)
            g1 = to_global(fr, sec.end, lat, sec)
            if k % 2 == 0:
                ax.plot([g0[0], g1[0]], [g0[1], g1[1]], "-", color="0.55",
                        lw=1.0, zorder=2)
            else:
                ax.plot([g0[0], g1[0]], [g0[1], g1[1]], ":", color="0.7",
                        lw=0.8, zorder=2)
        # centerline
        c0 = to_global(fr, 0.0, lat_center(fr), sec)
        c1 = to_global(fr, sec.end, lat_center(fr), sec)
        ax.plot([c0[0], c1[0]], [c0[1], c1[1]], "--", color="orange", lw=1, zorder=2)
        # lane-0 (left-of-travel) edge highlighted
        e0, e1 = lane0_edge(fr, sec)
        ax.plot([e0[0], e1[0]], [e0[1], e1[1]], "-", color="green", lw=2.5, zorder=3)
        # section label near source
        sx, sy = sec.source
        ax.annotate(f"sec {n}\n(angle {sec.angle_deg:g}°)", (sx, sy),
                    color="navy", fontsize=9, zorder=6)

    # connector arcs between consecutive sections (from = n, to = n+1)
    def _sec_ov(n):
        s = fr.sections[n]
        if angle_override and n in angle_override:
            return Section(s.idx, s.source, angle_override[n], s.end)
        return s

    for n in sorted(fr.sections):
        if n + 1 not in fr.sections:
            continue
        sf, stg = _sec_ov(n), _sec_ov(n + 1)
        for k in range(n_norm):
            lat = y_max_tech(fr) - k * step_lat
            ao = to_global(fr, sf.end, lat, sf)
            aof = to_global(fr, 0.0, lat, sf)
            at = to_global(fr, 0.0, lat, stg)
            atf = to_global(fr, stg.end, lat, stg)
            nice = _nice_between_points(ao, aof, at, atf)
            pts = [_cubic_bezier(t / 40.0, ao, nice[0], nice[2], at) for t in range(41)]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            style = "-" if k % 2 == 0 else ":"
            col = "0.6" if k % 2 == 0 else "0.75"
            ax.plot(xs, ys, style, color=col, lw=0.8, zorder=2)

    # dots colored per vehicle
    veh_colors = {}
    palette = plt.cm.tab10.colors
    for d in dots:
        if d.veh not in veh_colors:
            veh_colors[d.veh] = palette[len(veh_colors) % len(palette)]
        color = "0.5" if d.section < 0 else veh_colors[d.veh]
        ax.plot(d.x, d.y, "o", color=color, ms=7,
                markeredgecolor="black", markeredgewidth=0.5, zorder=5)
        ax.annotate(f"{int(d.lane)}", (d.x, d.y), fontsize=6, color="black",
                    ha="center", va="center", zorder=7)

    ax.set_aspect("equal", "datalim")
    if flip_y:
        ax.invert_yaxis()  # match highway-env screen orientation (y down)
    orient = "screen (y down, like highway-env)" if flip_y else "world (y up)"
    ax.set_title(title + f"\n[{orient}] green=lane-0 edge; solid grey=lane centers; "
                 "dotted=half positions; number=normalized_lane; grey dot=connector")
    ax.grid(True, ls=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def check_on_road(fr: Frame, dots: list[Dot],
                  angle_override: dict[int, float] | None = None) -> None:
    """Report any straight-section dot that falls outside its road band (lat check)."""
    half_w = fr.num_actual * fr.lane_width_he / 2.0
    lc = lat_center(fr)
    bad = 0
    for d in dots:
        if d.section < 0 or d.section not in fr.sections:
            continue
        sec = fr.sections[d.section]
        if angle_override and d.section in angle_override:
            sec = Section(sec.idx, sec.source, angle_override[d.section], sec.end)
        # invert the transform: recover local lat by rotating back about source+correction
        ang = math.radians(sec.angle_deg)
        ca, sa = math.cos(ang), math.sin(ang)
        px = d.x - sec.source[0] - lc * sa
        py = d.y - sec.source[1] - lc * (1.0 - ca)
        # local = R(-ang) * (px, py)
        loc_lat = -px * sa + py * ca
        if not (lc - half_w - 1e-3 <= loc_lat <= lc + half_w + 1e-3):
            bad += 1
            print(f"  OFF-ROAD: veh {d.veh} step {d.step} sec {d.section} "
                  f"lane {d.lane:g} -> local lat {loc_lat:.2f} not in "
                  f"[{lc - half_w:.2f}, {lc + half_w:.2f}]")
    if bad == 0:
        print("  all straight-section dots are within their road band (lateral check OK)")
    else:
        print(f"  {bad} dot(s) OFF-ROAD")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--out", default="section_dots.png")
    ap.add_argument("--sweep-angle", default="",
                    help="comma-separated angles (deg) to re-render section 1 with")
    ap.add_argument("--flip-y", action="store_true",
                    help="mirror vertically to match highway-env screen orientation")
    args = ap.parse_args()

    states = parse_trace(args.trace)
    if not states:
        print("no states parsed from trace", file=sys.stderr)
        return 2
    fr = build_frame(states)
    print(f"frame: lane_width_he={fr.lane_width_he} num_tech={fr.num_tech} "
          f"num_actual={fr.num_actual} dist_scale={fr.dist_scale}")
    print(f"y_max_tech={y_max_tech(fr):.2f} lat_center={lat_center(fr):.2f}")
    for n, s in sorted(fr.sections.items()):
        print(f"  section {n}: source={s.source} angle={s.angle_deg}° end={s.end:g}")

    dots = collect_dots(states, fr)
    print(f"dots: {len(dots)} total "
          f"({sum(1 for d in dots if d.section >= 0)} on straight sections, "
          f"{sum(1 for d in dots if d.section < 0)} on connectors)")
    check_on_road(fr, dots)
    render(fr, dots, args.out, f"section dots \u2014 {args.trace}", flip_y=args.flip_y)

    if args.sweep_angle.strip():
        angles = [float(a) for a in args.sweep_angle.split(",") if a.strip()]
        for a in angles:
            ov = {1: a}
            sdots = collect_dots(states, fr, angle_override=ov)
            out = args.out.replace(".png", f"_sec1angle{int(a)}.png")
            print(f"[sweep] section 1 @ {a}°:")
            check_on_road(fr, sdots, angle_override=ov)
            render(fr, sdots, out, f"sweep: section 1 @ {a}°", angle_override=ov,
                   flip_y=args.flip_y)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
