"""Find free-obstacle placements that force exactly PARK_MOVES parking moves (CEGIS loop).

Usage (repo root, venv active):
    python3 parking/synth_obstacles.py [--config src/templates/envmodel_config.tpl.json]
                                       [--max-iters 30] [--max-sections 6] [--apply] [--no-regen]
                                       [--history DIR]

Every candidate and refutation is snapshotted (trace + EnvModel.smv) into the history dir
(default <examples>/synth_history), with progress events in its events.jsonl (read by mc_parker).

Free obstacles are the RECT_OBSTACLES_* boxes with a non-zero RECT_OBSTACLES_SHIFT_* range.
Each iteration
  1. synthesizes shifts so that some road graph's shortest route needs exactly N = PARK_MOVES
     moves (subject to all lemmas learned so far), then
  2. fixes these shifts and checks whether the target is reachable within N - 1 moves.
If no package finds such a shorter route, the placement provably needs exactly N moves (within
the model: section count, angle grid, turning radius, straight connector approximation).
Otherwise the shorter route yields a lemma "some free obstacle blocks this route" and we repeat.
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from fractions import Fraction

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mc_parker as mp  # noqa: E402

TARGET_SECTION = 1
EGO_WIDTH_M = 2.0
LEMMA_BEGIN = "-- BEGIN OBSTACLE SYNTH LEMMAS (patched by parking/synth_obstacles.py)"
LEMMA_END = "-- END OBSTACLE SYNTH LEMMAS"
_FREE_DECL_RE = re.compile(
    r"(rect_obstacles_(shift_kx|shift_ky|grow_kw|grow_kh)_(\d+)\s*:\s*)(-?\d+)\s*\.\.\s*(-?\d+)(\s*;)")
FREE_RANGE_KEYS = (("RECT_OBSTACLES_SHIFT_MIN_Xs", "RECT_OBSTACLES_SHIFT_MAX_Xs"),
                   ("RECT_OBSTACLES_SHIFT_MIN_Ys", "RECT_OBSTACLES_SHIFT_MAX_Ys"),
                   ("RECT_OBSTACLES_GROW_MIN_Ws", "RECT_OBSTACLES_GROW_MAX_Ws"),
                   ("RECT_OBSTACLES_GROW_MIN_Hs", "RECT_OBSTACLES_GROW_MAX_Hs"))
_GOAL_LINE_RE = re.compile(r"^(\s*is_target_reachable\s*:=.*)$", re.MULTILINE)
_TRACE_LINE_RE = re.compile(r"^\s*(?:env\.)?([A-Za-z_][\w.]*)\s*=\s*(-?\d+)\s*$")
_SECTIONS_RE = re.compile(r"Found variable SECTIONS with value (\d+)")
_events_path = None


def emit(kind, **data):
    if _events_path:
        with open(_events_path, "a") as f:
            f.write(json.dumps({"type": kind, **data}) + "\n")


def fail(msg):
    emit("error", message=msg)
    sys.exit(msg)


def start_history(history_dir):
    global _events_path
    os.makedirs(history_dir, exist_ok=True)
    for name in os.listdir(history_dir):
        if re.fullmatch(r"iter\d+_\w+", name):
            shutil.rmtree(os.path.join(history_dir, name), ignore_errors=True)
    _events_path = os.path.join(history_dir, "events.jsonl")
    open(_events_path, "w").close()


def snapshot(history_dir, name, pkg):
    """Copy a package's trace and model so the GUI can show this step after the packages move on."""
    d = os.path.join(history_dir, name)
    os.makedirs(d, exist_ok=True)
    for f in ("debug_trace_array.txt", "EnvModel.smv"):
        shutil.copy2(os.path.join(pkg, f), d)
    return d


def load_config(tpl_path):
    with open(tpl_path) as f:
        cfg = json.load(f)["#TEMPLATE"]

    def floats(key):
        return [float(v) for v in mp._parse_paren_list(cfg.get(key, ""))]

    obstacles = list(zip(floats("RECT_OBSTACLES_TL_Xs"), floats("RECT_OBSTACLES_TL_Ys"),
                         floats("RECT_OBSTACLES_BR_Xs"), floats("RECT_OBSTACLES_BR_Ys")))
    n = len(obstacles)
    step = float(cfg.get("RECT_OBSTACLES_SHIFT_STEP", 1))
    errors = []
    moves = int(cfg.get("PARK_MOVES", 0))
    if moves < 1:
        errors.append("PARK_MOVES must be >= 1.")
    if not cfg.get("REACHABILITY_ONLY", False):
        errors.append("REACHABILITY_ONLY must be true.")
    any_free = False
    for lo_key, hi_key in FREE_RANGE_KEYS:
        lo, hi = floats(lo_key), floats(hi_key)
        for key, vals in ((lo_key, lo), (hi_key, hi)):
            for v in vals:
                if abs(v / step - round(v / step)) > 1e-9:
                    errors.append(f"{key}: {v} is not a multiple of RECT_OBSTACLES_SHIFT_STEP={step}.")
        lo, hi = (lo + [0.0] * n)[:n], (hi + [0.0] * n)[:n]
        any_free = any_free or any(a != b for a, b in zip(lo, hi))
    if not errors and not any_free:
        errors.append("No free obstacle: give at least one RECT_OBSTACLES_SHIFT_*/GROW_* range min < max.")
    if errors:
        fail("Config error in " + tpl_path + ":\n  " + "\n  ".join(errors))
    if int(cfg.get("BMC_CNT", 0)) > 0:
        print(f"WARNING: BMC_CNT={cfg['BMC_CNT']} > 0. Bound 0 is already complete here; larger bounds only slow down.")
    return cfg, moves, obstacles, step


def vfm_script_path(tpl_path):
    """Config path as seen from bin/, the worker's working directory."""
    return os.path.relpath(os.path.abspath(tpl_path), os.path.join(mp.REPO_ROOT, "bin"))


def generated_prefix_dir(cfg):
    return os.path.normpath(os.path.join(mp.REPO_ROOT, "bin", cfg["_GENERATED_PATH"]))


def discover_packages(prefix_dir):
    parent, prefix = os.path.dirname(prefix_dir), os.path.basename(prefix_dir)
    if not os.path.isdir(parent):
        return []
    return [os.path.join(parent, n) for n in sorted(os.listdir(parent))
            if n.startswith(prefix) and n != prefix and os.path.isdir(os.path.join(parent, n))]


def regenerate(tpl_path, prefix_dir):
    for p in discover_packages(prefix_dir):
        shutil.rmtree(p, ignore_errors=True)
    print("Generating EnvModel packages...")
    subprocess.run([sys.executable, mp._MC_WORKER, f"@{{{vfm_script_path(tpl_path)}}}@.generateEnvmodels"],
                   cwd=mp.REPO_ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=False)


def smv_path(pkg):
    return os.path.join(pkg, "EnvModel.smv")


def read_text(path):
    with open(path, "r", errors="replace") as f:
        return f.read()


def read_raw_trace(trace_path):
    """{name: int} of the initial state, in model units (all relevant vars are frozen)."""
    values = {}
    for line in read_text(trace_path).splitlines():
        m = _TRACE_LINE_RE.match(line)
        if m and m.group(1) not in values:
            values[m.group(1)] = int(m.group(2))
    return values


def geometry_factor(text):
    """Model units per metre (GEOMETRYSCALING header) as an exact fraction."""
    m = re.search(r"-- GEOMETRYSCALING\(\(\(\((-?[\d.]+)\)\)\)\)", text)
    return Fraction(m.group(1)) if m else Fraction(1)


def free_obstacles(base_text):
    """{obs: {'shift_kx'|'shift_ky'|'grow_kw'|'grow_kh': (lo, hi)}} from the FROZENVAR declarations."""
    free = {}
    for m in _FREE_DECL_RE.finditer(base_text):
        free.setdefault(int(m.group(3)), {})[m.group(2)] = (int(m.group(4)), int(m.group(5)))
    return free


def render_lemma(boxes, obs_list, g):
    """INIT: some obstacle of obs_list overlaps some box (world-coordinate Fractions)."""
    def le(coef_var, bound):  # var <= bound with bound = n/d  <=>  d*var <= n
        b = bound * g
        return f"{b.denominator}*{coef_var} <= {b.numerator}" if b.denominator != 1 else f"{coef_var} <= {b.numerator}"

    def ge(coef_var, bound):
        b = bound * g
        return f"{b.denominator}*{coef_var} >= {b.numerator}" if b.denominator != 1 else f"{coef_var} >= {b.numerator}"

    terms = []
    for o in sorted(obs_list):
        for xmin, xmax, ymin, ymax in boxes:
            terms.append(f"({le(f'rect_obstacles_tl_x_{o}', xmax)} & {ge(f'rect_obstacles_br_x_{o}', xmin)}"
                         f" & {le(f'rect_obstacles_tl_y_{o}', ymax)} & {ge(f'rect_obstacles_br_y_{o}', ymin)})")
    return "INIT " + ("\n   | ".join(terms) if terms else "FALSE") + ";"


def patch_model(base_text, goal_moves, fixed_shifts, clauses, g, exact=False):
    """clauses: [(obstacle indices, boxes)], each rendered as one render_lemma INIT.

    exact: reachable within goal_moves but NOT within goal_moves - 1 in the same road graph.
    """
    def goal(m):
        head, expr = m.group(1).split(":=", 1)
        expr = expr.strip().rstrip(";")

        def within(k):
            return re.sub(r"_within_\d+_moves", f"_within_{k}_moves", expr)
        excl = f" & !({within(goal_moves - 1)})" if exact and goal_moves > 1 else ""
        return f"{head}:= ({within(goal_moves)}){excl};"
    text = _GOAL_LINE_RE.sub(goal, base_text, count=1)
    if fixed_shifts is not None:
        def fix(m):
            v = fixed_shifts[int(m.group(3))][m.group(2)]
            return f"{m.group(1)}{v} .. {v}{m.group(6)}"
        text = _FREE_DECL_RE.sub(fix, text)
    block = "\n".join(render_lemma(boxes, obs, g) for obs, boxes in clauses)
    begin, end = text.index(LEMMA_BEGIN) + len(LEMMA_BEGIN), text.index(LEMMA_END)
    return text[:begin] + "\n" + block + ("\n" if block else "") + text[end:]


def write_packages(packages, base, goal_moves, fixed_shifts, clauses, exact=False):
    for p in packages:
        with open(smv_path(p), "w") as f:
            f.write(patch_model(base[p], goal_moves, fixed_shifts, clauses, geometry_factor(base[p]), exact))


def hug_groups(tpl_path, mode):
    """Point groups around the (fixed) target slot that free boxes should hug, [] if not fixed.

    slot: one group (the section's ends and middle). ends: in front of / behind the parked car
    (parallel parking), sides: left / right of it (perpendicular parking); with two groups, the
    hugging boxes are spread over both.
    """
    values = mp._config_values(tpl_path)
    geom = mp._section_geom(values.get, TARGET_SECTION)
    if geom is None:
        return []
    (sx, sy), (dx, dy) = geom
    mx, my = (sx + dx) / 2, (sy + dy) / 2
    length = math.hypot(dx - sx, dy - sy) or 1.0
    ux, uy = (dx - sx) / length, (dy - sy) / length
    if mode == "ends":
        a, (vx, vy) = mp.EGO_LENGTH_M / 2 + 0.5, (ux, uy)
        groups = [[(mx + vx * a, my + vy * a)], [(mx - vx * a, my - vy * a)]]
    elif mode == "sides":
        a, (vx, vy) = EGO_WIDTH_M / 2 + 0.5, (-uy, ux)
        groups = [[(mx + vx * a, my + vy * a)], [(mx - vx * a, my - vy * a)]]
    else:
        groups = [[(sx, sy), (mx, my), (dx, dy)]]
    return [[(Fraction(x).limit_denominator(1000), Fraction(y).limit_denominator(1000)) for x, y in g]
            for g in groups]


def all_proven(packages):
    """Every package finished THIS run without a counterexample (BMC 'no cex' / IC3 'is true')."""
    for p in packages:
        trace = mp.package_trace_path(p)
        if not os.path.isfile(trace) or os.path.getmtime(trace) < os.path.getmtime(smv_path(p)):
            return False
        content = read_text(trace)
        if mp.trace_has_counterexample(trace) or not ("is true" in content or "no counterexample found" in content):
            return False
    return True


def race(packages, mc_script, label, **phase):
    emit("phase", label=label, **phase)
    t0 = time.time()
    print(f"[{label}]", end=" ", flush=True)
    win = mp.race_first_solution(packages=packages, mc_script=mc_script)
    print(f"[{label}] {time.time() - t0:.0f}s")
    if win is None and not all_proven(packages):
        fail(f"[{label}] Aborted: not every package finished (interrupted or MC failure).")
    return win, time.time() - t0


def route_boxes(raw, smv_text, target_dirs, margin):
    """Lemma boxes along the min-move route: (plain, directed, moves, states).

    Plain boxes are exactly the model's obstacle checks (a free box overlapping any of them blocks
    the route). Directed ones are inverted boxes around points on the route: overlapping one means
    covering that point with 'margin' to spare, so slightly shifted variants of the route are
    blocked too. Directed implies plain.
    """
    states = mp.route_states(raw.get, TARGET_SECTION, target_dirs)
    if states is None:
        fail("Counterexample trace holds no route to the target (trace parsing problem?).")
    g = geometry_factor(smv_text)
    scale = raw.get("connector_curve_scale")
    d = Fraction(margin).limit_denominator(1000)

    def pt(sec, end):
        return raw[f"section_{sec}.{end}.x"], raw[f"section_{sec}.{end}.y"]

    boxes, points = set(), set()
    for sec in {s for s, _ in states}:
        for end in ("source", "drain"):
            x, y = pt(sec, end)
            boxes.add((Fraction(x) / g, Fraction(x) / g, Fraction(y) / g, Fraction(y) / g))
            points.add((Fraction(x) / g, Fraction(y) / g))
    for (s1, d1), (s2, _) in zip(states, states[1:]):
        if s1 == s2 or scale is None:
            continue
        a, b = (s1, s2) if d1 == "F" else (s2, s1)
        j = 0
        while f"conn_{a}_to_{b}_arc_{j}_q0_x" in raw:
            q = [(Fraction(raw[f"conn_{a}_to_{b}_arc_{j}_q{i}_x"], scale) / g,
                  Fraction(raw[f"conn_{a}_to_{b}_arc_{j}_q{i}_y"], scale) / g) for i in range(4)]
            xs, ys = [p[0] for p in q], [p[1] for p in q]
            boxes.add((min(xs), max(xs), min(ys), max(ys)))
            # Sub-arc start, end and middle (de Casteljau) lie on the curve, inside the hull box.
            points.update((q[0], q[3], ((q[0][0] + 3 * q[1][0] + 3 * q[2][0] + q[3][0]) / 8,
                                        (q[0][1] + 3 * q[1][1] + 3 * q[2][1] + q[3][1]) / 8)))
            j += 1
    directed = sorted((x + d, x - d, y + d, y - d) for x, y in points)
    moves = 1 + sum(1 for (a, _), (b, _) in zip(states, states[1:]) if a == b)
    return sorted(boxes), directed, moves, states


def world_obstacles(raw, g, n_obs):
    return [tuple(float(Fraction(raw[f"rect_obstacles_{f}_{o}"]) / g) for f in ("tl_x", "tl_y", "br_x", "br_y"))
            for o in range(n_obs)]


def write_recheck_config(tpl_path, recheck_dir, obstacles_world, goal_moves, geometry_scaling):
    """Copy of the config with the candidate's boxes fixed, a finer geometry grid and goal_moves."""
    content = read_text(tpl_path)
    for key, idx in (("RECT_OBSTACLES_TL_Xs", 0), ("RECT_OBSTACLES_TL_Ys", 1),
                     ("RECT_OBSTACLES_BR_Xs", 2), ("RECT_OBSTACLES_BR_Ys", 3)):
        content = mp._replace_tpl_string_value(
            content, key, mp._format_paren_list([mp._fmt_world(o[idx]) for o in obstacles_world]))
    for key in (k for pair in FREE_RANGE_KEYS for k in pair):
        if f'"{key}"' in content:
            content = mp._replace_tpl_string_value(content, key, "")
    content = re.sub(r'("PARK_MOVES"\s*:\s*)\d+', lambda m: f"{m.group(1)}{goal_moves}", content)
    content = mp._replace_tpl_string_value(content, "GEOMETRYSCALING", f"#{{{geometry_scaling}}}#")
    bin_dir = os.path.join(mp.REPO_ROOT, "bin")
    for key, sub in (("_GENERATED_PATH", "gp"), ("_CACHED_PATH", "tmp")):
        content = mp._replace_tpl_string_value(content, key, os.path.relpath(os.path.join(recheck_dir, sub), bin_dir))
    path = os.path.join(recheck_dir, "recheck.tpl.json")
    with open(path, "w") as f:
        f.write(content)
    if re.search(r'"#\d+"\s*:\s*"[^"]*@geomgrid\b', content):
        mp.patch_tpl_variants(path, "geomgrid", [geometry_scaling])
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=mp.TPL_PATH)
    ap.add_argument("--max-iters", type=int, default=30)
    ap.add_argument("--max-sections", type=int, default=6,
                    help="Refuse packages with more sections (their proofs get very slow).")
    ap.add_argument("--apply", action="store_true",
                    help="Write the result into RECT_OBSTACLES_*, fix all shifts and regenerate.")
    ap.add_argument("--no-regen", action="store_true", help="Reuse the already generated packages.")
    ap.add_argument("--history", help="Snapshot/event dir (default: <examples>/synth_history).")
    ap.add_argument("--margin", type=float, default=0.5,
                    help="Directed search: first look for placements covering each refuting route with "
                         "this margin (m); 0 = plain blocking only.")
    ap.add_argument("--no-seed", action="store_true",
                    help="Skip checking the configured placement for shorter routes first.")
    ap.add_argument("--hug", default="1,3",
                    help="Clearance ladder (m, comma-separated): first require every free box within "
                         "this distance of the target slot (tight gaps force moves, like parallel "
                         "parking), relaxing step by step and finally dropping it; '' = off.")
    ap.add_argument("--hug-at", choices=("slot", "ends", "sides"),
                    help="Where boxes hug the target: slot = anywhere around it, ends = in front of and "
                         "behind the car (parallel parking), sides = left and right of it (perpendicular "
                         "parking). Default: SYNTH_HUG_AT from the config, else slot.")
    ap.add_argument("--recheck-factor", type=int, default=2,
                    help="Before certifying, recheck the placement on a geometry grid this many times finer "
                         "(catches shortcuts the coarse grid cannot represent); 0 or 1 = off.")
    args = ap.parse_args()

    if args.history:
        args.history = os.path.abspath(args.history)
        start_history(args.history)
    tpl_path = os.path.abspath(args.config)
    cfg, n_moves, obstacles, step = load_config(tpl_path)
    prefix_dir = generated_prefix_dir(cfg)
    history_dir = args.history or os.path.join(os.path.dirname(prefix_dir), "synth_history")
    if not args.history:
        start_history(history_dir)
    mc_script = f"@{{{vfm_script_path(tpl_path)}}}@.runMCJobs[16]"

    if not args.no_regen:
        emit("phase", label="Generating EnvModel packages", phase="regen")
        regenerate(tpl_path, prefix_dir)
    packages = discover_packages(prefix_dir)
    if not packages:
        fail(f"No packages under {os.path.dirname(prefix_dir)}; generation failed?")
    # The loop rewrites EnvModel.smv in place; keep the generated original for --no-regen reruns.
    for p in packages:
        pristine = smv_path(p) + ".base"
        if not args.no_regen or not os.path.isfile(pristine):
            shutil.copy2(smv_path(p), pristine)
    base = {p: read_text(smv_path(p) + ".base") for p in packages}
    too_big = [os.path.basename(p) for p in packages
               if int((_SECTIONS_RE.search(base[p]) or [0, 0])[1]) > args.max_sections]
    if too_big:
        fail(f"Packages with more than {args.max_sections} sections: {too_big}. "
             "Limit the @sections range in the config.")
    free_obs = free_obstacles(base[packages[0]])
    if not free_obs or "_within_" not in base[packages[0]]:
        fail("Generated model lacks free obstacles or move-limited reachability; regenerate.")
    max_sections = max(int((_SECTIONS_RE.search(base[p]) or [0, 0])[1]) for p in packages)
    if n_moves > max_sections - 1:
        fail(f"PARK_MOVES={n_moves} is unreachable with at most {max_sections} sections: every gear "
             f"change needs its own dead-end section (start and target can't serve), so moves <= "
             f"SECTIONS - 1. Allow {n_moves + 1} sections or lower PARK_MOVES.")
    target_dirs = mp.target_park_in_dirs(smv_path(packages[0]))
    hug_at = args.hug_at or cfg.get("SYNTH_HUG_AT", "slot")
    groups = hug_groups(tpl_path, hug_at)
    hug_points = [p for g in groups for p in g]

    def huggers(c):
        """Free boxes whose shift/grow ranges can bring them within c of the target slot."""
        out = []
        for o, r in free_obs.items():
            tl_x, tl_y, br_x, br_y = obstacles[o]
            lo_x, lo_y = tl_x + step * r["shift_kx"][0], tl_y + step * r["shift_ky"][0]
            hi_x = br_x + step * (r["shift_kx"][1] + r["grow_kw"][1])
            hi_y = br_y + step * (r["shift_ky"][1] + r["grow_kh"][1])
            if any(lo_x <= x + c and hi_x >= x - c and lo_y <= y + c and hi_y >= y - c for x, y in hug_points):
                out.append(o)
        return out

    hugs = [Fraction(v).limit_denominator(1000) for v in args.hug.split(",") if v.strip()] if hug_points else []
    hugs = [c for c in hugs if huggers(c)] + [None]
    print(f"{len(packages)} package(s), free obstacles {sorted(free_obs)}, target: exactly {n_moves} move(s).")
    emit("start", packages=[os.path.basename(p) for p in packages], moves=n_moves,
         free=sorted(free_obs), max_iters=args.max_iters, hug=[float(h) for h in hugs if h is not None],
         hug_at=hug_at)

    lemmas, shifts, certified = [], None, False
    directed = args.margin > 0
    hug_idx = 0

    def clauses():
        out = [(sorted(free_obs), strong if directed else plain) for plain, strong in lemmas]
        c = hugs[hug_idx]
        if c is not None:
            boxes = huggers(c)
            # Enough boxes for every group: spread them (front/back or left/right), else any point.
            spread = len(boxes) >= len(groups)
            for i, o in enumerate(boxes):
                pts = groups[i % len(groups)] if spread else hug_points
                out.append(([o], [(x - c, x + c, y - c, y + c) for x, y in pts]))
        return out

    def refute(win, it, secs, obstacles_world, recheck=False):
        raw = read_raw_trace(mp.package_trace_path(win))
        plain, strong, moves, states = route_boxes(raw, read_text(smv_path(win)), target_dirs, args.margin)
        print(f"  refuted by a {moves}-move route in {os.path.basename(win)}{' (finer grid)' if recheck else ''}: {states}")
        lemmas.append((plain, strong))
        emit("refuted", iter=it, snapshot=snapshot(history_dir, f"iter{it:02d}_refutation", win),
             package=os.path.basename(win), moves=moves, lemmas=len(lemmas), obstacles=obstacles_world,
             free=sorted(free_obs), seconds=secs, recheck=recheck)

    recheck_dir = os.path.join(os.path.dirname(prefix_dir), "synth_recheck")
    fine_scaling = round(float(geometry_factor(base[packages[0]])) * 1000 * args.recheck_factor)

    def recheck(it, obstacles_world):
        """Shorter route on the finer grid for the fixed placement: (package, seconds) or (None, seconds)."""
        shutil.rmtree(recheck_dir, ignore_errors=True)
        os.makedirs(recheck_dir)
        cfg_path = write_recheck_config(tpl_path, recheck_dir, obstacles_world, n_moves - 1, fine_scaling)
        emit("phase", label=f"iter {it}: recheck < {n_moves} moves on finer grid", phase="recheck",
             iter=it, grid=1000 / fine_scaling)
        regenerate(cfg_path, os.path.join(recheck_dir, "gp"))
        fine = discover_packages(os.path.join(recheck_dir, "gp"))
        if not fine:
            fail("Finer-grid recheck: package generation failed.")
        return race(fine, f"@{{{vfm_script_path(cfg_path)}}}@.runMCJobs[16]",
                    f"iter {it}: recheck < {n_moves} moves (grid {1000 / fine_scaling:g} m)")

    if n_moves > 1 and not args.no_seed:
        # Start from the configured boxes: the shorter route they allow is the first one to block.
        seed = {o: {p: min(max(0, lo), hi) for p, (lo, hi) in free_obs[o].items()} for o in free_obs}
        write_packages(packages, base, n_moves - 1, seed, [])
        win, secs = race(packages, mc_script, f"seed: verify < {n_moves} moves", phase="verify", iter=0)
        if win is not None:
            seed_raw = read_raw_trace(mp.package_trace_path(win))
            refute(win, 0, secs, world_obstacles(seed_raw, geometry_factor(base[win]), len(obstacles)))

    for it in range(1, args.max_iters + 1):
        while True:
            write_packages(packages, base, n_moves, None, clauses(), exact=True)
            win, secs = race(packages, mc_script, f"iter {it}: synth = {n_moves} moves", phase="synth", iter=it)
            if win is not None:
                break
            # Preferences are relaxed in order; only the fully relaxed (plain) search proves impossibility.
            if directed and lemmas:
                directed = False
                print(f"  no placement covers all refuting routes with {args.margin} m margin; plain blocking now")
                emit("fallback", reason="margin", iter=it, margin=args.margin)
            elif hugs[hug_idx] is not None:
                hug_idx += 1
                directed = args.margin > 0
                nxt = hugs[hug_idx]
                print(f"  no placement with all free boxes within {float(hugs[hug_idx - 1]):g} m of the target "
                      f"works; now {'%g m' % float(nxt) if nxt is not None else 'anywhere'}")
                emit("fallback", reason="hug", iter=it, previous=float(hugs[hug_idx - 1]),
                     hug=None if nxt is None else float(nxt))
            else:
                break
        if win is None:
            print(f"No placement of the free obstacles forces exactly {n_moves} moves (within the model).")
            emit("impossible", iter=it, lemmas=len(lemmas), moves=n_moves, seconds=secs)
            return 2
        raw = read_raw_trace(mp.package_trace_path(win))
        synth_win, synth_trace = win, read_text(mp.package_trace_path(win))
        shifts = {o: {p: raw[f"rect_obstacles_{p}_{o}"] for p in free_obs[o]} for o in free_obs}
        print(f"  candidate from {os.path.basename(win)}: free parameters (steps) {shifts}")
        cand_obstacles = world_obstacles(raw, geometry_factor(base[win]), len(obstacles))
        cand_states = mp.route_states(raw.get, TARGET_SECTION, target_dirs) or []
        cand_moves = 1 + sum(1 for (a, _), (b, _) in zip(cand_states, cand_states[1:]) if a == b)
        if cand_moves < n_moves:
            # Excluded by the exact goal; if it slips through, its own route already disproves it.
            refute(win, it, secs, cand_obstacles)
            continue
        emit("candidate", iter=it, snapshot=snapshot(history_dir, f"iter{it:02d}_candidate", win),
             package=os.path.basename(win), obstacles=cand_obstacles, free=sorted(free_obs),
             moves=cand_moves, seconds=secs)
        if n_moves == 1:
            certified = True
            break

        write_packages(packages, base, n_moves - 1, shifts, clauses())
        win, secs = race(packages, mc_script, f"iter {it}: verify < {n_moves} moves", phase="verify", iter=it)
        if win is None and args.recheck_factor > 1:
            win, secs = recheck(it, cand_obstacles)
            if win is not None:
                refute(win, it, secs, cand_obstacles, recheck=True)
                continue
        if win is None:
            certified = True
            break
        refute(win, it, secs, cand_obstacles)

    if not certified:
        print(f"Gave up after {args.max_iters} iterations ({len(lemmas)} lemmas).")
        emit("gave_up", iter=args.max_iters, lemmas=len(lemmas))
        return 3

    # Restore the synth solution (goal N, placement fixed) as the freshest CEX for mc_parker.py.
    write_packages(packages, base, n_moves, shifts, clauses(), exact=True)
    for p in packages:
        mp.clear_package_trace(p)
    with open(mp.package_trace_path(synth_win), "w") as f:
        f.write(synth_trace)
    result = world_obstacles(read_raw_trace(mp.package_trace_path(synth_win)),
                             geometry_factor(base[synth_win]), len(obstacles))
    print(f"\nCERTIFIED: these obstacles force exactly {n_moves} move(s) "
          f"(packages {[os.path.basename(p) for p in packages]}):")
    for o, box in enumerate(result):
        print(f"  obstacle {o}{' (free)' if o in free_obs else ''}: "
              f"tl=({box[0]:g}, {box[1]:g}) br=({box[2]:g}, {box[3]:g})")
    emit("certified", iter=it, package=synth_win, obstacles=result, moves=n_moves, lemmas=len(lemmas),
         recheck_grid=1000 / fine_scaling if args.recheck_factor > 1 and n_moves > 1 else None)

    if args.apply:
        mp.patch_tpl_config(tpl_path, None, None, None, None, result)
        content = read_text(tpl_path)
        zeros = mp._format_paren_list(["0"] * len(result))
        for key in (k for pair in FREE_RANGE_KEYS for k in pair):
            if f'"{key}"' in content:
                content = mp._replace_tpl_string_value(content, key, zeros)
        with open(tpl_path, "w") as f:
            f.write(content)
        print(f"Applied to {os.path.relpath(tpl_path, mp.REPO_ROOT)} (all shift/grow ranges reset to 0).")
        regenerate(tpl_path, prefix_dir)
        print("Re-run with: python3 parking/mc_parker.py --race")
    else:
        print(f"Solution in {os.path.basename(synth_win)}; open with: python3 parking/mc_parker.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
