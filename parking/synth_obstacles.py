"""Find free-obstacle placements that force exactly PARK_MOVES parking moves (CEGIS loop).

Usage (repo root, venv active):
    python3 parking/synth_obstacles.py [--config src/templates/envmodel_config.tpl.json]
                                       [--max-iters 30] [--max-sections 6] [--apply] [--no-regen]

Free obstacles are the RECT_OBSTACLES_* boxes with a non-zero RECT_OBSTACLES_SHIFT_* range.
Each iteration
  1. synthesizes shifts so that the target is reachable within N = PARK_MOVES moves
     (subject to all lemmas learned so far), then
  2. fixes these shifts and checks whether the target is reachable within N - 1 moves.
If no package finds such a shorter route, the placement provably needs exactly N moves (within
the model: section count, angle grid, turning radius, straight connector approximation).
Otherwise the shorter route yields a lemma "some free obstacle blocks this route" and we repeat.
"""
import argparse
import json
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
LEMMA_BEGIN = "-- BEGIN OBSTACLE SYNTH LEMMAS (patched by parking/synth_obstacles.py)"
LEMMA_END = "-- END OBSTACLE SYNTH LEMMAS"
_SHIFT_DECL_RE = re.compile(r"(rect_obstacles_shift_k([xy])_(\d+)\s*:\s*)(-?\d+)\s*\.\.\s*(-?\d+)(\s*;)")
_GOAL_LINE_RE = re.compile(r"^(\s*is_target_reachable\s*:=.*)$", re.MULTILINE)
_TRACE_LINE_RE = re.compile(r"^\s*(?:env\.)?([A-Za-z_][\w.]*)\s*=\s*(-?\d+)\s*$")
_K_RE = re.compile(r"conn_\d+_to_\d+_sub_\d+_ksx\s*:=\s*(\d+)\s*\*")
_SECTIONS_RE = re.compile(r"Found variable SECTIONS with value (\d+)")


def load_config(tpl_path):
    with open(tpl_path) as f:
        cfg = json.load(f)["#TEMPLATE"]

    def floats(key):
        return [float(v) for v in mp._parse_paren_list(cfg.get(key, ""))]

    obstacles = list(zip(floats("RECT_OBSTACLES_TL_Xs"), floats("RECT_OBSTACLES_TL_Ys"),
                         floats("RECT_OBSTACLES_BR_Xs"), floats("RECT_OBSTACLES_BR_Ys")))
    shifts = [floats(k) for k in ("RECT_OBSTACLES_SHIFT_MIN_Xs", "RECT_OBSTACLES_SHIFT_MAX_Xs",
                                  "RECT_OBSTACLES_SHIFT_MIN_Ys", "RECT_OBSTACLES_SHIFT_MAX_Ys")]
    step = float(cfg.get("RECT_OBSTACLES_SHIFT_STEP", 1))
    errors = []
    moves = int(cfg.get("PARK_MOVES", 0))
    if moves < 1:
        errors.append("PARK_MOVES must be >= 1.")
    if not cfg.get("REACHABILITY_ONLY", False):
        errors.append("REACHABILITY_ONLY must be true.")
    if any(len(s) < len(obstacles) for s in shifts):
        errors.append(f"All RECT_OBSTACLES_SHIFT_* lists need one entry per obstacle ({len(obstacles)}).")
    elif not any(lo != hi for s_lo, s_hi in ((shifts[0], shifts[1]), (shifts[2], shifts[3]))
                 for lo, hi in list(zip(s_lo, s_hi))[:len(obstacles)]):
        errors.append("No free obstacle: give at least one RECT_OBSTACLES_SHIFT_* range min < max.")
    for v in (x for s in shifts for x in s):
        if abs(v / step - round(v / step)) > 1e-9:
            errors.append(f"Shift bound {v} is not a multiple of RECT_OBSTACLES_SHIFT_STEP={step}.")
    if errors:
        sys.exit("Config error in " + tpl_path + ":\n  " + "\n  ".join(errors))
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
    """{obs: {'x': (lo, hi), 'y': (lo, hi)}} from the FROZENVAR shift declarations."""
    free = {}
    for m in _SHIFT_DECL_RE.finditer(base_text):
        free.setdefault(int(m.group(3)), {})[m.group(2)] = (int(m.group(4)), int(m.group(5)))
    return free


def render_lemma(boxes, free_obs, g):
    """INIT: some free obstacle overlaps some route box (world-coordinate Fractions)."""
    def le(coef_var, bound):  # var <= bound with bound = n/d  <=>  d*var <= n
        b = bound * g
        return f"{b.denominator}*{coef_var} <= {b.numerator}" if b.denominator != 1 else f"{coef_var} <= {b.numerator}"

    def ge(coef_var, bound):
        b = bound * g
        return f"{b.denominator}*{coef_var} >= {b.numerator}" if b.denominator != 1 else f"{coef_var} >= {b.numerator}"

    terms = []
    for o in sorted(free_obs):
        for xmin, xmax, ymin, ymax in boxes:
            terms.append(f"({le(f'rect_obstacles_tl_x_{o}', xmax)} & {ge(f'rect_obstacles_br_x_{o}', xmin)}"
                         f" & {le(f'rect_obstacles_tl_y_{o}', ymax)} & {ge(f'rect_obstacles_br_y_{o}', ymin)})")
    return "INIT " + ("\n   | ".join(terms) if terms else "FALSE") + ";"


def patch_model(base_text, goal_moves, fixed_shifts, lemmas, free_obs, g):
    text = _GOAL_LINE_RE.sub(lambda m: re.sub(r"_within_\d+_moves", f"_within_{goal_moves}_moves", m.group(1)),
                             base_text, count=1)
    if fixed_shifts is not None:
        def fix(m):
            v = fixed_shifts[int(m.group(3))][m.group(2)]
            return f"{m.group(1)}{v} .. {v}{m.group(6)}"
        text = _SHIFT_DECL_RE.sub(fix, text)
    block = "\n".join(render_lemma(b, free_obs, g) for b in lemmas)
    begin, end = text.index(LEMMA_BEGIN) + len(LEMMA_BEGIN), text.index(LEMMA_END)
    return text[:begin] + "\n" + block + ("\n" if block else "") + text[end:]


def write_packages(packages, base, goal_moves, fixed_shifts, lemmas, free_obs):
    for p in packages:
        with open(smv_path(p), "w") as f:
            f.write(patch_model(base[p], goal_moves, fixed_shifts, lemmas, free_obs, geometry_factor(base[p])))


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


def race(packages, mc_script, label):
    t0 = time.time()
    print(f"[{label}]", end=" ", flush=True)
    win = mp.race_first_solution(packages=packages, mc_script=mc_script)
    print(f"[{label}] {time.time() - t0:.0f}s")
    if win is None and not all_proven(packages):
        sys.exit(f"[{label}] Aborted: not every package finished (interrupted or MC failure).")
    return win


def route_boxes(raw, smv_text, target_dirs):
    """World-coordinate boxes the model's obstacle check uses along the min-move route."""
    states = mp.route_states(raw.get, TARGET_SECTION, target_dirs)
    if states is None:
        sys.exit("Counterexample trace holds no route to the target (trace parsing problem?).")
    g = geometry_factor(smv_text)
    m = _K_RE.search(smv_text)
    k = int(m.group(1)) if m else 0

    def pt(sec, end):
        return raw[f"section_{sec}.{end}.x"], raw[f"section_{sec}.{end}.y"]

    boxes = set()
    for sec in {s for s, _ in states}:
        for end in ("source", "drain"):
            x, y = pt(sec, end)
            boxes.add((Fraction(x) / g, Fraction(x) / g, Fraction(y) / g, Fraction(y) / g))
    for (s1, d1), (s2, _) in zip(states, states[1:]):
        if s1 == s2 or k == 0:
            continue
        a, b = (s1, s2) if d1 == "F" else (s2, s1)
        (dx, dy), (sx, sy) = pt(a, "drain"), pt(b, "source")
        for seg in range(k):
            xs = (k * dx + seg * (sx - dx), k * dx + (seg + 1) * (sx - dx))
            ys = (k * dy + seg * (sy - dy), k * dy + (seg + 1) * (sy - dy))
            boxes.add(tuple(Fraction(v, k) / g for v in (min(xs), max(xs), min(ys), max(ys))))
    moves = 1 + sum(1 for (a, _), (b, _) in zip(states, states[1:]) if a == b)
    return sorted(boxes), moves, states


def world_obstacles(raw, g, n_obs):
    return [tuple(float(Fraction(raw[f"rect_obstacles_{f}_{o}"]) / g) for f in ("tl_x", "tl_y", "br_x", "br_y"))
            for o in range(n_obs)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=mp.TPL_PATH)
    ap.add_argument("--max-iters", type=int, default=30)
    ap.add_argument("--max-sections", type=int, default=6,
                    help="Refuse packages with more sections (their proofs get very slow).")
    ap.add_argument("--apply", action="store_true",
                    help="Write the result into RECT_OBSTACLES_*, fix all shifts and regenerate.")
    ap.add_argument("--no-regen", action="store_true", help="Reuse the already generated packages.")
    args = ap.parse_args()

    tpl_path = os.path.abspath(args.config)
    cfg, n_moves, obstacles, _ = load_config(tpl_path)
    prefix_dir = generated_prefix_dir(cfg)
    mc_script = f"@{{{vfm_script_path(tpl_path)}}}@.runMCJobs[16]"

    if not args.no_regen:
        regenerate(tpl_path, prefix_dir)
    packages = discover_packages(prefix_dir)
    if not packages:
        sys.exit(f"No packages under {os.path.dirname(prefix_dir)}; generation failed?")
    base = {p: read_text(smv_path(p)) for p in packages}
    too_big = [os.path.basename(p) for p in packages
               if int((_SECTIONS_RE.search(base[p]) or [0, 0])[1]) > args.max_sections]
    if too_big:
        sys.exit(f"Packages with more than {args.max_sections} sections: {too_big}. "
                 "Limit the @sections range in the config.")
    free_obs = free_obstacles(base[packages[0]])
    if not free_obs or "_within_" not in base[packages[0]]:
        sys.exit("Generated model lacks free obstacles or move-limited reachability; regenerate.")
    target_dirs = mp.target_park_in_dirs(smv_path(packages[0]))
    print(f"{len(packages)} package(s), free obstacles {sorted(free_obs)}, target: exactly {n_moves} move(s).")

    lemmas, shifts, certified = [], None, False
    for it in range(1, args.max_iters + 1):
        write_packages(packages, base, n_moves, None, lemmas, free_obs)
        win = race(packages, mc_script, f"iter {it}: synth <= {n_moves} moves")
        if win is None:
            print(f"No placement of the free obstacles forces exactly {n_moves} moves (within the model).")
            return 2
        raw = read_raw_trace(mp.package_trace_path(win))
        synth_win, synth_trace = win, read_text(mp.package_trace_path(win))
        shifts = {o: {a: raw[f"rect_obstacles_shift_k{a}_{o}"] for a in "xy"} for o in free_obs}
        print(f"  candidate from {os.path.basename(win)}: shift steps {shifts}")
        if n_moves == 1:
            certified = True
            break

        write_packages(packages, base, n_moves - 1, shifts, lemmas, free_obs)
        win = race(packages, mc_script, f"iter {it}: verify < {n_moves} moves")
        if win is None:
            certified = True
            break
        raw = read_raw_trace(mp.package_trace_path(win))
        boxes, moves, states = route_boxes(raw, read_text(smv_path(win)), target_dirs)
        print(f"  refuted by a {moves}-move route in {os.path.basename(win)}: {states}")
        lemmas.append(boxes)

    if not certified:
        print(f"Gave up after {args.max_iters} iterations ({len(lemmas)} lemmas).")
        return 3

    # Restore the synth solution (goal N, placement fixed) as the freshest CEX for mc_parker.py.
    write_packages(packages, base, n_moves, shifts, lemmas, free_obs)
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

    if args.apply:
        mp.patch_tpl_config(tpl_path, None, None, None, None, result)
        content = read_text(tpl_path)
        zeros = mp._format_paren_list(["0"] * len(result))
        for key in ("RECT_OBSTACLES_SHIFT_MIN_Xs", "RECT_OBSTACLES_SHIFT_MAX_Xs",
                    "RECT_OBSTACLES_SHIFT_MIN_Ys", "RECT_OBSTACLES_SHIFT_MAX_Ys"):
            content = mp._replace_tpl_string_value(content, key, zeros)
        with open(tpl_path, "w") as f:
            f.write(content)
        print(f"Applied to {os.path.relpath(tpl_path, mp.REPO_ROOT)} (all shifts reset to 0).")
        regenerate(tpl_path, prefix_dir)
        print("Re-run with: python3 parking/mc_parker.py --race")
    else:
        print(f"Solution in {os.path.basename(synth_win)}; open with: python3 parking/mc_parker.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
