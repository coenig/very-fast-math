#!/usr/bin/env python3
"""Summarize all-to-all future_point matches across iterations.

For each iteration t (starting at 1), files in iteration_t/<config> are compared
against all files in iteration_(t-1)/<config>. By default, lines containing
".v" are ignored before comparison.

Supported output modes per point in iteration t:
- min_or_x: output smallest matching previous counter, or X if no match.
- set_or_x: output {a,b,c} with all matching previous counters, or X.
- unique_or_x: output i only for unique match, else X.

Optional post-processing can compress strictly increasing numeric runs, e.g.:
  ..., 4,5,6,7, ...  ->  ..., [4-7], ...
Only runs of length >= 2 are compressed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path

# Default: drop "velocity" lines like `abs(env.veh___609___.v - (28)) <= 2`.
# We match `.v` as a whole token (\.v\b) so it does NOT also match `.veh`
# (which appears in every abs_pos / on_normalized_lane line).
DEFAULT_IGNORE_REGEX = r"\.v\b"


ITER_RE = re.compile(r"^iteration_(\d+)$")
POINT_RE = re.compile(r"^future_point_(\d+)\.txt$")


@dataclass(frozen=True)
class IterationData:
    index: int
    path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-dir",
        default="examples/exp/detailed_archive/run_1",
        help="Path containing iteration_<n> directories.",
    )
    parser.add_argument(
        "--config",
        default="_config_vehwidth=9",
        help="Config folder name inside each iteration directory.",
    )
    parser.add_argument(
        "--all-configs",
        action="store_true",
        help=(
            "Summarize every config folder found across the iterations instead of "
            "just --config. Each config is printed under a '=== <config> ===' header."
        ),
    )
    parser.add_argument(
        "--ignore-regex",
        action="append",
        default=[DEFAULT_IGNORE_REGEX],
        help=(
            "Drop lines matching this regex before comparing files. Repeatable. "
            r"Default '\.v\b' removes velocity lines without touching '.veh' lines."
        ),
    )
    parser.add_argument(
        "--mode",
        choices=["min_or_x", "set_or_x", "unique_or_x"],
        default="min_or_x",
        help="How to represent matches for each current point.",
    )
    parser.add_argument(
        "--no-match-token",
        default="X",
        help="Token for points with no match (or non-unique in unique_or_x mode).",
    )
    parser.add_argument(
        "--compress-increasing",
        action="store_true",
        help="Compress strictly increasing numeric runs i..j into [i-j] (length >= 2).",
    )
    parser.add_argument(
        "--compress-use-any-match",
        action="store_true",
        help=(
            "When compressing, allow selecting any value from each match-set so "
            "ranges can stay connected. Non-compressed numeric tokens use the "
            "smallest available match."
        ),
    )
    parser.add_argument(
        "--out",
        default="",
        help="Optional output file. If omitted, prints to stdout.",
    )
    return parser.parse_args()


def list_iterations(base_dir: Path) -> list[IterationData]:
    iterations: list[IterationData] = []
    for child in base_dir.iterdir():
        if not child.is_dir():
            continue
        match = ITER_RE.match(child.name)
        if match:
            iterations.append(IterationData(index=int(match.group(1)), path=child))
    return sorted(iterations, key=lambda x: x.index)


def list_future_points(config_dir: Path) -> list[tuple[int, Path]]:
    points: list[tuple[int, Path]] = []
    if not config_dir.is_dir():
        return points
    for child in config_dir.iterdir():
        if not child.is_file():
            continue
        match = POINT_RE.match(child.name)
        if match:
            points.append((int(match.group(1)), child))
    return sorted(points, key=lambda x: x[0])


def discover_configs(iterations: list[IterationData]) -> list[str]:
    """Return the sorted union of config folder names across all iterations."""
    configs: set[str] = set()
    for it in iterations:
        if not it.path.is_dir():
            continue
        for child in it.path.iterdir():
            if child.is_dir():
                configs.add(child.name)
    return sorted(configs)


def load_config_priorities(base_dir: Path) -> list[str]:
    """Read the priority-ordered config names from UCD_CONFIG_PRIOS.

    The list lives in each pristine config's template JSON at
    ``0_pristine/<any_config>/templates_archive/envmodel_config.tpl.json`` under
    ``["#TEMPLATE"]["UCD_CONFIG_PRIOS"]`` as a ';'-separated string. Returns the
    ordered names, or [] when the file/entry cannot be found or parsed.
    """
    pristine = base_dir / "0_pristine"
    if not pristine.is_dir():
        return []
    for config_dir in sorted(pristine.iterdir()):
        tpl = config_dir / "templates_archive" / "envmodel_config.tpl.json"
        if not tpl.is_file():
            continue
        try:
            data = json.loads(tpl.read_text(encoding="utf-8", errors="replace"))
        except (json.JSONDecodeError, OSError):
            continue
        prios = data.get("#TEMPLATE", {}).get("UCD_CONFIG_PRIOS", "")
        if isinstance(prios, str) and prios.strip():
            return [c for c in (p.strip() for p in prios.split(";")) if c]
    return []


def load_selected_configs(iterations: list[IterationData]) -> dict[int, str]:
    """Read the per-iteration selected config from ``selected_config.txt``.

    morty.py writes this marker into each archived ``iteration_<n>`` folder
    (the selection cannot be reliably inferred from the raw files). Returns
    {iteration_index: config_name}; iterations without a marker are omitted.
    """
    selected: dict[int, str] = {}
    for it in iterations:
        marker = it.path / "selected_config.txt"
        if not marker.is_file():
            continue
        try:
            name = marker.read_text(encoding="utf-8", errors="replace").strip()
        except OSError:
            continue
        if name:
            selected[it.index] = name
    return selected


def normalized_hash(path: Path, ignore_patterns: list[re.Pattern[str]]) -> str:
    hasher = hashlib.sha256()
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if any(pattern.search(line) for pattern in ignore_patterns):
                continue
            hasher.update(line.encode("utf-8", errors="replace"))
    return hasher.hexdigest()


def token_for_matches(
    matches: list[int],
    mode: str,
    no_match_token: str,
) -> str:
    if not matches:
        return no_match_token

    if mode == "min_or_x":
        return str(min(matches))

    if mode == "unique_or_x":
        return str(matches[0]) if len(matches) == 1 else no_match_token

    # mode == "set_or_x"
    return "{" + ",".join(str(x) for x in matches) + "}"


def compress_increasing_tokens(tokens: list[str]) -> list[str]:
    def flush_run(out: list[str], run: list[int]) -> None:
        if not run:
            return
        if len(run) >= 2:
            out.append(f"[{run[0]}-{run[-1]}]")
        else:
            out.extend(str(x) for x in run)

    out: list[str] = []
    run: list[int] = []

    for token in tokens:
        if token.isdigit():
            number = int(token)
            if not run or number == run[-1] + 1:
                run.append(number)
            else:
                flush_run(out, run)
                run = [number]
        else:
            flush_run(out, run)
            run = []
            out.append(token)

    flush_run(out, run)
    return out


def compress_with_any_match(
    candidate_tokens: list[list[int]],
    no_match_token: str,
) -> list[str]:
    """Compress using candidate sets, not just fixed chosen numbers.

    For each position, candidate_tokens[pos] contains all numeric matches.
    The algorithm greedily takes the longest consecutive run starting at each
    position where values can be chosen as v, v+1, ..., v+n across positions.
    Runs of length >= 2 become [v-(v+n)]. Shorter runs emit explicit numbers,
    choosing the smallest candidate at each position.
    """

    out: list[str] = []
    i = 0
    n = len(candidate_tokens)

    while i < n:
        choices = candidate_tokens[i]
        if not choices:
            out.append(no_match_token)
            i += 1
            continue

        # end_value -> start_value for runs ending at current position
        state: dict[int, int] = {v: v for v in sorted(choices)}
        best_len = 1
        best_end_pos = i
        best_state = state.copy()

        j = i
        while j + 1 < n and candidate_tokens[j + 1]:
            next_choices = candidate_tokens[j + 1]
            next_state: dict[int, int] = {}
            for v in sorted(next_choices):
                prev = v - 1
                if prev in state:
                    next_state[v] = state[prev]
            if not next_state:
                break
            j += 1
            state = next_state
            run_len = j - i + 1
            if run_len > best_len:
                best_len = run_len
                best_end_pos = j
                best_state = state.copy()

        if best_len >= 2:
            end_value = min(best_state.keys())
            start_value = best_state[end_value]
            out.append(f"[{start_value}-{end_value}]")
            i = best_end_pos + 1
        else:
            out.append(str(min(choices)))
            i += 1

    return out


def build_iteration_line(
    iteration: int,
    curr_points: list[tuple[int, Path]],
    prev_hash_map: dict[str, list[int]],
    mode: str,
    no_match_token: str,
    ignore_patterns: list[re.Pattern[str]],
    compress_increasing: bool,
    compress_use_any_match: bool,
) -> str:
    tokens: list[str] = []
    candidate_tokens: list[list[int]] = []
    for _, curr_path in curr_points:
        curr_hash = normalized_hash(curr_path, ignore_patterns)
        matches = prev_hash_map.get(curr_hash, [])
        candidate_tokens.append(matches)
        tokens.append(token_for_matches(matches, mode, no_match_token))

    if compress_increasing:
        if compress_use_any_match:
            tokens = compress_with_any_match(candidate_tokens, no_match_token)
        else:
            tokens = compress_increasing_tokens(tokens)

    return f"{iteration}: " + ",".join(tokens)


def summarize_config(
    config: str,
    iterations: list[IterationData],
    iteration_by_index: dict[int, IterationData],
    args: argparse.Namespace,
    ignore_patterns: list[re.Pattern[str]],
    selected_by_iter: dict[int, str],
    mark_selected: bool,
) -> list[str]:
    lines: list[str] = []

    def prefix_for(index: int) -> str:
        # Mark the iterations where THIS config was the selected one with '* '.
        if not mark_selected:
            return ""
        return "* " if selected_by_iter.get(index) == config else "  "

    for it in iterations:
        prev = iteration_by_index.get(it.index - 1)
        curr_config = it.path / config
        prev_config = prev.path / config if prev is not None else None

        if not curr_config.is_dir():
            lines.append(f"{prefix_for(it.index)}{it.index}:")
            continue

        curr_points = list_future_points(curr_config)

        # Build the match map from the previous iteration. Iteration 0 (and any
        # iteration whose predecessor is missing) is compared against an empty
        # "dummy" iteration, so every point ends up as a no-match token (X).
        prev_hash_map: dict[str, list[int]] = {}
        if prev_config is not None and prev_config.is_dir():
            prev_points = list_future_points(prev_config)
            for prev_counter, prev_path in prev_points:
                prev_hash = normalized_hash(prev_path, ignore_patterns)
                prev_hash_map.setdefault(prev_hash, []).append(prev_counter)

            for counters in prev_hash_map.values():
                counters.sort()

        line = build_iteration_line(
            iteration=it.index,
            curr_points=curr_points,
            prev_hash_map=prev_hash_map,
            mode=args.mode,
            no_match_token=args.no_match_token,
            ignore_patterns=ignore_patterns,
            compress_increasing=args.compress_increasing,
            compress_use_any_match=args.compress_use_any_match,
        )
        lines.append(prefix_for(it.index) + line)

    return lines


def summarize(args: argparse.Namespace) -> list[str]:
    base_dir = Path(args.base_dir)
    if not base_dir.is_dir():
        raise FileNotFoundError(f"Base dir not found: {base_dir}")

    iterations = list_iterations(base_dir)
    if not iterations:
        return []

    ignore_patterns = [re.compile(p) for p in args.ignore_regex]
    iteration_by_index = {it.index: it for it in iterations}

    # Per-iteration selected config (from selected_config.txt markers, if present).
    selected_by_iter = load_selected_configs(iterations)
    mark_selected = bool(selected_by_iter)

    if args.all_configs:
        configs = discover_configs(iterations)
        if not configs:
            return []
        # Order configs by the UCD_CONFIG_PRIOS priority list when available;
        # any discovered config not listed there is appended (sorted) afterwards.
        prio = load_config_priorities(base_dir)
        if prio:
            discovered = set(configs)
            ordered = [c for c in prio if c in discovered]
            remaining = sorted(discovered.difference(ordered))
            configs = ordered + remaining
    else:
        configs = [args.config]

    output_lines: list[str] = []
    for idx, config in enumerate(configs):
        if len(configs) > 1:
            if idx > 0:
                output_lines.append("")
            output_lines.append(f"=== {config} ===")
        output_lines.extend(
            summarize_config(
                config,
                iterations,
                iteration_by_index,
                args,
                ignore_patterns,
                selected_by_iter,
                mark_selected,
            )
        )

    return output_lines


def main() -> None:
    args = parse_args()
    lines = summarize(args)

    output = "\n".join(lines)
    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(output + ("\n" if output else ""), encoding="utf-8")
    else:
        print(output)


if __name__ == "__main__":
    main()
