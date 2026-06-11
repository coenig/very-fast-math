"""
stitch_gifs.py  –  Stack cex-smooth-birdseye GIFs from multiple iterations vertically
                   into one animated GIF that plays all rows simultaneously.

The folder structure under each iteration_j has one sub-folder per config
(e.g. _config_vehwidth=8), and within that a numbered CEX folder ("0") containing
cex-smooth-birdseye/cex-smooth-birdseye.gif.

The config priority order is read from morty/envmodel_config.tpl.json (key
"UCD_CONFIG_PRIOS" inside "#TEMPLATE").  For each iteration, we walk that list and
pick the first config whose "0/" subfolder exists.  If no config has a CEX, the
iteration row is a red placeholder.

For each frame index f:
  - Row j shows frame f of iteration j's GIF (or its last frame if f exceeds that GIF's length).
  - Iterations whose GIF is missing are shown as a red placeholder row.

Usage:
  python3 morty/stitch_gifs.py <path/to/run_i> --up-to-iteration J \\
      [--out output.gif] [--row-height 200]
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

# Brown background color (same as make_run_mosaic.py).
_BG_COLOR = np.array([101, 67, 33], dtype=np.uint8)
_BG_TOLERANCE = 10


def crop_background(arr: np.ndarray) -> np.ndarray:
    """Crop the uniform brown border from an RGBA/RGB numpy array. Returns cropped array."""
    rgb = arr[:, :, :3]
    mask = np.any(np.abs(rgb.astype(int) - _BG_COLOR) > _BG_TOLERANCE, axis=2)
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any():
        return arr
    r0, r1 = int(rows.argmax()), int(len(rows) - rows[::-1].argmax())
    c0, c1 = int(cols.argmax()), int(len(cols) - cols[::-1].argmax())
    return arr[r0:r1, c0:c1]


def load_gif_frames(path: Path, row_height: int) -> list[np.ndarray]:
    """Return list of RGB numpy arrays (all scaled to row_height), one per frame."""
    img = Image.open(path)
    frames = []
    try:
        while True:
            frame = img.convert("RGB")
            arr = np.array(frame)
            arr = crop_background(arr)
            h, w = arr.shape[:2]
            scale = row_height / h
            new_w = max(1, round(w * scale))
            resized = np.array(
                Image.fromarray(arr).resize((new_w, row_height), Image.LANCZOS)
            )
            frames.append(resized)
            img.seek(img.tell() + 1)
    except EOFError:
        pass
    return frames


def add_border_arr(arr: np.ndarray, thickness: int = 1, color=(0, 0, 0)) -> np.ndarray:
    h, w = arr.shape[:2]
    out = np.full((h + 2 * thickness, w + 2 * thickness, 3), color, dtype=np.uint8)
    out[thickness:thickness + h, thickness:thickness + w] = arr
    return out


def make_red_row(width: int, height: int) -> np.ndarray:
    return np.full((height, width, 3), (220, 30, 30), dtype=np.uint8)


def parse_config_prios(raw: str) -> list[str]:
    """Parse a semicolon-separated config-prios string into an ordered list."""
    return [c.strip() for c in raw.split(";") if c.strip()]


_TPL_JSON = Path(__file__).resolve().parent / "envmodel_config.tpl.json"


def load_config_prios_from_template() -> list[str]:
    """Read UCD_CONFIG_PRIOS from the template JSON next to this script."""
    with _TPL_JSON.open() as fh:
        data = json.load(fh)
    raw = data["#TEMPLATE"]["UCD_CONFIG_PRIOS"]
    return parse_config_prios(raw)


def resolve_gif(iteration_dir: Path, config_prios: list[str]) -> Path | None:
    """Return the path to the cex-smooth-birdseye GIF from the first matching config, or None."""
    for cfg in config_prios:
        candidate = iteration_dir / cfg / "0" / "cex-smooth-birdseye" / "cex-smooth-birdseye.gif"
        if candidate.is_file():
            return candidate
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stitch cex-smooth-birdseye GIFs from iterations into one animated GIF."
    )
    parser.add_argument("run_dir", type=Path, help="Path to a run_i directory.")
    parser.add_argument(
        "--config-prios", type=str, default=None,
        help='Override config priority list (semicolon-separated). '
             'Default: read UCD_CONFIG_PRIOS from morty/envmodel_config.tpl.json',
    )
    parser.add_argument(
        "--up-to-iteration", type=int, required=True, metavar="J",
        help="Include iterations 0..J (inclusive).",
    )
    parser.add_argument(
        "--out", type=Path, default=None,
        help="Output GIF path. Defaults to <run_dir>/stitched.gif",
    )
    parser.add_argument(
        "--row-height", type=int, default=200,
        help="Height in pixels for each iteration row. Default: 200",
    )
    args = parser.parse_args()

    run_dir: Path = args.run_dir.resolve()
    if not run_dir.is_dir():
        sys.exit(f"Error: '{run_dir}' is not a directory.")

    if args.config_prios:
        config_prios = parse_config_prios(args.config_prios)
    else:
        config_prios = load_config_prios_from_template()
        print(f"Loaded UCD_CONFIG_PRIOS from {_TPL_JSON.name}")
    if not config_prios:
        sys.exit("Error: config priority list is empty.")
    print(f"Config priority order: {config_prios}")

    out_path: Path = args.out if args.out else run_dir / "stitched.gif"
    J: int = args.up_to_iteration
    row_height: int = args.row_height

    # Load frames for each iteration 0..J.
    all_frame_lists: list[list[np.ndarray] | None] = []
    ref_width = 400  # fallback; updated as real frames come in

    print(f"Loading GIFs for iterations 0 to {J} …")
    for j in range(J + 1):
        iter_dir = run_dir / f"iteration_{j}"
        gif_path = resolve_gif(iter_dir, config_prios) if iter_dir.is_dir() else None
        if gif_path is None:
            print(f"  iteration_{j}: GIF not found — red placeholder")
            all_frame_lists.append(None)
        else:
            print(f"  iteration_{j}: loading {gif_path.relative_to(run_dir)} …", end="\r")
            frames = load_gif_frames(gif_path, row_height - 2)
            if frames:
                ref_width = frames[0].shape[1]
            all_frame_lists.append(frames)
            print(f"  iteration_{j}: {len(frames)} frame(s), width={frames[0].shape[1] if frames else '?'}")

    # Determine total frame count (max across all GIFs).
    max_frames = max(
        (len(fl) for fl in all_frame_lists if fl is not None),
        default=1,
    )
    print(f"Total frames in output: {max_frames}")

    # Build composite canvas width (max row width after borders).
    # Use the max width across all frames (not just frame 0) because crop_background
    # can produce slightly different widths per frame due to rounding in the resize.
    canvas_w = max(
        (max(frame.shape[1] for frame in fl) + 2 for fl in all_frame_lists if fl is not None),
        default=ref_width + 2,
    )
    canvas_h = (J + 1) * row_height

    # Each GIF is right-aligned in time: shorter GIFs start later so they all end together.
    # start_offset[j] = number of leading frames where row j shows its first frame held still.
    start_offsets: list[int] = []
    for fl in all_frame_lists:
        n = len(fl) if fl is not None else 0
        start_offsets.append(max_frames - n if n > 0 else 0)

    # Compose frames.
    output_frames: list[Image.Image] = []
    for f in range(max_frames):
        canvas = np.full((canvas_h, canvas_w, 3), 255, dtype=np.uint8)
        for j, fl in enumerate(all_frame_lists):
            y = j * row_height
            if fl is None:
                cell = add_border_arr(make_red_row(canvas_w - 2, row_height - 2))
            else:
                # fi is the index into this GIF's frame list:
                # before the start offset, hold frame 0; after, advance normally.
                fi = max(0, f - start_offsets[j])
                cell = add_border_arr(fl[fi])
            h, w = cell.shape[:2]
            canvas[y: y + h, 0:w] = cell
        output_frames.append(Image.fromarray(canvas))

    print(f"Saving {len(output_frames)} frame(s) to {out_path} …")
    output_frames[0].save(
        out_path,
        save_all=True,
        append_images=output_frames[1:],
        loop=0,
        duration=20,  # match source GIFs (20 ms/frame)
        optimize=False,
    )
    print(f"Done: {out_path}  ({canvas_w}x{canvas_h} px, {len(output_frames)} frames)")


if __name__ == "__main__":
    main()
