"""
make_run_mosaic.py  –  Combine all preview2 images from a run_i folder into one PNG.

Layout (full mosaic):
  - One row per iteration_j  (rows ordered by j ascending)
  - Images within each row ordered left-to-right by k (preview2_k.png)
  - All images are scaled to a common height; missing cells are left blank.

Layout (column mosaics, --column-mosaics):
  - One file per distinct k value: mosaic_k.png
  - Each file stacks iterations vertically, showing only the k-th image of that iteration.

Usage:
  python3 morty/make_run_mosaic.py <path/to/run_i> [--out output.png] [--row-height 120]
                                     [--exclude-image path/to/placeholder.png]
                                     [--column-mosaics] [--column-out-dir path/to/dir]
"""

import argparse
import hashlib
import os
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

# Brown background color found in preview2 images.
_BG_COLOR = np.array([101, 67, 33], dtype=np.uint8)
_BG_TOLERANCE = 10  # per-channel tolerance for the crop


def crop_background(img: Image.Image) -> Image.Image:
    """Crop away the uniform brown border from the image."""
    a = np.array(img.convert("RGB"))
    mask = np.any(np.abs(a.astype(int) - _BG_COLOR) > _BG_TOLERANCE, axis=2)
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any():
        return img  # entire image is background — leave as-is
    r0, r1 = int(rows.argmax()), int(len(rows) - rows[::-1].argmax())
    c0, c1 = int(cols.argmax()), int(len(cols) - cols[::-1].argmax())
    return img.crop((c0, r0, c1, r1))


_BLIND_PATTERN = re.compile(r"--\s*no counterexample found for invariant .+ up to .+", re.IGNORECASE)


def file_hash(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def is_blind_iteration(iteration_dir: Path) -> bool:
    """Return True if the last non-empty line of debug_trace_array.txt matches the blind pattern."""
    trace = iteration_dir / "debug_trace_array.txt"
    if not trace.is_file():
        return False
    last_line = ""
    with trace.open() as fh:
        for line in fh:
            stripped = line.rstrip()
            if stripped:
                last_line = stripped
    return bool(_BLIND_PATTERN.search(last_line))


# Sentinel: an iteration whose images should be replaced by a red placeholder.
_BLIND = object()


def collect_iterations(
    run_dir: Path, exclude_hash: str | None = None
) -> dict[int, list[tuple[int, Path]] | object]:
    """Return {iteration_index: [(k, path), ...]} sorted by k.
    Blind iterations map to the _BLIND sentinel instead of an image list."""
    result: dict[int, list[tuple[int, Path]] | object] = {}
    for item in run_dir.iterdir():
        m = re.fullmatch(r"iteration_(\d+)", item.name)
        if not m or not item.is_dir():
            continue
        j = int(m.group(1))
        if is_blind_iteration(item):
            result[j] = _BLIND
            continue
        preview_dir = item / "preview2"
        if not preview_dir.is_dir():
            continue
        images: list[tuple[int, Path]] = []
        for f in preview_dir.iterdir():
            fm = re.fullmatch(r"preview2_(\d+)\.png", f.name)
            if fm:
                if exclude_hash is None or file_hash(f) != exclude_hash:
                    images.append((int(fm.group(1)), f))
        if images:
            result[j] = sorted(images, key=lambda x: x[0])
    return result


def make_red_image(width: int, height: int) -> Image.Image:
    return Image.new("RGB", (width, height), (220, 30, 30))


def add_border(img: Image.Image, thickness: int = 1, color: tuple = (0, 0, 0)) -> Image.Image:
    """Return a new image with a solid border of `thickness` pixels around `img`."""
    w, h = img.size
    bordered = Image.new(img.mode, (w + 2 * thickness, h + 2 * thickness), color)
    bordered.paste(img, (thickness, thickness))
    return bordered


def build_column_mosaics(
    iterations: dict[int, list[tuple[int, Path]] | object],
    row_height: int,
    out_dir: Path,
) -> None:
    """For each distinct k, save a mosaic_k.png with one iteration per row."""
    # Build a mapping: k -> {j: path}  (blind iterations are excluded from the column map
    # but still contribute a red row to every column mosaic).
    col_map: dict[int, dict[int, Path]] = {}
    for j, imgs in iterations.items():
        if imgs is _BLIND:
            continue
        for k, path in imgs:  # type: ignore[union-attr]
            col_map.setdefault(k, {})[j] = path

    if not col_map:
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    sorted_js = sorted(iterations.keys())

    # Reference width: use the most common resized width across all real images.
    ref_width = 200  # fallback

    for k in sorted(col_map.keys()):
        rows: list[Image.Image] = []
        for j in sorted_js:
            if iterations[j] is _BLIND:
                rows.append(add_border(make_red_image(ref_width, row_height - 2)))
                continue
            if j not in col_map[k]:
                continue
            img = crop_background(Image.open(col_map[k][j]).convert("RGBA"))
            scale = (row_height - 2) / img.height
            new_w = max(1, round(img.width * scale))
            cell = add_border(img.resize((new_w, row_height - 2), Image.LANCZOS).convert("RGB"))
            ref_width = cell.width  # update running reference
            rows.append(cell)

        if not rows:
            continue

        canvas_w = max(img.width for img in rows)
        canvas_h = len(rows) * row_height
        canvas = Image.new("RGBA", (canvas_w, canvas_h), (255, 255, 255, 255))
        y = 0
        for img in rows:
            canvas.paste(img, (0, y))
            y += row_height

        dest = out_dir / f"mosaic_{k}.png"
        canvas.convert("RGB").save(dest)

    print(f"Saved {len(col_map)} column mosaic(s) to: {out_dir}")


def build_mosaic(
    iterations: dict[int, list[tuple[int, Path]] | object], row_height: int
) -> Image.Image:
    # Load & resize all images to a uniform height, keep aspect ratio.
    # Blind iterations get a single red placeholder image for the whole row.
    rows: list[list[Image.Image]] = []
    ref_width = 200  # fallback width for red rows before any real image is seen
    for j in sorted(iterations.keys()):
        if iterations[j] is _BLIND:
            rows.append([add_border(make_red_image(ref_width, row_height - 2))])
            continue
        row_imgs: list[Image.Image] = []
        for _k, path in iterations[j]:  # type: ignore[union-attr]
            img = crop_background(Image.open(path).convert("RGBA"))
            scale = (row_height - 2) / img.height
            new_w = max(1, round(img.width * scale))
            cell = add_border(img.resize((new_w, row_height - 2), Image.LANCZOS).convert("RGB"))
            ref_width = cell.width
            row_imgs.append(cell)
        rows.append(row_imgs)

    if not rows:
        raise ValueError("No images found.")

    # Canvas dimensions.
    row_widths = [sum(img.width for img in row) for row in rows]
    canvas_w = max(row_widths)
    canvas_h = len(rows) * row_height
    canvas = Image.new("RGBA", (canvas_w, canvas_h), (255, 255, 255, 255))

    # Paste rows.
    y = 0
    for row in rows:
        x = 0
        for img in row:
            canvas.paste(img, (x, y))
            x += img.width
        y += row_height

    return canvas.convert("RGB")

# Run like this: python3 morty/make_run_mosaic.py detailed_results/run_0 --exclude-image detailed_results/run_0/iteration_0/preview2/preview2_42.png --column-mosaics

def main() -> None:
    parser = argparse.ArgumentParser(description="Mosaic all preview2 images from a run_i folder.")
    parser.add_argument("run_dir", type=Path, help="Path to a run_i directory.")
    parser.add_argument("--out", type=Path, default=None,
                        help="Output PNG path. Defaults to <run_dir>/mosaic.png")
    parser.add_argument("--row-height", type=int, default=120,
                        help="Height in pixels for each iteration row. Default: 120")
    parser.add_argument("--exclude-image", type=Path, default=None,
                        help="Path to a PNG whose duplicates should be excluded from the mosaic.")
    parser.add_argument("--column-mosaics", action="store_true",
                        help="Also produce per-column mosaics (mosaic_k.png), one per distinct k.")
    parser.add_argument("--column-out-dir", type=Path, default=None,
                        help="Directory for column mosaics. Defaults to <run_dir>/column_mosaics/")
    args = parser.parse_args()

    run_dir: Path = args.run_dir.resolve()
    if not run_dir.is_dir():
        sys.exit(f"Error: '{run_dir}' is not a directory.")

    out_path: Path = args.out if args.out else run_dir / "mosaic.png"

    exclude_hash: str | None = None
    if args.exclude_image:
        exclude_path: Path = args.exclude_image.resolve()
        if not exclude_path.is_file():
            sys.exit(f"Error: --exclude-image '{exclude_path}' not found.")
        exclude_hash = file_hash(exclude_path)
        print(f"Excluding images matching: {exclude_path.name} (md5={exclude_hash})")

    iterations = collect_iterations(run_dir, exclude_hash)
    if not iterations:
        sys.exit(f"No iteration_j/preview2/preview2_k.png files found under '{run_dir}'.")

    print(f"Found {len(iterations)} iteration(s). Building mosaic …")
    if args.column_mosaics:
        col_dir = args.column_out_dir if args.column_out_dir else run_dir / "column_mosaics"
        build_column_mosaics(iterations, args.row_height, col_dir)
    else:
        mosaic = build_mosaic(iterations, args.row_height)
        mosaic.save(out_path)
        print(f"Saved: {out_path}  ({mosaic.width}x{mosaic.height} px)")


if __name__ == "__main__":
    main()
