#!/usr/bin/env python3

"""Center-crop figures in geo_only_sim to a 16:9 region of height 1800."""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

SOURCE_DIR = PACKAGE_DIR / "data" / "figures" / "geo_only_sim"
OUTPUT_DIR = SOURCE_DIR / "cropped"
COMPOSITE_PATH = OUTPUT_DIR / "composite_2x4.png"

TARGET_HEIGHT = 2200
TARGET_WIDTH = TARGET_HEIGHT * 16 // 9  # 3200

COMPOSITE_ROWS = 2
COMPOSITE_COLS = 4

LABEL_FONT_PATH = "/usr/share/fonts/truetype/msttcorefonts/timesbd.ttf"
LABEL_CIRCLE_DIAMETER = 240
LABEL_MARGIN = 70
LABEL_FONT_SIZE = 170

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".webp"}


def center_crop(image: Image.Image, width: int, height: int) -> Image.Image:
    src_w, src_h = image.size
    if width > src_w or height > src_h:
        raise ValueError(
            f"Crop size {width}x{height} exceeds image size {src_w}x{src_h}"
        )
    left = (src_w - width) // 2
    top = (src_h - height) // 2
    return image.crop((left, top, left + width, top + height))


def draw_circled_number(tile: Image.Image, number: int) -> None:
    draw = ImageDraw.Draw(tile)
    font = ImageFont.truetype(LABEL_FONT_PATH, LABEL_FONT_SIZE)

    x0, y0 = LABEL_MARGIN, LABEL_MARGIN
    x1, y1 = x0 + LABEL_CIRCLE_DIAMETER, y0 + LABEL_CIRCLE_DIAMETER
    draw.ellipse((x0, y0, x1, y1), fill="black")

    text = str(number)
    bbox = draw.textbbox((0, 0), text, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    text_x = x0 + (LABEL_CIRCLE_DIAMETER - text_w) // 2 - bbox[0]
    text_y = y0 + (LABEL_CIRCLE_DIAMETER - text_h) // 2 - bbox[1]
    draw.text((text_x, text_y), text, font=font, fill="white")


def build_composite(
    cropped_images: list[Image.Image], rows: int, cols: int
) -> Image.Image:
    tile_w, tile_h = cropped_images[0].size
    composite = Image.new("RGB", (tile_w * cols, tile_h * rows), color="white")
    for idx, tile in enumerate(cropped_images[: rows * cols]):
        draw_circled_number(tile, idx + 1)
        row, col = divmod(idx, cols)
        composite.paste(tile, (col * tile_w, row * tile_h))
    return composite


def main() -> None:
    if not SOURCE_DIR.is_dir():
        raise SystemExit(f"Source directory not found: {SOURCE_DIR}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    image_paths = sorted(
        p for p in SOURCE_DIR.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
    )

    if not image_paths:
        print(f"No images found in {SOURCE_DIR}")
        return

    print(f"Cropping {len(image_paths)} image(s) to {TARGET_WIDTH}x{TARGET_HEIGHT}")
    cropped_tiles: list[Image.Image] = []
    for src_path in image_paths:
        with Image.open(src_path) as img:
            cropped = center_crop(img, TARGET_WIDTH, TARGET_HEIGHT)
            dst_path = OUTPUT_DIR / src_path.name
            cropped.save(dst_path)
            cropped_tiles.append(cropped.convert("RGB"))
        print(f"  {src_path.name} -> {dst_path.relative_to(SOURCE_DIR)}")

    expected = COMPOSITE_ROWS * COMPOSITE_COLS
    if len(cropped_tiles) < expected:
        print(
            f"Skipping composite: need {expected} images, got {len(cropped_tiles)}"
        )
        return
    if len(cropped_tiles) > expected:
        print(
            f"Using first {expected} of {len(cropped_tiles)} images for composite"
        )

    composite = build_composite(cropped_tiles, COMPOSITE_ROWS, COMPOSITE_COLS)
    composite.save(COMPOSITE_PATH)
    print(
        f"Composite {COMPOSITE_ROWS}x{COMPOSITE_COLS} "
        f"({composite.size[0]}x{composite.size[1]}) -> "
        f"{COMPOSITE_PATH.relative_to(SOURCE_DIR)}"
    )


if __name__ == "__main__":
    main()
