#!/usr/bin/env python3

"""Center-crop sim snapshot figures to a 4:3 region of height 2200."""

from __future__ import annotations

from pathlib import Path

from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

SOURCE_DIR = PACKAGE_DIR / "data" / "figures" / "stability_metrics" / "sim_snapshots"
OUTPUT_DIR = SOURCE_DIR / "cropped"

TARGET_HEIGHT = 2200
TARGET_WIDTH = round(TARGET_HEIGHT * 4 / 3)  # 2933

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
    for src_path in image_paths:
        with Image.open(src_path) as img:
            cropped = center_crop(img, TARGET_WIDTH, TARGET_HEIGHT)
            dst_path = OUTPUT_DIR / src_path.name
            cropped.save(dst_path)
        print(f"  {src_path.name} -> {dst_path.relative_to(SOURCE_DIR)}")


if __name__ == "__main__":
    main()
