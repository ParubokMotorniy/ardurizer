#!/usr/bin/env python3
"""Convert an RGB-float + depth-float frame dump into PNG images."""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    Image = None


def clamp_byte(value: float) -> int:
    if not math.isfinite(value):
        return 0
    value = max(0.0, min(1.0, value))
    return int(round(value * 255.0))


def unpack_floats(data: bytes, endian: str) -> tuple[float, ...]:
    prefix = "<" if endian == "little" else ">"
    return struct.unpack(f"{prefix}{len(data) // 4}f", data)


def make_color_pixels(values: tuple[float, ...], width: int, height: int) -> bytes:
    pixels = bytearray()
    cursor = 0
    for _ in range(width * height):
        pixels.append(clamp_byte(values[cursor]))
        pixels.append(clamp_byte(values[cursor + 1]))
        pixels.append(clamp_byte(values[cursor + 2]))
        cursor += 3
    return bytes(pixels)


def make_depth_pixels(values: tuple[float, ...]) -> bytes:
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        min_depth = 0.0
        max_depth = 1.0
    else:
        min_depth = min(finite_values)
        max_depth = max(finite_values)

    depth_span = max_depth - min_depth
    if depth_span <= 0.0:
        depth_span = 1.0

    pixels = bytearray()
    for value in values:
        if math.isfinite(value):
            normalized = (value - min_depth) / depth_span
            pixels.append(clamp_byte(normalized))
        else:
            pixels.append(0)
    return bytes(pixels)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a frame dump containing RGB float32 pixels followed by "
            "float32 depth values into color and grayscale depth PNGs."
        )
    )
    parser.add_argument("width", type=int, help="frame width in pixels")
    parser.add_argument("height", type=int, help="frame height in pixels")
    parser.add_argument("dump_file", type=Path, help="binary frame dump file")
    parser.add_argument(
        "-o",
        "--output-prefix",
        type=Path,
        help="output prefix; defaults to the dump file name without extension",
    )
    parser.add_argument(
        "--endian",
        choices=("little", "big"),
        default="little",
        help="float byte order in the dump file, default: little",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if Image is None:
        print("Pillow is required: python3 -m pip install Pillow", file=sys.stderr)
        return 1

    if args.width <= 0 or args.height <= 0:
        print("width and height must be positive", file=sys.stderr)
        return 2

    pixel_count = args.width * args.height
    color_buffer_size = pixel_count * 3 * 4
    depth_buffer_size = pixel_count * 4
    expected_bytes = color_buffer_size + depth_buffer_size

    try:
        data = args.dump_file.read_bytes()
    except OSError as exc:
        print(f"failed to read {args.dump_file}: {exc}", file=sys.stderr)
        return 1

    if len(data) < expected_bytes:
        print(
            f"{args.dump_file} is too small: expected at least {expected_bytes} bytes, "
            f"got {len(data)} bytes",
            file=sys.stderr,
        )
        return 1

    if len(data) > expected_bytes:
        print(
            f"warning: ignoring {len(data) - expected_bytes} trailing bytes",
            file=sys.stderr,
        )

    depth_offset = color_buffer_size
    color_data = data[:depth_offset]
    depth_data = data[depth_offset : depth_offset + depth_buffer_size]

    color_values = unpack_floats(color_data, args.endian)
    depth_values = unpack_floats(depth_data, args.endian)

    output_prefix = args.output_prefix or args.dump_file.with_suffix("")
    color_path = output_prefix.with_name(output_prefix.name + "_color.png")
    depth_path = output_prefix.with_name(output_prefix.name + "_depth.png")

    Image.frombytes(
        "RGB",
        (args.width, args.height),
        make_color_pixels(color_values, args.width, args.height),
    ).save(color_path)
    Image.frombytes(
        "L",
        (args.width, args.height),
        make_depth_pixels(depth_values),
    ).save(depth_path)

    print(f"depth offset: {depth_offset} bytes")
    print(f"wrote: {color_path}")
    print(f"wrote: {depth_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
