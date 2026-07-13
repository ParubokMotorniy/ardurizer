#!/usr/bin/env python3
"""Convert RGB-float + depth-float frame dump chunks into PNGs or GIFs."""

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


def depth_range(values: tuple[float, ...]) -> tuple[float, float]:
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return 0.0, 1.0
    return min(finite_values), max(finite_values)


def make_normalized_depth_pixels(
    values: tuple[float, ...],
    value_range: tuple[float, float],
) -> bytes:
    min_depth, max_depth = value_range
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


def make_frame_images(
    frame_data: bytes,
    width: int,
    height: int,
    color_buffer_size: int,
    depth_buffer_size: int,
    endian: str,
    depth_value_range: tuple[float, float] | None = None,
) -> tuple["Image.Image", "Image.Image"]:
    depth_offset = color_buffer_size
    color_data = frame_data[:depth_offset]
    depth_data = frame_data[depth_offset : depth_offset + depth_buffer_size]

    color_values = unpack_floats(color_data, endian)
    depth_values = unpack_floats(depth_data, endian)

    color_image = Image.frombytes(
        "RGB",
        (width, height),
        make_color_pixels(color_values, width, height),
    )
    depth_image = Image.frombytes(
        "L",
        (width, height),
        make_normalized_depth_pixels(
            depth_values,
            depth_value_range or depth_range(depth_values),
        ),
    )
    return color_image, depth_image


def split_frame_chunks(data: bytes, frame_size: int) -> tuple[list[bytes], int]:
    complete_size = len(data) - (len(data) % frame_size)
    frames = [
        data[offset : offset + frame_size]
        for offset in range(0, complete_size, frame_size)
    ]
    return frames, len(data) - complete_size


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a frame dump containing RGB float32 pixels followed by "
            "float32 depth values into color and grayscale depth images. "
            "One complete frame writes PNGs; multiple complete frames write GIFs."
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
    parser.add_argument(
        "--duration",
        type=int,
        default=120,
        help="GIF frame duration in milliseconds, default: 120",
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

    valid_frames, trailing_bytes = split_frame_chunks(data, expected_bytes)
    if not valid_frames:
        print(f"{args.dump_file} has no frame data", file=sys.stderr)
        return 1

    depth_offset = color_buffer_size
    output_prefix = args.output_prefix or args.dump_file.with_suffix("")

    print(f"depth offset: {depth_offset} bytes")
    print(f"frame size: {expected_bytes} bytes")
    if trailing_bytes:
        print(
            f"warning: ignoring {trailing_bytes} trailing byte(s) after "
            f"{len(valid_frames)} complete frame(s)",
            file=sys.stderr,
        )

    if len(valid_frames) == 1:
        color_path = output_prefix.with_name(output_prefix.name + "_color.png")
        depth_path = output_prefix.with_name(output_prefix.name + "_depth.png")
        color_image, depth_image = make_frame_images(
            valid_frames[0],
            args.width,
            args.height,
            color_buffer_size,
            depth_buffer_size,
            args.endian,
        )
        color_image.save(color_path)
        depth_image.save(depth_path)
        print("frames: 1")
        print(f"wrote: {color_path}")
        print(f"wrote: {depth_path}")
        return 0

    depth_values_by_frame = [
        unpack_floats(frame[depth_offset : depth_offset + depth_buffer_size], args.endian)
        for frame in valid_frames
    ]
    finite_depth_values = [
        value
        for frame_depth_values in depth_values_by_frame
        for value in frame_depth_values
        if math.isfinite(value)
    ]
    if finite_depth_values:
        global_depth_range = (min(finite_depth_values), max(finite_depth_values))
    else:
        global_depth_range = (0.0, 1.0)

    color_frames = []
    depth_frames = []
    for frame in valid_frames:
        color_image, depth_image = make_frame_images(
            frame,
            args.width,
            args.height,
            color_buffer_size,
            depth_buffer_size,
            args.endian,
            global_depth_range,
        )
        color_frames.append(color_image)
        depth_frames.append(depth_image)

    color_path = output_prefix.with_name(output_prefix.name + "_color.gif")
    depth_path = output_prefix.with_name(output_prefix.name + "_depth.gif")

    color_frames[0].save(
        color_path,
        save_all=True,
        append_images=color_frames[1:],
        duration=args.duration,
        loop=0,
        optimize=False,
    )
    depth_frames[0].save(
        depth_path,
        save_all=True,
        append_images=depth_frames[1:],
        duration=args.duration,
        loop=0,
        optimize=False,
    )
    print(f"frames: {len(valid_frames)}")
    print(f"wrote: {color_path}")
    print(f"wrote: {depth_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
