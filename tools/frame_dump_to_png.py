#!/usr/bin/env python3
"""Convert marked RGB565 + depth-float frame dumps into PNGs or GIFs."""

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


def unpack_rgb565(data: bytes, endian: str) -> tuple[int, ...]:
    prefix = "<" if endian == "little" else ">"
    return struct.unpack(f"{prefix}{len(data) // 2}H", data)


def expand_5_to_8(value: int) -> int:
    return (value << 3) | (value >> 2)


def expand_6_to_8(value: int) -> int:
    return (value << 2) | (value >> 4)


def make_color_pixels(values: tuple[int, ...]) -> bytes:
    pixels = bytearray()
    for value in values:
        pixels.append(expand_5_to_8((value >> 11) & 0x1F))
        pixels.append(expand_6_to_8((value >> 5) & 0x3F))
        pixels.append(expand_5_to_8(value & 0x1F))
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

    color_values = unpack_rgb565(color_data, endian)
    depth_values = unpack_floats(depth_data, endian)

    color_image = Image.frombytes(
        "RGB",
        (width, height),
        make_color_pixels(color_values),
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


def split_marked_frames(
    data: bytes,
    separator: bytes,
    frame_size: int,
) -> tuple[list[bytes], int, int]:
    frames = []
    ignored_bytes = 0
    cursor = 0

    while cursor < len(data):
        marker_offset = data.find(separator, cursor)
        if marker_offset < 0:
            return frames, ignored_bytes, len(data) - cursor

        ignored_bytes += marker_offset - cursor
        payload_offset = marker_offset + len(separator)
        payload_end = payload_offset + frame_size
        if payload_end > len(data):
            return frames, ignored_bytes, len(data) - marker_offset

        frames.append(data[payload_offset:payload_end])
        cursor = payload_end

    return frames, ignored_bytes, 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a frame dump containing RGB565 pixels followed by "
            "float32 depth values into color and grayscale depth images. "
            "Each frame must start with the separator word. One complete frame "
            "writes PNGs; multiple complete frames write GIFs."
        )
    )
    parser.add_argument("width", type=int, help="frame width in pixels")
    parser.add_argument("height", type=int, help="frame height in pixels")
    parser.add_argument("dump_file", type=Path, help="binary frame dump file")
    parser.add_argument(
        "separator",
        help="ASCII/UTF-8 marker that appears immediately before every frame",
    )
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
        help="RGB565 and float byte order in the dump file, default: little",
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
    if not args.separator:
        print("separator must not be empty", file=sys.stderr)
        return 2

    pixel_count = args.width * args.height
    color_buffer_size = pixel_count * 2
    depth_buffer_size = pixel_count * 4
    expected_bytes = color_buffer_size + depth_buffer_size

    try:
        data = args.dump_file.read_bytes()
    except OSError as exc:
        print(f"failed to read {args.dump_file}: {exc}", file=sys.stderr)
        return 1

    separator = args.separator.encode()
    valid_frames, ignored_bytes, trailing_bytes = split_marked_frames(
        data,
        separator,
        expected_bytes,
    )
    if not valid_frames:
        print(
            f"{args.dump_file} has no complete frame starting with "
            f"{args.separator!r}",
            file=sys.stderr,
        )
        return 1

    depth_offset = color_buffer_size
    output_prefix = args.output_prefix or args.dump_file.with_suffix("")

    print(f"depth offset: {depth_offset} bytes")
    print(f"frame size: {expected_bytes} bytes")
    print(f"separator: {args.separator!r} ({len(separator)} bytes)")
    if ignored_bytes:
        print(
            f"warning: ignored {ignored_bytes} byte(s) before/between marked frames",
            file=sys.stderr,
        )
    if trailing_bytes:
        print(
            f"warning: ignored {trailing_bytes} trailing byte(s) without a "
            f"complete marked frame",
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
        optimize=False,
    )
    depth_frames[0].save(
        depth_path,
        save_all=True,
        append_images=depth_frames[1:],
        duration=args.duration,
        optimize=False,
    )
    print(f"frames: {len(valid_frames)}")
    print(f"wrote: {color_path}")
    print(f"wrote: {depth_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
