#!/usr/bin/env python3
"""Generate IQS915x per-axis coordinate correction LUTs from coord logs."""

from __future__ import annotations

import argparse
import bisect
import glob
import re
import statistics
from pathlib import Path


COORD_RE = re.compile(
    r"coord,t=(\d+),f=(\d+),info=0x([0-9a-fA-F]+),flags=0x([0-9a-fA-F]+),"
    r"x1=(\d+),y1=(\d+)"
)

TP_MOVEMENT = 0x10
Q15_SCALE = 32768


def parse_log(path: Path) -> list[tuple[float, int, int]]:
    rows: list[tuple[int, int, int, int]] = []

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = COORD_RE.search(line)
        if not match:
            continue

        timestamp, fingers, _info, flags, x, y = match.groups()
        if int(fingers) != 1:
            continue

        raw_x = int(x)
        raw_y = int(y)
        if raw_x == 65535 or raw_y == 65535:
            continue

        rows.append((int(timestamp), raw_x, raw_y, int(flags, 16)))

    if len(rows) < 2:
        return []

    moving = [i for i, row in enumerate(rows) if row[3] & TP_MOVEMENT]
    if moving:
        rows = rows[max(0, moving[0] - 1) : moving[-1] + 1]

    start_time = rows[0][0]
    end_time = rows[-1][0]
    if end_time <= start_time:
        return []

    return [
        ((timestamp - start_time) / (end_time - start_time), raw_x, raw_y)
        for timestamp, raw_x, raw_y, _flags in rows
    ]


def classify_axis(rows: list[tuple[float, int, int]]) -> str:
    xs = [row[1] for row in rows]
    ys = [row[2] for row in rows]
    return "x" if max(xs) - min(xs) >= max(ys) - min(ys) else "y"


def collect_axis_pairs(
    runs: list[list[tuple[float, int, int]]],
    axis: str,
    resolution: int,
    blocks: int,
) -> list[tuple[float, float]]:
    coord_index = 1 if axis == "x" else 2
    block_width = resolution / blocks
    half_block = block_width / 2
    pairs: list[tuple[float, float]] = []

    for rows in runs:
        forward = rows[-1][coord_index] >= rows[0][coord_index]

        for normalized_time, raw_x, raw_y in rows:
            raw = raw_x if axis == "x" else raw_y
            ideal = (normalized_time if forward else 1.0 - normalized_time) * resolution
            ideal = max(0.0, min(float(resolution), ideal))

            raw_block = min(blocks - 1, max(0, int(raw / block_width)))
            ideal_block = min(blocks - 1, max(0, int(ideal / block_width)))
            if raw_block != ideal_block:
                continue

            center = (ideal_block + 0.5) * block_width
            raw_delta = raw - center
            ideal_delta = ideal - center
            if raw_delta * ideal_delta < 0:
                continue

            raw_norm = abs(raw_delta) / half_block
            ideal_norm = abs(ideal_delta) / half_block
            if 0.0 <= raw_norm <= 1.0 and 0.0 <= ideal_norm <= 1.0:
                pairs.append((raw_norm, ideal_norm))

    return sorted(pairs)


def build_lut(pairs: list[tuple[float, float]], steps: int) -> list[int]:
    if not pairs:
        raise ValueError("no usable coordinate samples for LUT generation")

    raw_values = [pair[0] for pair in pairs]
    ideal_values = [pair[1] for pair in pairs]
    lut: list[float] = []

    for step in range(steps + 1):
        target = step / steps
        if step == 0:
            lut.append(0.0)
            continue
        if step == steps:
            lut.append(1.0)
            continue

        window = [ideal for raw, ideal in pairs if abs(raw - target) <= 0.06]
        if len(window) >= 3:
            value = statistics.median(window)
        else:
            idx = bisect.bisect_left(raw_values, target)
            if idx <= 0:
                value = ideal_values[0]
            elif idx >= len(raw_values):
                value = ideal_values[-1]
            else:
                raw_low = raw_values[idx - 1]
                raw_high = raw_values[idx]
                ideal_low = ideal_values[idx - 1]
                ideal_high = ideal_values[idx]
                if raw_high == raw_low:
                    value = ideal_low
                else:
                    value = ideal_low + (
                        (ideal_high - ideal_low) * (target - raw_low) / (raw_high - raw_low)
                    )

        lut.append(value)

    lut[0] = 0.0
    lut[-1] = 1.0
    previous = 0.0
    for idx in range(1, len(lut) - 1):
        lut[idx] = max(previous, min(1.0, lut[idx]))
        previous = lut[idx]

    return [round(value * Q15_SCALE) for value in lut]


def format_c_array(name: str, values: list[int]) -> str:
    lines = [f"static const uint16_t {name}[] = {{"]
    for start in range(0, len(values), 8):
        chunk = values[start : start + 8]
        suffix = "," if start + 8 < len(values) else ""
        lines.append("    " + ", ".join(f"{value:5d}" for value in chunk) + suffix)
    lines.append("};")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs", default="docs/logs/*.txt", help="glob for coord log .txt files")
    parser.add_argument("--steps", type=int, default=16, help="LUT steps per half block")
    parser.add_argument("--x-resolution", type=int, default=6000)
    parser.add_argument("--y-resolution", type=int, default=4000)
    parser.add_argument("--x-blocks", type=int, default=6)
    parser.add_argument("--y-blocks", type=int, default=4)
    args = parser.parse_args()

    classified: dict[str, list[list[tuple[float, int, int]]]] = {"x": [], "y": []}
    for log_path in sorted(glob.glob(args.logs)):
        rows = parse_log(Path(log_path))
        if not rows:
            continue
        axis = classify_axis(rows)
        classified[axis].append(rows)
        x_delta = max(row[1] for row in rows) - min(row[1] for row in rows)
        y_delta = max(row[2] for row in rows) - min(row[2] for row in rows)
        print(f"/* {log_path}: {axis}-axis samples={len(rows)} dx={x_delta} dy={y_delta} */")

    if len(classified["x"]) != 3 or len(classified["y"]) != 3:
        raise SystemExit(
            f"expected 3 X logs and 3 Y logs, got X={len(classified['x'])} "
            f"Y={len(classified['y'])}"
        )

    x_pairs = collect_axis_pairs(classified["x"], "x", args.x_resolution, args.x_blocks)
    y_pairs = collect_axis_pairs(classified["y"], "y", args.y_resolution, args.y_blocks)
    x_lut = build_lut(x_pairs, args.steps)
    y_lut = build_lut(y_pairs, args.steps)

    print(f"/* X pairs={len(x_pairs)} Y pairs={len(y_pairs)} steps={args.steps} */")
    print(format_c_array("iqs915x_coord_lut_x_q15", x_lut))
    print()
    print(format_c_array("iqs915x_coord_lut_y_q15", y_lut))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
