#!/usr/bin/env python3
"""
Chunk 1 — G-code layer counter.
Reads a G-code file and reports layer change positions.
"""

import sys


def count_layers(filepath: str) -> None:
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        print(f"Error: cannot read file: {exc}")
        sys.exit(1)

    layer_change_lines = []  # 1-based line numbers where ;LAYER_CHANGE appears

    for lineno, line in enumerate(lines, start=1):
        if line.strip() == ";LAYER_CHANGE":
            layer_change_lines.append(lineno)

    total = len(layer_change_lines)

    if total == 0:
        print("No ;LAYER_CHANGE markers found in file.")
        print("Ensure this file was sliced with OrcaSlicer or ElegooSlicer.")
        sys.exit(0)

    print(f"File:         {filepath}")
    print(f"Total lines:  {len(lines)}")
    print(f"Total layers: {total}")

    # First and last 3 layer-change line numbers
    first_3 = layer_change_lines[:3]
    last_3  = layer_change_lines[-3:]

    print(f"\nFirst {min(3, total)} layer change(s) at line(s): {first_3}")
    print(f"Last  {min(3, total)} layer change(s) at line(s): {last_3}")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python gcode_golden_angle_chunk1.py <file.gcode>")
        sys.exit(1)

    count_layers(sys.argv[1])


if __name__ == "__main__":
    main()
