#!/usr/bin/env python3
"""
Chunk 2 — Per-layer blocks and sparse infill detection.

Builds on Chunk 1: splits the G-code into layer blocks at ;LAYER_CHANGE,
then walks each layer tracking ;TYPE: sections to find sparse infill
extrusion moves. Read-only — the G-code is not modified.
"""

import sys

# ;TYPE: values treated as sparse infill (exact match, case-insensitive).
# OrcaSlicer/ElegooSlicer emit "Sparse infill"; "Internal infill" is the
# older PrusaSlicer-style name. "Internal solid infill" is deliberately
# excluded — it is solid top/bottom fill, not sparse infill.
SPARSE_INFILL_TYPES = {"sparse infill", "internal infill"}


class Layer:
    def __init__(self, index, start_line):
        self.index = index
        self.start_line = start_line   # 1-based line of ;LAYER_CHANGE
        self.end_line = start_line     # 1-based last line of this layer
        self.z = None                  # from ;Z: comment, if present
        self.infill_moves = 0          # sparse infill extrusion moves
        self.infill_sections = 0       # separate ;TYPE:Sparse infill blocks
        self.types_seen = []           # ;TYPE: values in order of appearance


def parse_words(line):
    """Return {letter: float} for a G0/G1 line, ignoring any ; comment."""
    code = line.split(";", 1)[0].split()
    words = {}
    for token in code[1:]:
        try:
            words[token[0].upper()] = float(token[1:])
        except (ValueError, IndexError):
            pass
    return words


def split_layers(lines):
    """Split lines into Layer objects at each ;LAYER_CHANGE marker."""
    layers = []
    for lineno, raw in enumerate(lines, start=1):
        line = raw.strip()
        if line == ";LAYER_CHANGE":
            if layers:
                layers[-1].end_line = lineno - 1
            layers.append(Layer(len(layers), lineno))
        elif layers and line.startswith(";Z:") and layers[-1].z is None:
            try:
                layers[-1].z = float(line[3:])
            except ValueError:
                pass
    if layers:
        layers[-1].end_line = len(lines)
    return layers


def scan_layer(layer, lines, relative_e):
    """Count sparse infill sections and extrusion moves within one layer.

    relative_e is the extrusion mode in effect at the start of the layer
    (M83 = relative, M82 = absolute). Returns the mode at the end of the
    layer so the caller can carry it into the next one.
    """
    in_infill = False
    last_e = 0.0  # only meaningful in absolute mode

    for raw in lines[layer.start_line - 1:layer.end_line]:
        line = raw.strip()

        if line.startswith(";TYPE:"):
            type_name = line[6:].strip()
            layer.types_seen.append(type_name)
            in_infill = type_name.lower() in SPARSE_INFILL_TYPES
            if in_infill:
                layer.infill_sections += 1
            continue

        cmd = line.split(";", 1)[0].strip().upper()
        if cmd == "M83":
            relative_e = True
            continue
        if cmd == "M82":
            relative_e = False
            continue
        if cmd.startswith("G92"):
            words = parse_words(line)
            if "E" in words:
                last_e = words["E"]
            continue
        if not (cmd.startswith("G1 ") or cmd.startswith("G0 ")):
            continue

        words = parse_words(line)
        if "E" not in words:
            continue

        # A move extrudes if E is positive (relative) or increasing (absolute)
        if relative_e:
            extrudes = words["E"] > 0
        else:
            extrudes = words["E"] > last_e
            last_e = words["E"]

        if in_infill and extrudes and ("X" in words or "Y" in words):
            layer.infill_moves += 1

    return relative_e


def analyze(filepath):
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        print(f"Error: cannot read file: {exc}")
        sys.exit(1)

    layers = split_layers(lines)
    if not layers:
        print("Error: no ;LAYER_CHANGE markers found — layer count is zero.")
        print("Ensure this file was sliced with OrcaSlicer or ElegooSlicer.")
        sys.exit(1)

    # Extrusion mode set in the start G-code (before layer 0) carries over.
    # OrcaSlicer defaults to relative E, so assume that unless told otherwise.
    relative_e = True
    for raw in lines[:layers[0].start_line - 1]:
        cmd = raw.split(";", 1)[0].strip().upper()
        if cmd == "M83":
            relative_e = True
        elif cmd == "M82":
            relative_e = False

    for layer in layers:
        relative_e = scan_layer(layer, lines, relative_e)

    print(f"File:         {filepath}")
    print(f"Total lines:  {len(lines)}")
    print(f"Total layers: {len(layers)}")
    print()
    print(f"{'Layer':>5}  {'Z':>7}  {'Lines':>15}  {'Infill sect.':>12}  {'Infill moves':>12}")
    print("-" * 58)
    for layer in layers:
        z = f"{layer.z:.3f}" if layer.z is not None else "?"
        span = f"{layer.start_line}-{layer.end_line}"
        print(f"{layer.index:>5}  {z:>7}  {span:>15}  "
              f"{layer.infill_sections:>12}  {layer.infill_moves:>12}")

    with_infill = [l for l in layers if l.infill_moves > 0]
    without = [l for l in layers if l.infill_moves == 0]
    total_moves = sum(l.infill_moves for l in layers)

    print("-" * 58)
    print(f"Layers with sparse infill:    {len(with_infill)}")
    print(f"Layers without sparse infill: {len(without)}")
    print(f"Total sparse infill moves:    {total_moves}")

    if without:
        print("\nLayers with no sparse infill (these would be skipped):")
        for layer in without:
            types = ", ".join(dict.fromkeys(layer.types_seen)) or "no ;TYPE: comments"
            print(f"  Layer {layer.index}: {types}")

    if total_moves == 0:
        print("\nWarning: no sparse infill detected anywhere in this file.")
        print("Check that the slicer writes ;TYPE: comments and that sparse infill is enabled.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python gcode_golden_angle_chunk2.py <file.gcode>")
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
