#!/usr/bin/env python3
"""
Chunk 4 — Slicer-native orientation: template generation and verification.

Instead of rotating G-code coordinates (which pushes infill through the
walls of any non-round part), the schedule is handed to OrcaSlicer /
ElegooSlicer through the "Sparse infill rotation template" setting, so the
slicer itself generates infill at each layer's angle, clipped to the part.

  template  Print the comma-separated angle list to paste into
            Process > Strength > Sparse infill rotation template.
  verify    Check a G-code file sliced with that template: measure each
            layer's infill angle and compare it with the schedule.

Typical workflow:
  1. python gcode_golden_angle_chunk4.py template --layers 200
  2. Paste the output into the slicer, slice, export G-code.
  3. python gcode_golden_angle_chunk4.py verify part.gcode
  4. If verify reports a constant offset (the slicer's fill patterns add
     their own internal rotation), regenerate with --offset as instructed.

Read-only — the G-code is not modified.
"""

import argparse
import math
import sys

from gcode_golden_angle_chunk2 import split_layers
from gcode_golden_angle_chunk3 import (MATERIAL_PROFILES, angle_diff_180, collect_infill,
                                       compute_orientation, measure_angle, read_slicer_settings,
                                       resolve_config, signed_line_delta)

# Patterns that lay one set of parallel lines per layer, so a single
# orientation angle is meaningful (keys from s_keys_map_InfillPattern).
LINE_PATTERNS = {"monotonic", "monotonicline", "rectilinear", "alignedrectilinear", "zigzag", "line"}

DEFAULT_TOLERANCE = 1.0      # degrees; must be well under the micro-variation
MIN_COHERENCE = 0.5
REFINE_WINDOW = 10.0         # degrees; segments this close to the first estimate are kept
OFFSET_SPREAD_MAX = 1.0      # degrees; errors this consistent indicate a constant offset

VERIFY_SETTINGS = ("sparse_infill_pattern", "sparse_infill_rotate_template",
                   "infill_direction", "align_infill_direction_to_model", "raft_layers")


# =============================================================================
# TEMPLATE
# =============================================================================

def schedule_line_angles(layer_count, cfg, offset=0.0):
    """Template values for object layers 0..layer_count-1, in [0, 180).

    seed_value inside cfg is a TRADE SECRET; only the resulting angles leave
    this function.
    """
    seed, max_var = cfg["seed_value"], cfg["max_micro_variation"]
    return [(compute_orientation(i, seed, max_var)["final"] + offset) % 180.0
            for i in range(layer_count)]


def format_template(angles):
    # Plain non-negative decimals only: a '+' or '-' would switch OrcaSlicer's
    # parser into its rotation metalanguage instead of a per-layer list.
    return ",".join(f"{a:.2f}" for a in angles)


def parse_template(text):
    """Return the list of angles in a plain template, or None if it isn't one."""
    try:
        values = [float(v) for v in text.replace(" ", ",").split(",") if v]
    except ValueError:
        return None
    return values or None


def cmd_template(args):
    cfg = resolve_config(args)
    if args.from_gcode:
        try:
            with open(args.from_gcode, "r", encoding="utf-8", errors="replace") as fh:
                lines = fh.readlines()
        except OSError as exc:
            print(f"Error: cannot read file: {exc}")
            sys.exit(1)
        layer_count = len(split_layers(lines))
        raft = int(read_slicer_settings(lines, ("raft_layers",)).get("raft_layers", "0") or 0)
        layer_count -= raft
        if layer_count <= 0:
            print("Error: no object layers found in that G-code file.")
            sys.exit(1)
    else:
        layer_count = args.layers

    template = format_template(schedule_line_angles(layer_count, cfg, args.offset))
    if args.out:
        try:
            with open(args.out, "w", encoding="utf-8") as fh:
                fh.write(template + "\n")
        except OSError as exc:
            print(f"Error: cannot write {args.out}: {exc}")
            sys.exit(1)

    # Status goes to stderr so stdout can be piped or redirected cleanly.
    print(f"{layer_count} layers, material {cfg['material_profile'] or 'default'}, "
          f"max micro-variation {cfg['max_micro_variation']}°, offset {args.offset:+.2f}°",
          file=sys.stderr)
    print("Paste into Process > Strength > Sparse infill rotation template:", file=sys.stderr)
    if args.out:
        print(f"(also written to {args.out})", file=sys.stderr)
    print(template)


# =============================================================================
# VERIFY
# =============================================================================

def measure_refined(segments):
    """measure_angle, then re-measure using only segments near the first
    estimate, so connector moves between infill lines don't bias the result."""
    first, coherence = measure_angle(segments)
    if first is None:
        return None, 0.0
    near = []
    for seg in segments:
        _, x0, y0, x1, y1 = seg
        if math.hypot(x1 - x0, y1 - y0) < 1e-6:
            continue
        a = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0
        if angle_diff_180(a, first) <= REFINE_WINDOW:
            near.append(seg)
    refined, _ = measure_angle(near) if near else (first, coherence)
    return refined, coherence


def circular_mean_180(values):
    """Mean of line angles/offsets (period 180°) and their spread."""
    c = sum(math.cos(math.radians(2 * v)) for v in values)
    s = sum(math.sin(math.radians(2 * v)) for v in values)
    mean = math.degrees(math.atan2(s, c)) / 2.0
    spread = max(angle_diff_180(v, mean) for v in values)
    return mean, spread


def cmd_verify(args):
    cfg = resolve_config(args)
    try:
        with open(args.gcode, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        print(f"Error: cannot read file: {exc}")
        sys.exit(1)

    layers = split_layers(lines)
    if not layers:
        print("Error: no ;LAYER_CHANGE markers found — layer count is zero.")
        sys.exit(1)

    settings = read_slicer_settings(lines, VERIFY_SETTINGS)
    infill, warnings = collect_infill(lines, len(layers))
    raft = int(settings.get("raft_layers", "0") or 0)
    object_layers = len(layers) - raft
    expected = schedule_line_angles(object_layers, cfg)

    print(f"File:             {args.gcode}")
    print(f"Layers:           {len(layers)} ({raft} raft + {object_layers} object)")
    print(f"Material profile: {cfg['material_profile'] or 'none (config defaults)'}")
    print(f"Tolerance:        ±{args.tolerance}°")
    print("Seed value:       [TRADE SECRET — set in CONFIG block]")

    # --- Settings checks ---
    problems = []
    pattern = settings.get("sparse_infill_pattern")
    if pattern is not None:
        print(f"Infill pattern:   {pattern}")
        if pattern not in LINE_PATTERNS:
            problems.append(f"infill pattern '{pattern}' is not a single-direction line pattern; "
                            f"use one of: {', '.join(sorted(LINE_PATTERNS))}")
    if settings.get("align_infill_direction_to_model", "0") not in ("0", "false", ""):
        problems.append("align_infill_direction_to_model is on — it adds the model's rotation "
                        "to every infill angle; turn it off")
    if raft:
        warnings.append(f"{raft} raft layers: assuming the template is indexed from the first "
                        "object layer above the raft — check the results below")

    file_template = settings.get("sparse_infill_rotate_template")
    if file_template is None:
        print("Template in file: not found (no slicer config block in this file)")
    elif not file_template:
        problems.append("sparse_infill_rotate_template is empty — the file was not sliced "
                        "with the golden-angle template")
    else:
        values = parse_template(file_template)
        if values is None:
            problems.append("sparse_infill_rotate_template is not a plain angle list")
        else:
            if len(values) < object_layers:
                problems.append(f"template has {len(values)} angles but the part has "
                                f"{object_layers} layers — the slicer will repeat it from the start")
            used = [values[i % len(values)] for i in range(object_layers)]
            # The file's template may legitimately include a constant --offset.
            file_offset, file_spread = circular_mean_180(
                [signed_line_delta(u, e) for u, e in zip(used, expected)])
            file_offset = 0.0 if abs(file_offset) < 0.005 else file_offset
            if file_spread > 0.02:
                problems.append("template in the file does not match this schedule — "
                                "different seed, material profile or max variation?")
            else:
                print(f"Template in file: matches schedule ({len(values)} angles, "
                      f"offset {file_offset:+.2f}°)")

    # --- Per-layer measurement ---
    print(f"\n{'Layer':>5}  {'Obj':>4}  {'Target':>7}  {'Measured':>8}  {'Error':>7}  "
          f"{'Coher.':>6}  Status")
    print("-" * 56)
    errors = []      # (layer index, signed error)
    skipped = []
    for layer, inf in zip(layers, infill):
        obj = layer.index - raft
        if obj < 0:
            continue
        target = expected[obj]
        measured, coherence = measure_refined(inf.segments)
        if measured is None:
            skipped.append((layer.index, "no sparse infill"))
            print(f"{layer.index:>5}  {obj:>4}  {target:>6.2f}°  {'—':>8}  {'—':>7}  {'—':>6}  "
                  "skip: no sparse infill")
            continue
        if coherence < MIN_COHERENCE:
            skipped.append((layer.index, f"not parallel lines (coherence {coherence:.2f})"))
            print(f"{layer.index:>5}  {obj:>4}  {target:>6.2f}°  {measured:>7.2f}°  {'—':>7}  "
                  f"{coherence:>6.2f}  skip: not parallel lines")
            continue
        err = signed_line_delta(measured, target)
        err = 0.0 if abs(err) < 0.005 else err   # avoid printing "-0.00"
        errors.append((layer.index, err))
        status = "OK" if abs(err) <= args.tolerance else "OFF"
        print(f"{layer.index:>5}  {obj:>4}  {target:>6.2f}°  {measured:>7.2f}°  {err:>+6.2f}°  "
              f"{coherence:>6.2f}  {status}")

    # --- Summary ---
    print("\nSummary")
    checked = len(errors)
    passed = sum(1 for _, e in errors if abs(e) <= args.tolerance)
    print(f"  Layers checked:  {checked}")
    print(f"  Within ±{args.tolerance}°:    {passed}/{checked}")
    if errors:
        print(f"  Largest error:   {max(abs(e) for _, e in errors):.2f}°")

    if errors and passed < checked:
        offset, spread = circular_mean_180([e for _, e in errors])
        if spread <= OFFSET_SPREAD_MAX:
            print(f"  All layers are off by the same {offset:+.2f}° (±{spread:.2f}°): the slicer "
                  "adds a constant rotation to the template.")
            print(f"  Fix: regenerate the template with --offset {-offset:.2f} and slice again.")
        else:
            print("  Errors are not a constant offset — the template is probably not "
                  "applied to every layer, or layers are indexed differently.")

    for idx, reason in skipped:
        print(f"  Skipped layer {idx}: {reason}")
    for w in warnings:
        print(f"  Warning: {w}")
    for p in problems:
        print(f"  Problem: {p}")

    ok = checked > 0 and passed == checked and not problems
    if checked == 0:
        print("  Warning: no layers with measurable sparse infill — nothing to verify.")
    print(f"\nResult: {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 2)


# =============================================================================
# CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Chunk 4: generate and verify the slicer rotation template for the "
                    "golden-angle schedule.")
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--material", choices=sorted(MATERIAL_PROFILES),
                        help="material profile (sets max micro-variation)")
    common.add_argument("--max-variation", type=float,
                        help="max micro-variation in degrees (overrides material profile)")
    common.add_argument("--seed", type=int,
                        help="override the seed (TRADE SECRET — prefer the CONFIG block)")
    sub = parser.add_subparsers(dest="command")

    t = sub.add_parser("template", parents=[common], help="print the rotation template")
    src = t.add_mutually_exclusive_group(required=True)
    src.add_argument("--layers", type=int, help="number of object layers")
    src.add_argument("--from-gcode", metavar="FILE",
                     help="take the layer count from an already-sliced G-code file")
    t.add_argument("--offset", type=float, default=0.0,
                   help="constant correction in degrees, as reported by verify")
    t.add_argument("--out", metavar="FILE", help="also write the template to this file")

    v = sub.add_parser("verify", parents=[common], help="check a sliced G-code file")
    v.add_argument("gcode", help="G-code file sliced with the template")
    v.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                   help=f"allowed angle error in degrees (default {DEFAULT_TOLERANCE})")

    args = parser.parse_args()
    if args.command == "template":
        if args.layers is not None and args.layers <= 0:
            parser.error("--layers must be positive")
        cmd_template(args)
    elif args.command == "verify":
        cmd_verify(args)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
