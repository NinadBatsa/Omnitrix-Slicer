#!/usr/bin/env python3
"""
Chunk 3 — Orientation schedule and current infill angle measurement.

Builds on Chunk 2: for every layer it
  1. computes the target orientation from the four-component schedule
     (warp-weft + golden angle + seed-based micro-variation),
  2. extracts the sparse infill segments (start/end points) and measures
     the infill angle the slicer actually produced,
  3. reports the rotation each layer would need.
Read-only — the G-code is not modified.

Angles are measured counter-clockwise from the printer's +X axis.
Infill lines have no direction, so 10° and 190° are the same line set;
"line angle" below means the orientation folded into [0, 180).
"""

import argparse
import math
import sys

from gcode_golden_angle_chunk2 import SPARSE_INFILL_TYPES, parse_words, split_layers

# =============================================================================
# CONFIGURATION
# =============================================================================
# *** TRADE SECRET ***
# seed_value drives the micro-variation component of the schedule. It is a
# trade secret: keep it here rather than passing --seed on the command line,
# because OrcaSlicer copies the whole post-processing command into every
# G-code file it exports (and into saved .3mf projects).
CONFIG = {
    "seed_value": 42,               # *** TRADE SECRET — do not disclose ***
    "max_micro_variation": 3.0,     # degrees; overridden by material_profile
    "material_profile": None,       # None, or 'pla' | 'petg' | 'tpu' | 'abs'
}
# *** END TRADE SECRET ***

MATERIAL_PROFILES = {
    "pla":  {"max_micro_variation": 2.0, "overlap_percentage": 18.0},
    "petg": {"max_micro_variation": 3.0, "overlap_percentage": 15.0},
    "tpu":  {"max_micro_variation": 4.0, "overlap_percentage": 12.0},
    "abs":  {"max_micro_variation": 2.5, "overlap_percentage": 20.0},
}

GOLDEN_ANGLE = 137.507764
MICRO_PRIME = 9973
MIN_CONSECUTIVE_DIFF = 5.0   # degrees
MIN_COHERENCE = 0.5          # below this the infill isn't one set of parallel lines

# Slicer settings worth reporting, read from the config block OrcaSlicer
# appends to the end of the file.
REPORTED_SETTINGS = ("sparse_infill_pattern", "infill_direction",
                     "sparse_infill_rotate_template", "enable_arc_fitting")


# =============================================================================
# ORIENTATION SCHEDULE
# =============================================================================

def compute_orientation(layer_index, seed_value, max_variation):
    """Four-component orientation for one layer. seed_value is a TRADE SECRET."""
    # Component 1 — warp-weft alternation
    base = 0.0 if layer_index % 2 == 0 else 90.0

    # Component 2 — golden angle cumulative rotation
    rotation = (layer_index * GOLDEN_ANGLE) % 360.0

    # Component 3 — seed-based micro-variation (*** TRADE SECRET seed ***)
    raw = ((seed_value * layer_index) % MICRO_PRIME) / MICRO_PRIME * 2 * max_variation - max_variation
    micro = max(-max_variation, min(max_variation, raw))

    # Component 4 — final orientation
    final = (base + rotation + micro) % 360.0

    return {"base": base, "rotation": rotation, "micro": micro, "final": final}


def angle_diff_360(a, b):
    """Smallest difference between two directions, 0–180°."""
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def angle_diff_180(a, b):
    """Smallest difference between two undirected line angles, 0–90°."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def signed_line_delta(target, current):
    """Rotation (−90, 90] that turns lines at `current` onto `target`."""
    d = (target - current) % 180.0
    return d - 180.0 if d > 90.0 else d


# =============================================================================
# SEGMENT EXTRACTION
# =============================================================================

class LayerInfill:
    def __init__(self):
        self.segments = []   # (line_no, x0, y0, x1, y1)
        self.arcs = 0        # G2/G3 moves inside sparse infill


def collect_infill(lines, layer_count):
    """Walk the whole file once and collect sparse infill segments per layer.

    Position and extrusion mode carry across layers, so the start point of
    the first infill move in a layer is known even though it was set by a
    travel move in an earlier section. Returns (per-layer list, warnings).
    """
    infill = [LayerInfill() for _ in range(layer_count)]
    warnings = []
    layer = -1
    in_infill = False
    x = y = 0.0
    relative_e = True      # OrcaSlicer default (M83)
    relative_xy = False
    last_e = 0.0

    for lineno, raw in enumerate(lines, start=1):
        line = raw.strip()
        if line == ";LAYER_CHANGE":
            layer += 1
            in_infill = False
            continue
        if line.startswith(";TYPE:"):
            in_infill = layer >= 0 and line[6:].strip().lower() in SPARSE_INFILL_TYPES
            continue

        cmd = line.split(";", 1)[0].strip().upper()
        if not cmd:
            continue
        head = cmd.split()[0]

        if head == "M83":
            relative_e = True
        elif head == "M82":
            relative_e = False
        elif head == "G90":
            relative_xy = False
        elif head == "G91":
            if not relative_xy:
                warnings.append(f"line {lineno}: G91 relative positioning — "
                                "positions after this point may be wrong")
            relative_xy = True
        elif head == "G92":
            w = parse_words(line)
            last_e = w.get("E", last_e)
            x, y = w.get("X", x), w.get("Y", y)
        elif head in ("G0", "G1", "G2", "G3"):
            w = parse_words(line)
            nx = x + w.get("X", 0.0) if relative_xy else w.get("X", x)
            ny = y + w.get("Y", 0.0) if relative_xy else w.get("Y", y)

            extrudes = False
            if "E" in w:
                if relative_e:
                    extrudes = w["E"] > 0
                else:
                    extrudes = w["E"] > last_e
                    last_e = w["E"]

            if in_infill and extrudes and (nx, ny) != (x, y):
                if head in ("G2", "G3"):
                    infill[layer].arcs += 1
                else:
                    infill[layer].segments.append((lineno, x, y, nx, ny))
            x, y = nx, ny

    return infill, warnings


def measure_angle(segments):
    """Dominant line angle of a set of segments.

    Uses a length-weighted mean of doubled angles, so lines at 179° and 1°
    average to 0° rather than 90°, and short zig-zag connectors barely count.
    Returns (angle in [0, 180), coherence 0–1). Coherence is 1.0 for
    perfectly parallel lines and near 0 for lines in many directions
    (gyroid, grid, honeycomb...).
    """
    c = s = total = 0.0
    for _, x0, y0, x1, y1 in segments:
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        theta2 = 2.0 * math.atan2(dy, dx)
        c += length * math.cos(theta2)
        s += length * math.sin(theta2)
        total += length
    if total == 0.0:
        return None, 0.0
    angle = (math.degrees(math.atan2(s, c)) / 2.0) % 180.0
    return angle, math.hypot(c, s) / total


def centroid(segments):
    """Length-weighted centroid of the deposited infill (rotation centre for Chunk 4)."""
    sx = sy = total = 0.0
    for _, x0, y0, x1, y1 in segments:
        length = math.hypot(x1 - x0, y1 - y0)
        sx += length * (x0 + x1) / 2.0
        sy += length * (y0 + y1) / 2.0
        total += length
    return (sx / total, sy / total) if total else (None, None)


def read_slicer_settings(lines, keys=REPORTED_SETTINGS):
    """Pick selected '; key = value' settings out of the slicer's config block."""
    found = {}
    for raw in lines[-2000:]:
        line = raw.strip()
        if not line.startswith("; ") or "=" not in line:
            continue
        key, value = line[2:].split("=", 1)
        if key.strip() in keys:
            found[key.strip()] = value.strip()
    return found


# =============================================================================
# REPORT
# =============================================================================

def resolve_config(args):
    cfg = dict(CONFIG)
    if args.material:
        cfg["material_profile"] = args.material
    if cfg["material_profile"]:
        cfg["max_micro_variation"] = MATERIAL_PROFILES[cfg["material_profile"]]["max_micro_variation"]
    if args.max_variation is not None:
        cfg["max_micro_variation"] = args.max_variation
    if args.seed is not None:
        cfg["seed_value"] = args.seed   # *** TRADE SECRET override ***
    return cfg


def analyze(filepath, cfg):
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        print(f"Error: cannot read file: {exc}")
        sys.exit(1)

    layers = split_layers(lines)
    if not layers:
        print("Error: no ;LAYER_CHANGE markers found — layer count is zero.")
        sys.exit(1)

    infill, warnings = collect_infill(lines, len(layers))
    settings = read_slicer_settings(lines)
    seed, max_var = cfg["seed_value"], cfg["max_micro_variation"]
    schedule = [compute_orientation(l.index, seed, max_var) for l in layers]

    print(f"File:              {filepath}")
    print(f"Total layers:      {len(layers)}")
    print(f"Material profile:  {cfg['material_profile'] or 'none (config defaults)'}")
    print(f"Max micro-var.:    {max_var}°")
    print("Seed value:        [TRADE SECRET — set in CONFIG block]")
    for key in REPORTED_SETTINGS:
        if key in settings:
            print(f"Slicer {key + ':':<30}{settings[key] or '(empty)'}")

    # --- Schedule log (same format Chunk 5 will write into the G-code header) ---
    print("\nOrientation schedule:")
    for layer, o in zip(layers, schedule):
        print(f"; GOLDEN_ANGLE_METHOD | Layer {layer.index} | Base: {o['base']:.0f}° | "
              f"Rotation: {o['rotation']:.4f}° | Micro: {o['micro']:+.4f}° | "
              f"Final: {o['final']:.4f}°")

    # --- Measured vs target ---
    print(f"\n{'Layer':>5}  {'Segs':>5}  {'Target':>7}  {'Current':>7}  "
          f"{'Coher.':>6}  {'Rotate by':>9}  {'Centroid':>17}  Note")
    print("-" * 84)
    skipped = []
    for layer, o, inf in zip(layers, schedule, infill):
        target_line = o["final"] % 180.0
        current, coherence = measure_angle(inf.segments)
        notes = []
        if current is None:
            reason = "no sparse infill"
            if inf.arcs:
                reason += f" ({inf.arcs} arc moves only)"
            skipped.append((layer.index, reason))
            print(f"{layer.index:>5}  {0:>5}  {target_line:>6.2f}°  {'—':>7}  {'—':>6}  "
                  f"{'—':>9}  {'—':>17}  skip: {reason}")
            continue

        delta = signed_line_delta(target_line, current)
        cx, cy = centroid(inf.segments)
        if coherence < MIN_COHERENCE:
            notes.append("not parallel lines")
            skipped.append((layer.index, f"infill is not one set of parallel lines "
                                         f"(coherence {coherence:.2f})"))
        if inf.arcs:
            notes.append(f"{inf.arcs} arcs")
        print(f"{layer.index:>5}  {len(inf.segments):>5}  {target_line:>6.2f}°  {current:>6.2f}°  "
              f"{coherence:>6.2f}  {delta:>+8.2f}°  ({cx:>7.2f},{cy:>7.2f})  {', '.join(notes)}")

    # --- Validation ---
    finals = [o["final"] for o in schedule]
    print("\nSummary")
    print(f"  Final orientation range:  {min(finals):.4f}° – {max(finals):.4f}°")
    print(f"  Layers with sparse infill: {sum(1 for i in infill if i.segments)}")
    print(f"  Infill segments found:     {sum(len(i.segments) for i in infill)}")

    close_360 = [(a.index, b.index, angle_diff_360(p, q))
                 for (a, p), (b, q) in zip(zip(layers, finals), zip(layers[1:], finals[1:]))
                 if angle_diff_360(p, q) < MIN_CONSECUTIVE_DIFF]
    close_180 = [(a.index, b.index, angle_diff_180(p, q))
                 for (a, p), (b, q) in zip(zip(layers, finals), zip(layers[1:], finals[1:]))
                 if angle_diff_180(p, q) < MIN_CONSECUTIVE_DIFF]
    min_step = min((angle_diff_180(p, q) for p, q in zip(finals, finals[1:])), default=None)

    print(f"  Consecutive layers ≥{MIN_CONSECUTIVE_DIFF:.0f}° apart (direction, 0–360°): "
          f"{'PASS' if not close_360 else f'FAIL ({len(close_360)})'}")
    print(f"  Consecutive layers ≥{MIN_CONSECUTIVE_DIFF:.0f}° apart (line angle, 0–180°): "
          f"{'PASS' if not close_180 else f'FAIL ({len(close_180)})'}"
          + (f"  — smallest step {min_step:.2f}°" if min_step is not None else ""))
    for a, b, d in close_180:
        print(f"    Layers {a}→{b}: lines only {d:.2f}° apart")

    if skipped:
        print(f"  Layers that would be skipped: {len(skipped)}")
        for idx, reason in skipped:
            print(f"    Layer {idx}: {reason}")
    for w in warnings:
        print(f"  Warning: {w}")
    if not any(i.segments for i in infill):
        print("  Warning: no sparse infill detected anywhere in this file.")

    # --- Slicer-native alternative ---
    template = ",".join(f"{o['final'] % 180.0:.2f}" for o in schedule)
    print("\nOrcaSlicer 'Sparse infill rotation template' for this schedule")
    print("(Process > Strength > Sparse infill rotation template; one angle per layer):")
    print(template)


def main():
    parser = argparse.ArgumentParser(
        description="Chunk 3: compare the golden-angle schedule with the infill angles in a G-code file.")
    parser.add_argument("gcode", help="input .gcode file")
    parser.add_argument("--material", choices=sorted(MATERIAL_PROFILES),
                        help="material profile (sets max micro-variation)")
    parser.add_argument("--max-variation", type=float,
                        help="max micro-variation in degrees (overrides material profile)")
    parser.add_argument("--seed", type=int,
                        help="override the seed (TRADE SECRET — prefer the CONFIG block)")
    if len(sys.argv) < 2:
        parser.print_usage()
        sys.exit(1)
    args = parser.parse_args()
    analyze(args.gcode, resolve_config(args))


if __name__ == "__main__":
    main()
