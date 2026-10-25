#!/usr/bin/env python3
"""
gcode_golden_angle.py — Golden-angle non-periodic sparse infill orientation
===========================================================================

Post-processor for OrcaSlicer and ElegooSlicer G-code. Python 3.8+, standard
library only, Windows / macOS / Linux.

The deposition method gives every layer's sparse infill its own orientation:

    base      = 0° on even layers, 90° on odd layers          (warp-weft)
    rotation  = (layer × 137.507764°) mod 360                  (golden angle)
    micro     = ((S × layer) mod 9973) / 9973 × 2v − v,        (seed-based)
                clamped to ±v, where S is the seed and v the max variation
    final     = (base + rotation + micro) mod 360

*** TRADE SECRET: the seed S is set in the CONFIG block below. ***

How the orientation reaches the print
-------------------------------------
Rotating G-code coordinates after slicing would push infill through the walls
of any non-round part. Instead the slicer generates each layer's infill at the
scheduled angle, clipped to the part, from its "Sparse infill rotation
template" setting. This script produces that template, then post-processes
the sliced G-code:
  * verifies every layer's infill angle against the schedule,
  * applies the overlap window to sparse infill extrusion only:
        adjusted_E = original_E × (1 + overlap_percentage / 100)
  * inserts a documentation header with the parameters and a layer-by-layer
    orientation log (the seed itself is never written),
  * writes a log file and prints a validation report.

Workflow
--------
  1. Generate the template (use the material you will print):
         python gcode_golden_angle.py template --layers 200 --material pla
     Paste the output into Process > Strength > Sparse infill rotation template.
     Use a line pattern for sparse infill (rectilinear, aligned rectilinear,
     zig-zag, line or monotonic) and turn off "Align infill direction to model".
  2. Add this script under Others > Post-processing scripts:
         Windows:      "C:\\Python312\\python.exe" "C:\\scripts\\gcode_golden_angle.py";
         macOS/Linux:  /usr/bin/python3 /home/me/scripts/gcode_golden_angle.py;
     The slicer passes the G-code path as the last argument. The file is
     modified in place and exported as <name>_golden_angle.gcode.
  3. Slice and export. Read <name>_golden_angle.log next to the exported file.
     If it reports a constant offset, regenerate the template with the
     --offset it suggests.

Other uses
----------
  python gcode_golden_angle.py part.gcode        standalone: writes part_golden_angle.gcode
  (or drag a .gcode file onto this script)       and leaves part.gcode untouched
  python gcode_golden_angle.py verify part.gcode check orientation only, change nothing
  python gcode_golden_angle.py --self-test       verify the algorithm on synthetic G-code
  python gcode_golden_angle.py -h                all options
"""

import argparse
import logging
import math
import os
import re
import sys
import tempfile
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================
# *** TRADE SECRET ***
# seed_value drives the micro-variation component. Keep it here rather than
# passing --seed: the slicer copies the post-processing command line into
# every exported G-code file and saved .3mf project. Note that anyone holding
# the G-code can still recover the seed from the infill angles themselves.
CONFIG = {
    "seed_value": 42,                   # *** TRADE SECRET — do not disclose ***
    "max_micro_variation": 3.0,         # degrees; overridden by material_profile
    "overlap_percentage": 15.0,         # percent, 10–25; overridden by material_profile
    "material_profile": None,           # None = detect from slicer, or 'pla'|'petg'|'tpu'|'abs'
    "output_suffix": "_golden_angle",   # appended to the output file name
    "tolerance": 1.0,                   # degrees allowed between measured and target angle
    "strict": False,                    # True = block the slicer's export if verification fails
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
OVERLAP_RANGE = (10.0, 25.0)       # percent
MIN_CONSECUTIVE_DIFF = 5.0         # degrees between consecutive layers' infill lines
MIN_COHERENCE = 0.5                # below this a layer isn't one set of parallel lines
REFINE_WINDOW = 10.0               # degrees; segments this close to the first estimate are kept
OFFSET_SPREAD_MAX = 1.0            # degrees; errors this consistent indicate a constant offset
TEMPLATE_MATCH = 0.02              # degrees; template value rounding allowance
MARKER = "; GOLDEN_ANGLE_METHOD"

# ;TYPE: values that are sparse infill (exact match, lower-case). OrcaSlicer
# and ElegooSlicer write "Sparse infill"; "Internal infill" is the older
# PrusaSlicer name. "Internal solid infill" is solid fill and is excluded.
SPARSE_INFILL_TYPES = {"sparse infill", "internal infill"}

# Sparse infill patterns that lay one set of parallel lines per layer
# (keys from s_keys_map_InfillPattern in OrcaSlicer's PrintConfig.cpp).
LINE_PATTERNS = {"monotonic", "monotonicline", "rectilinear", "alignedrectilinear", "zigzag", "line"}

VERIFY_SETTINGS = ("sparse_infill_pattern", "sparse_infill_rotate_template",
                   "align_infill_direction_to_model", "raft_layers")

# Number after an E word in the command part of a line: E1.5, E-0.8, E.0321
RE_E_WORD = re.compile(r"(?<![A-Za-z])E([-+]?(?:\d+\.?\d*|\.\d+))")
# Blocks some printers' screens expect at the very top of the file
RE_TOP_BLOCK_END = re.compile(r";\s*(HEADER_BLOCK_END|THUMBNAIL_BLOCK_END|thumbnail(_\w+)? end)",
                              re.IGNORECASE)

log = logging.getLogger("golden_angle")
log.addHandler(logging.NullHandler())   # silent until setup_logging() runs
log.propagate = False


class FatalError(Exception):
    pass


# =============================================================================
# ORIENTATION SCHEDULE
# =============================================================================
# Angles are counter-clockwise from the printer's +X axis. Infill lines have no
# direction, so 10° and 190° are the same lines: a "line angle" is the
# orientation folded into [0, 180).

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


def angle_diff_180(a, b):
    """Smallest difference between two undirected line angles, 0–90°."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def signed_line_delta(target, current):
    """Rotation (−90, 90] that turns lines at `current` onto `target`."""
    d = (target - current) % 180.0
    return d - 180.0 if d > 90.0 else d


def circular_mean_180(values):
    """Mean of line angles/offsets (period 180°) and the largest deviation from it."""
    c = sum(math.cos(math.radians(2 * v)) for v in values)
    s = sum(math.sin(math.radians(2 * v)) for v in values)
    mean = math.degrees(math.atan2(s, c)) / 2.0
    return mean, max(angle_diff_180(v, mean) for v in values)


def schedule_line_angles(layer_count, cfg, offset=0.0):
    """Template values for object layers 0..layer_count-1, in [0, 180).

    seed_value inside cfg is a TRADE SECRET; only the angles leave this function.
    """
    seed, max_var = cfg["seed_value"], cfg["max_micro_variation"]
    return [(compute_orientation(i, seed, max_var)["final"] + offset) % 180.0
            for i in range(layer_count)]


def format_template(angles):
    # Plain non-negative decimals only: a '+' or '-' would switch OrcaSlicer's
    # parser into its rotation metalanguage instead of a per-layer list.
    return ",".join(f"{a:.2f}" for a in angles)


def parse_template(text):
    """Angles in a plain comma-separated template, or None if it isn't one."""
    try:
        values = [float(v) for v in text.replace(" ", ",").split(",") if v]
    except ValueError:
        return None
    return values or None


# =============================================================================
# G-CODE PARSING
# =============================================================================

class Layer:
    def __init__(self, index, start_line):
        self.index = index
        self.start_line = start_line   # 1-based line of ;LAYER_CHANGE
        self.end_line = start_line     # 1-based last line of this layer
        self.z = None                  # from ;Z: comment, if present


class LayerInfill:
    def __init__(self):
        self.segments = []   # (line_no, x0, y0, x1, y1)
        self.arcs = 0        # G2/G3 moves inside sparse infill


def parse_words(line):
    """{letter: float} for a G-code line, ignoring any ; comment."""
    words = {}
    for token in line.split(";", 1)[0].split()[1:]:
        try:
            words[token[0].upper()] = float(token[1:])
        except (ValueError, IndexError):
            pass
    return words


def split_layers(lines):
    """Layer objects, one per ;LAYER_CHANGE marker."""
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


def read_slicer_settings(lines, keys):
    """Selected '; key = value' settings from the config block the slicer
    appends to the end of the file."""
    found = {}
    for raw in lines[-3000:]:
        line = raw.strip()
        if not line.startswith("; ") or "=" not in line:
            continue
        key, value = line[2:].split("=", 1)
        if key.strip() in keys:
            found[key.strip()] = value.strip()
    return found


def raft_layer_count(lines):
    try:
        return int(read_slicer_settings(lines, ("raft_layers",)).get("raft_layers", "0") or 0)
    except ValueError:
        return 0


def collect_infill(lines, layer_count):
    """Sparse infill segments per layer, from one pass over the whole file.

    Position and extrusion mode carry across layers, so the start point of a
    layer's first infill move is known even when an earlier travel set it.
    Returns (per-layer list of LayerInfill, warnings).
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

    Length-weighted mean of doubled angles, so 179° and 1° average to 0° rather
    than 90°. Returns (angle in [0, 180), coherence 0–1): coherence is 1.0 for
    perfectly parallel lines and near 0 for lines in many directions (gyroid,
    grid, honeycomb...).
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
    return (math.degrees(math.atan2(s, c)) / 2.0) % 180.0, math.hypot(c, s) / total


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
        if angle_diff_180(math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0, first) <= REFINE_WINDOW:
            near.append(seg)
    refined, _ = measure_angle(near) if near else (first, coherence)
    return refined, coherence


# =============================================================================
# CONFIGURATION RESOLUTION
# =============================================================================

def detect_material(lines):
    """(profile, source) from the slicer's filament_type, or (None, reason)."""
    raw = os.environ.get("SLIC3R_FILAMENT_TYPE")
    if raw is None and lines is not None:
        raw = read_slicer_settings(lines, ("filament_type",)).get("filament_type")
    if not raw:
        return None, "no filament_type found"
    types = {t.strip().strip('"').lower() for t in raw.split(";") if t.strip()}
    if len(types) != 1:
        return None, f"several filament types ({raw}) — pass --material"
    material = types.pop()
    if material not in MATERIAL_PROFILES:
        return None, f"filament type '{material}' has no profile — pass --material"
    return material, "from slicer filament_type"


def resolve_config(args, lines=None):
    """Effective settings. Precedence: command line > CONFIG block >
    slicer filament_type (when G-code lines are given) > defaults."""
    cfg = dict(CONFIG)
    material = getattr(args, "material", None)
    if material:
        source = "command line"
    elif cfg["material_profile"]:
        material, source = cfg["material_profile"], "CONFIG block"
    elif lines is not None:
        material, source = detect_material(lines)
    else:
        source = None
    cfg["material_profile"], cfg["material_source"] = material, source

    if material:
        cfg.update(MATERIAL_PROFILES[material])
    if getattr(args, "max_variation", None) is not None:
        cfg["max_micro_variation"] = args.max_variation
    if getattr(args, "seed", None) is not None:
        cfg["seed_value"] = args.seed   # *** TRADE SECRET override ***
    if getattr(args, "overlap", None) is not None:
        cfg["overlap_percentage"] = args.overlap
    for key in ("tolerance", "suffix"):
        if getattr(args, key, None) is not None:
            cfg["output_suffix" if key == "suffix" else key] = getattr(args, key)
    if getattr(args, "strict", False):
        cfg["strict"] = True

    low, high = OVERLAP_RANGE
    if not low <= cfg["overlap_percentage"] <= high:
        clamped = min(max(cfg["overlap_percentage"], low), high)
        cfg["notes"] = [f"overlap {cfg['overlap_percentage']}% is outside "
                        f"{low:g}–{high:g}%; clamped to {clamped}%"]
        cfg["overlap_percentage"] = clamped
    else:
        cfg["notes"] = []
    return cfg


def describe_material(cfg):
    text = cfg["material_profile"] or "none"
    return text + (f" ({cfg['material_source']})" if cfg["material_source"] else "")


# =============================================================================
# VERIFICATION
# =============================================================================

def template_offset(values, cfg, object_layers):
    """(offset, matches) of a file's template against the schedule for cfg."""
    expected = schedule_line_angles(object_layers, cfg)
    used = [values[i % len(values)] for i in range(object_layers)]
    offset, spread = circular_mean_180([signed_line_delta(u, e) for u, e in zip(used, expected)])
    return (0.0 if abs(offset) < 0.005 else offset), spread <= TEMPLATE_MATCH


def verify_lines(lines, layers, cfg):
    """Compare the infill angles in G-code with the schedule.

    Returns a report dict: 'header', 'table' and 'summary' are lists of
    printable lines; 'ok', 'checked', 'passed', 'skipped', 'warnings' and
    'problems' carry the results.
    """
    tolerance = cfg["tolerance"]
    settings = read_slicer_settings(lines, VERIFY_SETTINGS)
    infill, warnings = collect_infill(lines, len(layers))
    raft = raft_layer_count(lines)
    object_layers = len(layers) - raft
    expected = schedule_line_angles(object_layers, cfg)

    header = [f"Layers:           {len(layers)} ({raft} raft + {object_layers} object)",
              f"Tolerance:        ±{tolerance}°"]

    # --- Settings checks ---
    problems = []
    pattern = settings.get("sparse_infill_pattern")
    if pattern is not None:
        header.append(f"Infill pattern:   {pattern}")
        if pattern not in LINE_PATTERNS:
            problems.append(f"infill pattern '{pattern}' is not a single-direction line pattern; "
                            f"use one of: {', '.join(sorted(LINE_PATTERNS))}")
    if settings.get("align_infill_direction_to_model", "0") not in ("0", "false", ""):
        problems.append("align_infill_direction_to_model is on — it adds the model's rotation "
                        "to every infill angle; turn it off")
    if raft:
        warnings.append(f"{raft} raft layers: assuming the template is indexed from the first "
                        "object layer above the raft — check the per-layer results")

    file_template = settings.get("sparse_infill_rotate_template")
    if file_template is None:
        header.append("Template in file: not found (no slicer config block in this file)")
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
            # The file's template may legitimately include a constant --offset.
            offset, matches = template_offset(values, cfg, object_layers)
            if matches:
                header.append(f"Template in file: matches schedule ({len(values)} angles, "
                              f"offset {offset:+.2f}°)")
            else:
                # Most often the template was made for a different material profile.
                hint = "different seed or max variation?"
                for name in sorted(MATERIAL_PROFILES):
                    trial = dict(cfg, max_micro_variation=MATERIAL_PROFILES[name]["max_micro_variation"])
                    if name != cfg["material_profile"] and template_offset(values, trial, object_layers)[1]:
                        hint = (f"it matches the '{name}' profile — run with --material {name}, "
                                "or regenerate the template for this material")
                        break
                problems.append(f"template in the file does not match this schedule: {hint}")

    # --- Per-layer measurement ---
    table = [f"{'Layer':>5}  {'Obj':>4}  {'Target':>7}  {'Measured':>8}  {'Error':>7}  "
             f"{'Coher.':>6}  Status",
             "-" * 56]
    errors = []      # (layer index, signed error)
    skipped = []
    for layer, inf in zip(layers, infill):
        obj = layer.index - raft
        if obj < 0:
            continue
        target = expected[obj]
        measured, coherence = measure_refined(inf.segments)
        if measured is None:
            reason = "no sparse infill" + (f" ({inf.arcs} arc moves only)" if inf.arcs else "")
            skipped.append((layer.index, reason))
            table.append(f"{layer.index:>5}  {obj:>4}  {target:>6.2f}°  {'—':>8}  {'—':>7}  "
                         f"{'—':>6}  skip: {reason}")
            continue
        if coherence < MIN_COHERENCE:
            skipped.append((layer.index, f"not parallel lines (coherence {coherence:.2f})"))
            table.append(f"{layer.index:>5}  {obj:>4}  {target:>6.2f}°  {measured:>7.2f}°  "
                         f"{'—':>7}  {coherence:>6.2f}  skip: not parallel lines")
            continue
        if inf.arcs:
            warnings.append(f"layer {layer.index}: {inf.arcs} arc moves in sparse infill were not "
                            "measured — consider turning off arc fitting")
        err = signed_line_delta(measured, target)
        err = 0.0 if abs(err) < 0.005 else err   # avoid printing "-0.00"
        errors.append((layer.index, err))
        status = "OK" if abs(err) <= tolerance else "OFF"
        table.append(f"{layer.index:>5}  {obj:>4}  {target:>6.2f}°  {measured:>7.2f}°  "
                     f"{err:>+6.2f}°  {coherence:>6.2f}  {status}")

    # --- Summary ---
    checked = len(errors)
    passed = sum(1 for _, e in errors if abs(e) <= tolerance)
    summary = [f"Layers checked:  {checked}",
               f"Within ±{tolerance}°:    {passed}/{checked}"]
    if errors:
        summary.append(f"Largest error:   {max(abs(e) for _, e in errors):.2f}°")
    if errors and passed < checked:
        offset, spread = circular_mean_180([e for _, e in errors])
        if spread <= OFFSET_SPREAD_MAX:
            summary.append(f"All layers are off by the same {offset:+.2f}° (±{spread:.2f}°): "
                           "the slicer adds a constant rotation to the template.")
            summary.append(f"Fix: regenerate the template with --offset {-offset:.2f} "
                           "and slice again.")
        else:
            summary.append("Errors are not a constant offset — the template is probably not "
                           "applied to every layer, or layers are indexed differently.")

    summary += [f"Skipped layer {idx}: {reason}" for idx, reason in skipped]
    summary += [f"Warning: {w}" for w in warnings]
    summary += [f"Problem: {p}" for p in problems]
    if checked == 0:
        summary.append("Warning: no layers with measurable sparse infill — nothing to verify.")

    return {"header": header, "table": table, "summary": summary,
            "ok": checked > 0 and passed == checked and not problems,
            "checked": checked, "passed": passed, "skipped": skipped,
            "warnings": warnings, "problems": problems}


# =============================================================================
# OVERLAP WINDOW
# =============================================================================

def replace_e(line, new_e, original_text):
    """Swap the E value in a line's command part, keeping everything else."""
    decimals = len(original_text.split(".", 1)[1]) if "." in original_text else 0
    text = f"{new_e:.{max(decimals, 5)}f}"
    code, sep, comment = line.partition(";")
    return RE_E_WORD.sub("E" + text, code, count=1) + sep + comment


def apply_overlap(lines, layer_count, factor):
    """Scale E on sparse infill extrusion moves by `factor`.

    Returns (new_lines, moves_per_layer, warnings). Relative E (M83): each
    infill move's E is scaled. Absolute E (M82): the extra material is carried
    forward as an offset added to every later E value until the next G92 E
    reset, so retractions and other features stay consistent. Travels,
    retractions, primes and every non-infill feature are left alone.
    """
    out = list(lines)
    moves = [0] * layer_count
    warnings = []
    layer = -1
    in_infill = False
    relative_e = True      # OrcaSlicer default (M83)
    last_e = 0.0           # absolute mode: last E value as written in the input
    offset = 0.0           # absolute mode: extra E accumulated since the last G92

    for i, raw in enumerate(lines):
        line = raw.strip()
        if line == ";LAYER_CHANGE":
            layer += 1
            in_infill = False
            continue
        if line.startswith(";TYPE:"):
            in_infill = layer >= 0 and line[6:].strip().lower() in SPARSE_INFILL_TYPES
            continue

        code = line.split(";", 1)[0].strip().upper()
        if not code:
            continue
        head = code.split()[0]
        if head == "M83":
            relative_e = True
        elif head == "M82":
            relative_e = False
        elif head == "G91":
            warnings.append(f"line {i + 1}: G91 relative positioning — overlap may be "
                            "applied incorrectly after this point")
        elif head == "G92":
            words = parse_words(line)
            if "E" in words or len(code.split()) == 1:   # bare G92 resets all axes
                last_e, offset = words.get("E", 0.0), 0.0
        elif head in ("G0", "G1", "G2", "G3"):
            match = RE_E_WORD.search(raw.split(";", 1)[0])
            if not match:
                continue
            e = float(match.group(1))
            deposits = in_infill and (head in ("G2", "G3") or "X" in code or "Y" in code)
            if relative_e:
                if deposits and e > 0:
                    out[i] = replace_e(raw, e * factor, match.group(1))
                    moves[layer] += 1
            else:
                delta = e - last_e
                last_e = e
                if deposits and delta > 0:
                    offset += delta * (factor - 1.0)
                    moves[layer] += 1
                if offset:
                    out[i] = replace_e(raw, e + offset, match.group(1))

    return out, moves, warnings


# =============================================================================
# DOCUMENTATION HEADER
# =============================================================================

def header_insert_index(lines):
    """Insert after thumbnail/header blocks, which some printers expect first."""
    index = 0
    for i, raw in enumerate(lines):
        line = raw.strip()
        if line and not line.startswith(";"):
            break
        if RE_TOP_BLOCK_END.match(line):
            index = i + 1
    return index


def build_header(cfg, source_name, layers, raft, schedule, moves, verify, eol):
    """Documentation block. The seed is a TRADE SECRET and is never included."""
    rows = [
        "; " + "=" * 67,
        f"{MARKER} — non-periodic sparse infill orientation",
        f"; processed:            {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"; source:               {source_name}",
        "; seed_value:           [TRADE SECRET — withheld]",
        f"; material_profile:     {describe_material(cfg)}",
        f"; max_micro_variation:  {cfg['max_micro_variation']} deg",
        f"; overlap_percentage:   {cfg['overlap_percentage']} % "
        f"(sparse infill E x {1 + cfg['overlap_percentage'] / 100:.4f})",
        "; orientation_method:   slicer sparse_infill_rotate_template (coordinates unchanged)",
        f"; layers:               {len(layers)} ({raft} raft + {len(layers) - raft} object)",
        f"; infill_moves_scaled:  {sum(moves)}",
        f"; verification:         {'PASS' if verify['ok'] else 'FAIL'} — {verify['passed']}/"
        f"{verify['checked']} layers within ±{cfg['tolerance']}°",
        "; " + "-" * 67,
    ]
    for obj, o in enumerate(schedule):
        rows.append(f"{MARKER} | Layer {obj} | Base: {o['base']:.0f}° | "
                    f"Rotation: {o['rotation']:.4f}° | Micro: {o['micro']:+.4f}° | "
                    f"Final: {o['final']:.4f}°")
    for idx, reason in verify["skipped"]:
        rows.append(f"{MARKER} | Skipped | Layer {idx - raft}: {reason}")
    rows.append("; " + "=" * 67)
    return [r + eol for r in rows]


# =============================================================================
# FILES AND LOGGING
# =============================================================================

def read_lines(path):
    # newline="" keeps CRLF line endings; surrogateescape round-trips any
    # bytes that aren't valid UTF-8, so untouched lines are written back exactly.
    try:
        with open(path, "r", encoding="utf-8", errors="surrogateescape", newline="") as fh:
            return fh.readlines()
    except OSError as exc:
        raise FatalError(f"cannot read {path}: {exc}")


def write_lines(path, lines):
    """Write via a temporary file so a failure never leaves a half-written G-code."""
    tmp = path + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8", errors="surrogateescape", newline="") as fh:
            fh.writelines(lines)
        os.replace(tmp, path)
    except OSError as exc:
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise FatalError(f"cannot write {path}: {exc}")


def plan_paths(input_path, suffix, in_place):
    """(write_path, log_path, new_output_name or None)."""
    if not in_place:
        base, ext = os.path.splitext(input_path)
        return base + suffix + ext, base + suffix + ".log", None

    # Slicer mode: modify in place; ask the slicer to rename the exported file.
    output_name = os.environ.get("SLIC3R_PP_OUTPUT_NAME", "")
    new_name = None
    log_dir = os.path.dirname(input_path)
    if output_name:
        base, ext = os.path.splitext(os.path.basename(output_name))
        new_name = (base if base.endswith(suffix) else base + suffix) + (ext or ".gcode")
        out_dir = os.path.dirname(output_name)
        if os.path.isabs(output_name) and os.path.isdir(out_dir):
            log_dir = out_dir
    stem = os.path.splitext(new_name or os.path.basename(input_path))[0]
    return input_path, os.path.join(log_dir, stem + ".log"), new_name


def setup_logging(log_path, console=True):
    log.setLevel(logging.INFO)
    log.propagate = False
    log.handlers.clear()
    log.addHandler(logging.NullHandler())   # no stray stderr output if the log file fails
    if console and sys.stdout is not None:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter("%(message)s"))
        log.addHandler(handler)
    try:
        handler = logging.FileHandler(log_path, mode="w", encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(message)s",
                                               "%Y-%m-%d %H:%M:%S"))
        log.addHandler(handler)
    except OSError as exc:
        log.warning(f"cannot create log file {log_path}: {exc}")


def close_logging():
    for handler in list(log.handlers):
        handler.close()
        log.removeHandler(handler)


# =============================================================================
# POST-PROCESSING
# =============================================================================

def process(args, console=True):
    """Post-process one G-code file. Returns an exit code; raises FatalError."""
    input_path = os.path.abspath(args.gcode)
    in_place = getattr(args, "in_place", False) or "SLIC3R_PP_OUTPUT_NAME" in os.environ
    suffix = args.suffix if getattr(args, "suffix", None) else CONFIG["output_suffix"]
    write_path, log_path, new_name = plan_paths(input_path, suffix, in_place)
    setup_logging(log_path, console)
    log.info(f"Golden angle post-processor ({'slicer / in-place' if in_place else 'standalone'})")
    log.info(f"Input:  {input_path}")

    lines = read_lines(input_path)
    if any(l.startswith(MARKER) for l in lines[:5000]):
        raise FatalError("file already contains a GOLDEN_ANGLE_METHOD header — processing it "
                         "again would apply the overlap twice")

    layers = split_layers(lines)
    if not layers:
        raise FatalError("no ;LAYER_CHANGE markers found — layer count is zero. "
                         "Slice with OrcaSlicer or ElegooSlicer.")

    cfg = resolve_config(args, lines)
    for note in cfg["notes"]:
        log.warning(note)
    raft = raft_layer_count(lines)
    seed, max_var = cfg["seed_value"], cfg["max_micro_variation"]   # seed: TRADE SECRET
    schedule = [compute_orientation(i, seed, max_var) for i in range(len(layers) - raft)]
    log.info(f"Material: {describe_material(cfg)}, max micro-variation {max_var}°, "
             f"overlap {cfg['overlap_percentage']}%")

    factor = 1.0 + cfg["overlap_percentage"] / 100.0
    new_lines, moves, overlap_warnings = apply_overlap(lines, len(layers), factor)
    for w in overlap_warnings:
        log.warning(w)
    if sum(moves) == 0:
        log.warning("No sparse infill detected — nothing to do; the G-code was not modified. "
                    "Check that sparse infill is enabled and the file has ;TYPE: comments.")
        return 0

    verify = verify_lines(lines, layers, cfg)
    eol = "\r\n" if lines[0].endswith("\r\n") else "\n"
    header = build_header(cfg, os.path.basename(input_path), layers, raft, schedule, moves,
                          verify, eol)
    at = header_insert_index(new_lines)
    write_lines(write_path, new_lines[:at] + header + new_lines[at:])
    log.info(f"Output: {write_path}")
    if new_name:
        try:
            with open(input_path + ".output_name", "w", encoding="utf-8") as fh:
                fh.write(new_name + "\n")
            log.info(f"Exported file will be named: {new_name}")
        except OSError as exc:
            log.warning(f"could not rename the exported file: {exc}")
    log.info(f"Log:    {log_path}")

    report(layers, raft, schedule, moves, verify, cfg["tolerance"])
    if cfg["strict"] and not verify["ok"]:
        print(f"Orientation verification failed (strict mode) — see {log_path}", file=sys.stderr)
        return 2
    return 0


def report(layers, raft, schedule, moves, verify, tolerance):
    """Validation report to console and log."""
    finals = [o["final"] for o in schedule]
    steps = [angle_diff_180(a, b) for a, b in zip(finals, finals[1:])]
    close = [(i, i + 1, d) for i, d in enumerate(steps) if d < MIN_CONSECUTIVE_DIFF]

    log.info("")
    log.info("=" * 62)
    log.info("VALIDATION REPORT")
    log.info("=" * 62)
    log.info(f"Total layers processed:        {len(layers)} ({raft} raft + {len(schedule)} object)")
    log.info(f"Layers with overlap applied:   {sum(1 for m in moves if m)}")
    log.info(f"Sparse infill moves adjusted:  {sum(moves)}")
    if finals:
        log.info(f"Final orientation range:       {min(finals):.4f}° – {max(finals):.4f}°")
    if close:
        log.warning(f"Consecutive layers within {MIN_CONSECUTIVE_DIFF:.0f}°: {len(close)}")
        for a, b, d in close:
            log.warning(f"  Layers {a}→{b}: infill lines only {d:.2f}° apart")
    else:
        log.info(f"Consecutive layers ≥{MIN_CONSECUTIVE_DIFF:.0f}° apart:  PASS "
                 f"(smallest step {min(steps, default=0):.2f}°)")
    log.info(f"Orientation verification:      {'PASS' if verify['ok'] else 'FAIL'} "
             f"({verify['passed']}/{verify['checked']} layers within ±{tolerance}°)")
    for line in verify["header"]:
        log.info(f"  {line}")
    for line in verify["summary"]:
        level = log.warning if line.startswith(("Problem", "Warning", "All layers", "Fix",
                                                "Errors")) else log.info
        level(f"  {line}")
    log.info("Per-layer orientation check:")
    for line in verify["table"]:
        log.info(f"  {line}")
    log.info("=" * 62)


# =============================================================================
# TEMPLATE AND VERIFY COMMANDS
# =============================================================================

def cmd_template(args):
    lines = None
    if args.from_gcode:
        try:
            with open(args.from_gcode, "r", encoding="utf-8", errors="replace") as fh:
                lines = fh.readlines()
        except OSError as exc:
            raise FatalError(f"cannot read {args.from_gcode}: {exc}")
        layer_count = len(split_layers(lines)) - raft_layer_count(lines)
        if layer_count <= 0:
            raise FatalError("no object layers found in that G-code file")
    else:
        layer_count = args.layers

    cfg = resolve_config(args, lines)
    template = format_template(schedule_line_angles(layer_count, cfg, args.offset))
    if args.out:
        try:
            with open(args.out, "w", encoding="utf-8") as fh:
                fh.write(template + "\n")
        except OSError as exc:
            raise FatalError(f"cannot write {args.out}: {exc}")

    # Status goes to stderr so stdout can be piped or redirected cleanly.
    err = sys.stderr
    print(f"{layer_count} layers, material {describe_material(cfg)}, max micro-variation "
          f"{cfg['max_micro_variation']}°, offset {args.offset:+.2f}°", file=err)
    if not cfg["material_profile"]:
        print("Note: no material profile. If the slicer's filament type is PLA, PETG, TPU or "
              "ABS the post-processor will use that profile's max variation and this template "
              "will not match — pass --material.", file=err)
    print("Paste into Process > Strength > Sparse infill rotation template:", file=err)
    if args.out:
        print(f"(also written to {args.out})", file=err)
    print(template)
    return 0


def cmd_verify(args):
    try:
        with open(args.gcode, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        raise FatalError(f"cannot read {args.gcode}: {exc}")
    layers = split_layers(lines)
    if not layers:
        raise FatalError("no ;LAYER_CHANGE markers found — layer count is zero")

    cfg = resolve_config(args, lines)
    rep = verify_lines(lines, layers, cfg)
    print(f"File:             {args.gcode}")
    print(f"Material profile: {describe_material(cfg)}")
    print("Seed value:       [TRADE SECRET — set in CONFIG block]")
    for line in rep["header"]:
        print(line)
    print()
    for line in rep["table"]:
        print(line)
    print("\nSummary")
    for line in rep["summary"]:
        print(f"  {line}")
    print(f"\nResult: {'PASS' if rep['ok'] else 'FAIL'}")
    return 0 if rep["ok"] else 2


# =============================================================================
# SELF-TEST
# =============================================================================

# Reference schedule for seed 42, max variation 3.0°, layers 0–9, computed by
# hand from the formula. It checks the algorithm, independent of the secret seed.
REFERENCE_SEED = 42
REFERENCE_FINALS = [357.0000, 224.5330, 272.0661, 139.5991, 187.1321,
                    54.6652, 102.1982, 329.7312, 17.2643, 244.7973]


def synthetic_gcode(angles, absolute_e=False, crlf=False):
    """Minimal OrcaSlicer-style G-code: one object, len(angles) layers.

    Layer 0 and the last layer are solid; the others have rectilinear sparse
    infill (zig-zag connected lines inside a 40 mm circle) at angles[i].
    """
    def infill_points(angle, cx=120.0, cy=120.0, r=20.0, spacing=3.0):
        a = math.radians(angle)
        ux, uy, nx, ny = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
        pts, k, flip = [], -r + spacing / 2, False
        while k < r:
            h = math.sqrt(r * r - k * k)
            p0 = (cx + nx * k - ux * h, cy + ny * k - uy * h)
            p1 = (cx + nx * k + ux * h, cy + ny * k + uy * h)
            pts += [p1, p0] if flip else [p0, p1]
            flip, k = not flip, k + spacing
        return pts

    e = 0.0
    out = ["; HEADER_BLOCK_START", "; generated by OrcaSlicer (synthetic self-test)",
           "; HEADER_BLOCK_END", "", "M82" if absolute_e else "M83", "G92 E0"]

    def extrude(x, y, amount):
        nonlocal e
        e += amount
        out.append(f"G1 X{x:.3f} Y{y:.3f} E{e if absolute_e else amount:.5f}")

    last = len(angles) - 1
    for i, angle in enumerate(angles):
        out += [";LAYER_CHANGE", f";Z:{0.2 * (i + 1):.1f}", f"G1 Z{0.2 * (i + 1):.1f} F600",
                ";TYPE:Outer wall", "G0 X140 Y120 F12000"]
        extrude(100, 120, 2.0)
        if i in (0, last):
            out.append(";TYPE:Bottom surface" if i == 0 else ";TYPE:Top surface")
            extrude(100, 130, 1.0)
            continue
        out.append(";TYPE:Sparse infill")
        pts = infill_points(angle)
        out.append(f"G0 X{pts[0][0]:.3f} Y{pts[0][1]:.3f}")
        for x, y in pts[1:]:
            extrude(x, y, 0.5)
        # retract / unretract: must not be scaled
        out.append(f"G1 E{e - 0.8 if absolute_e else -0.8:.5f} F2400")
        out.append(f"G1 E{e if absolute_e else 0.8:.5f} F2400")
        if absolute_e and i % 3 == 0:
            out.append("G92 E0")
            e = 0.0
    out += ["; sparse_infill_pattern = rectilinear",
            f"; sparse_infill_rotate_template = {format_template(angles)}",
            "; align_infill_direction_to_model = 0", "; raft_layers = 0",
            "; filament_type = PETG"]
    eol = "\r\n" if crlf else "\n"
    return eol.join(out) + eol


def deposited(path):
    """Independent reading of a file: [(feature type, material deposited)] for
    every move with E, used to check the overlap without reusing apply_overlap."""
    result, relative, last, feature = [], True, 0.0, ""
    with open(path, encoding="utf-8", errors="surrogateescape") as fh:
        for raw in fh:
            line = raw.strip()
            if line.startswith(";TYPE:"):
                feature = line[6:].lower()
                continue
            code = line.split(";")[0]
            if code.startswith(("M83",)):
                relative = True
            elif code.startswith("M82"):
                relative = False
            elif code.startswith("G92"):
                last = 0.0
            elif code.startswith(("G0", "G1")) and " E" in code:
                e = float(code.split(" E")[1].split()[0])
                result.append((feature, e if relative else e - last, " X" in code))
                if not relative:
                    last = e
    return result


def run_self_test():
    passed = failed = 0

    def check(name, ok, detail=""):
        nonlocal passed, failed
        passed, failed = passed + bool(ok), failed + (not ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{'  — ' + detail if detail else ''}")

    print("=" * 72)
    print("SELF-TEST — golden angle orientation schedule")
    print("=" * 72)

    # 1. Reference schedule (public test seed, not the secret one)
    print(f"\n1. Reference schedule: seed {REFERENCE_SEED}, max variation 3.0°, layers 0–9")
    print(f"   {'Layer':>5}  {'Base':>6}  {'Rotation':>10}  {'Micro':>8}  {'Final':>9}  {'Expected':>9}")
    worst = 0.0
    for i, expected in enumerate(REFERENCE_FINALS):
        o = compute_orientation(i, REFERENCE_SEED, 3.0)
        worst = max(worst, abs(o["final"] - expected))
        print(f"   {i:>5}  {o['base']:>5.0f}°  {o['rotation']:>9.4f}°  {o['micro']:>+7.4f}°  "
              f"{o['final']:>8.4f}°  {expected:>8.4f}°")
    check("final orientations match the hand-computed values", worst < 1e-4,
          f"largest difference {worst:.6f}°")

    # 2. Properties of the configured schedule (seed not printed)
    cfg = resolve_config(argparse.Namespace())
    seed, max_var = cfg["seed_value"], cfg["max_micro_variation"]
    sched = [compute_orientation(i, seed, max_var) for i in range(500)]
    print(f"\n2. Configured schedule: seed [TRADE SECRET], max variation {max_var}°, layers 0–499")
    check("base angle alternates 0° / 90°",
          all(o["base"] == (0.0 if i % 2 == 0 else 90.0) for i, o in enumerate(sched)))
    check("golden-angle component is (i × 137.507764) mod 360",
          all(abs(o["rotation"] - (i * GOLDEN_ANGLE) % 360) < 1e-9 for i, o in enumerate(sched)))
    check(f"micro-variation within ±{max_var}°", all(abs(o["micro"]) <= max_var for o in sched))
    finals = [o["final"] for o in sched]
    steps = [angle_diff_180(a, b) for a, b in zip(finals, finals[1:])]
    check(f"consecutive layers ≥ {MIN_CONSECUTIVE_DIFF:.0f}° apart",
          min(steps) >= MIN_CONSECUTIVE_DIFF, f"smallest step {min(steps):.2f}°")
    print(f"   Layers 0–9 final orientation: "
          + ", ".join(f"{o['final']:.2f}°" for o in sched[:10]))

    # 3. End-to-end on synthetic G-code
    print("\n3. Synthetic G-code, 10 layers, sliced at the PETG schedule")
    angles = schedule_line_angles(10, resolve_config(argparse.Namespace(material="petg")))
    factor = 1 + MATERIAL_PROFILES["petg"]["overlap_percentage"] / 100
    saved_env = {k: os.environ.pop(k, None) for k in ("SLIC3R_PP_OUTPUT_NAME", "SLIC3R_FILAMENT_TYPE")}
    with tempfile.TemporaryDirectory() as tmp:
        def run(name, text, **options):
            path = os.path.join(tmp, name)
            with open(path, "w", encoding="utf-8", newline="") as fh:
                fh.write(text)
            args = argparse.Namespace(gcode=path, in_place=False, **options)
            try:
                code = process(args, console=False)
            except FatalError as exc:
                code = f"error: {exc}"
            finally:
                close_logging()
            return path, os.path.splitext(path)[0] + CONFIG["output_suffix"] + ".gcode", code

        for label, absolute, crlf in (("relative E", False, False), ("absolute E, CRLF", True, True)):
            src, out, code = run(f"{'abs' if absolute else 'rel'}.gcode",
                                 synthetic_gcode(angles, absolute, crlf))
            check(f"{label}: processed", code == 0 and os.path.exists(out), f"exit {code}")
            if code != 0:
                continue
            layers = split_layers(read_lines(out))
            check(f"{label}: 10 layers detected", len(layers) == 10, f"{len(layers)} found")
            with open(out, encoding="utf-8") as fh:
                text = fh.read()
            check(f"{label}: orientation verified", "; verification:         PASS" in text)
            check(f"{label}: header has 10 layer log lines",
                  sum(1 for l in text.splitlines() if l.startswith(MARKER + " | Layer")) == 10)
            check(f"{label}: header placed after the header block",
                  text.splitlines()[3].startswith("; ====="))
            check(f"{label}: seed withheld", "seed_value:           [TRADE SECRET — withheld]" in text
                  and not re.search(r"seed_value:\s*\d", text))
            before, after = deposited(src), deposited(out)
            wrong = sum(1 for (f, d0, xy), (_, d1, _) in zip(before, after)
                        if abs(d1 - (d0 * factor if f == "sparse infill" and d0 > 0 and xy else d0)) > 1e-4)
            check(f"{label}: overlap ×{factor:.2f} on sparse infill only",
                  len(before) == len(after) and wrong == 0, f"{wrong} moves wrong")
            if crlf:
                with open(out, "rb") as fh:
                    data = fh.read()
                check(f"{label}: line endings preserved", data.count(b"\n") == data.count(b"\r\n"))

        with open(os.path.join(tmp, "rel_golden_angle.gcode"), encoding="utf-8") as fh:
            _, _, code = run("twice.gcode", fh.read())
        check("refuses to process a file twice", str(code).startswith("error"))
        _, out, code = run("solid.gcode", synthetic_gcode(angles).replace("Sparse infill", "Internal solid infill"))
        check("no sparse infill: exits cleanly, writes nothing", code == 0 and not os.path.exists(out))
        _, _, code = run("empty.gcode", "G28\nG1 X10 Y10 E1\n")
        check("no layers: clear error", "layer count is zero" in str(code))
        _, out, code = run("wrong.gcode", synthetic_gcode(angles), material="pla")
        with open(out, encoding="utf-8") as fh:
            check("template made for another material is reported",
                  "; verification:         FAIL" in fh.read())
    for key, value in saved_env.items():
        if value is not None:
            os.environ[key] = value

    print("\n" + "=" * 72)
    print(f"SELF-TEST {'PASSED' if not failed else 'FAILED'}: {passed} passed, {failed} failed")
    print("=" * 72)
    return 0 if not failed else 1


# =============================================================================
# COMMAND LINE
# =============================================================================

def add_schedule_options(parser):
    parser.add_argument("--material", choices=sorted(MATERIAL_PROFILES),
                        help="material profile (default: CONFIG block, else the slicer's "
                             "filament type)")
    parser.add_argument("--max-variation", type=float,
                        help="max micro-variation in degrees (overrides material profile)")
    parser.add_argument("--seed", type=int,
                        help="override the seed (TRADE SECRET — prefer the CONFIG block: the "
                             "slicer copies this command line into every exported file)")


def build_parsers():
    process_parser = argparse.ArgumentParser(
        prog="gcode_golden_angle.py",
        description="Golden-angle sparse infill post-processor for OrcaSlicer / ElegooSlicer.",
        epilog="Other commands:\n"
               "  gcode_golden_angle.py template (--layers N | --from-gcode FILE) [--offset DEG]\n"
               "  gcode_golden_angle.py verify FILE\n"
               "  gcode_golden_angle.py --self-test",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    process_parser.add_argument("gcode", help="G-code file (the slicer passes it as the last argument)")
    add_schedule_options(process_parser)
    process_parser.add_argument("--overlap", type=float,
                                help=f"overlap percentage {OVERLAP_RANGE[0]:g}–{OVERLAP_RANGE[1]:g} "
                                     "(overrides material profile)")
    process_parser.add_argument("--suffix", help=f"output suffix (default {CONFIG['output_suffix']})")
    process_parser.add_argument("--tolerance", type=float,
                                help=f"orientation tolerance in degrees (default {CONFIG['tolerance']})")
    process_parser.add_argument("--strict", action="store_true",
                                help="exit with an error, blocking the slicer's export, if the "
                                     "orientation check fails")
    process_parser.add_argument("--in-place", action="store_true",
                                help="modify the file in place, as when run by the slicer")
    process_parser.add_argument("--no-pause", action="store_true",
                                help="don't wait for a key press after drag-and-drop on Windows")

    template_parser = argparse.ArgumentParser(
        prog="gcode_golden_angle.py template",
        description="Print the sparse infill rotation template for the schedule.")
    source = template_parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--layers", type=int, help="number of object layers")
    source.add_argument("--from-gcode", metavar="FILE",
                        help="take the layer count (and filament type) from a sliced G-code file")
    add_schedule_options(template_parser)
    template_parser.add_argument("--offset", type=float, default=0.0,
                                 help="constant correction in degrees, as reported by verify")
    template_parser.add_argument("--out", metavar="FILE", help="also write the template to this file")

    verify_parser = argparse.ArgumentParser(
        prog="gcode_golden_angle.py verify",
        description="Check a sliced G-code file against the schedule without changing it.")
    verify_parser.add_argument("gcode", help="G-code file")
    add_schedule_options(verify_parser)
    verify_parser.add_argument("--tolerance", type=float,
                               help=f"allowed angle error in degrees (default {CONFIG['tolerance']})")
    return process_parser, template_parser, verify_parser


def launched_by_drag_and_drop(argv):
    """Windows runs a dropped file as `script.py <file>` in a console that closes
    on exit; detect that so the report stays readable."""
    return (os.name == "nt" and len(argv) <= 1 and "SLIC3R_PP_OUTPUT_NAME" not in os.environ
            and sys.stdin is not None and sys.stdin.isatty())


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    # Never crash on characters like ° or → when output is redirected to a
    # non-UTF-8 console or file (Windows code pages, slicer pipes).
    for stream in (sys.stdout, sys.stderr):
        if stream is not None and hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    pause = launched_by_drag_and_drop(argv)
    process_parser, template_parser, verify_parser = build_parsers()
    try:
        if not argv:
            process_parser.print_help()
            code = 1
        elif argv[0] in ("--self-test", "self-test", "selftest"):
            code = run_self_test()
        elif argv[0] == "template":
            args = template_parser.parse_args(argv[1:])
            if args.layers is not None and args.layers <= 0:
                template_parser.error("--layers must be positive")
            code = cmd_template(args)
        elif argv[0] == "verify":
            code = cmd_verify(verify_parser.parse_args(argv[1:]))
        else:
            args = process_parser.parse_args(argv)
            pause = pause and not args.no_pause
            code = process(args)
    except FatalError as exc:
        # Log file only; the console copy goes to stderr, which the slicer
        # shows in its error dialog.
        for handler in list(log.handlers):
            if type(handler) is logging.StreamHandler:
                log.removeHandler(handler)
        log.error(f"Error: {exc}")
        print(f"Golden angle post-processor error: {exc}", file=sys.stderr)
        code = 1
    finally:
        close_logging()

    if pause:
        try:
            input("\nPress Enter to close...")
        except EOFError:
            pass
    return code


if __name__ == "__main__":
    sys.exit(main())
