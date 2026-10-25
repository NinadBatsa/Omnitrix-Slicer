#!/usr/bin/env python3
"""
Chunk 5 — Post-processor: extrusion overlap, documentation header, logging.

Runs as an OrcaSlicer / ElegooSlicer post-processing script, or by hand:

  * Applies the overlap window to sparse infill only:
        adjusted_E = original_E × (1 + overlap_percentage / 100)
    Walls, solid/top/bottom infill, supports, travels, retractions and
    primes are untouched. Absolute-E files (M82) are re-accumulated so every
    later E value stays consistent.
  * Verifies the orientation the slicer produced (Chunk 4) — the coordinates
    themselves are never changed.
  * Inserts a documentation header with the parameters and the layer-by-layer
    orientation log. The seed value is a TRADE SECRET and is never written.
  * Writes a log file next to the output and prints a validation report.

Slicer mode (detected via SLIC3R_PP_OUTPUT_NAME, or forced with --in-place):
the file is modified in place, as the slicer requires, and the exported file
is renamed to <name>_golden_angle.gcode via the slicer's .output_name hook.
Set it up under Others > Post-processing scripts, e.g.
    "C:\\Python312\\python.exe" "C:\\scripts\\gcode_golden_angle_chunk5.py";
    /usr/bin/python3 /home/me/scripts/gcode_golden_angle_chunk5.py;

Standalone mode: the input is left untouched and <name>_golden_angle.gcode
is written next to it.
"""

import argparse
import logging
import os
import re
import sys
from datetime import datetime

from gcode_golden_angle_chunk2 import SPARSE_INFILL_TYPES, parse_words, split_layers
from gcode_golden_angle_chunk3 import (CONFIG, MATERIAL_PROFILES, angle_diff_180,
                                       compute_orientation, read_slicer_settings, resolve_config)
from gcode_golden_angle_chunk4 import DEFAULT_TOLERANCE, verify_lines

DEFAULT_OVERLAP = 15.0           # percent, used when no material profile applies
OVERLAP_RANGE = (10.0, 25.0)     # percent
OUTPUT_SUFFIX = "_golden_angle"
MARKER = "; GOLDEN_ANGLE_METHOD"
MIN_CONSECUTIVE_DIFF = 5.0       # degrees, between consecutive layers' infill lines

# Number after an E word in the command part of a line: E1.5, E-0.8, E.0321
RE_E_WORD = re.compile(r"(?<![A-Za-z])E([-+]?(?:\d+\.?\d*|\.\d+))")
# Blocks some printers' screens expect at the very top of the file
RE_TOP_BLOCK_END = re.compile(r";\s*(HEADER_BLOCK_END|THUMBNAIL_BLOCK_END|thumbnail(_\w+)? end)",
                              re.IGNORECASE)

log = logging.getLogger("golden_angle")


class FatalError(Exception):
    pass


# =============================================================================
# CONFIGURATION
# =============================================================================

def detect_material(lines):
    """Material profile from the slicer's filament_type, or (None, reason)."""
    raw = os.environ.get("SLIC3R_FILAMENT_TYPE")
    if raw is None:
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


def build_config(args, lines):
    """Precedence: command line > CONFIG block > slicer filament_type > defaults."""
    source = "command line" if args.material else None
    if not args.material and CONFIG["material_profile"]:
        source = "CONFIG block"
    if not args.material and not CONFIG["material_profile"]:
        args.material, source = detect_material(lines)
    cfg = resolve_config(args)   # sets max_micro_variation; seed is a TRADE SECRET
    cfg["material_source"] = source

    if args.overlap is not None:
        overlap = args.overlap
        low, high = OVERLAP_RANGE
        if not low <= overlap <= high:
            log.warning(f"overlap {overlap}% is outside {low:g}–{high:g}%; clamped")
            overlap = min(max(overlap, low), high)
    elif cfg["material_profile"]:
        overlap = MATERIAL_PROFILES[cfg["material_profile"]]["overlap_percentage"]
    else:
        overlap = DEFAULT_OVERLAP
    cfg["overlap_percentage"] = overlap
    return cfg


# =============================================================================
# OVERLAP
# =============================================================================

def replace_e(line, new_e, original_text):
    """Swap the E value in a line's command part, keeping everything else."""
    decimals = len(original_text.split(".", 1)[1]) if "." in original_text else 0
    text = f"{new_e:.{max(decimals, 5)}f}"
    code, sep, comment = line.partition(";")
    code = RE_E_WORD.sub("E" + text, code, count=1)
    return code + sep + comment


def apply_overlap(lines, layer_count, factor):
    """Scale E on sparse infill extrusion moves by `factor`.

    Returns (new_lines, moves_per_layer, warnings). In relative-E mode each
    infill move's E is scaled. In absolute-E mode the extra material is
    carried forward as an offset added to every later E value until the next
    G92 E reset, so retractions and other features stay consistent.
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
# HEADER
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


def build_header(cfg, source_name, layers, raft, schedule, moves, skipped, verify, eol):
    """Documentation block. The seed is a TRADE SECRET and is never included."""
    material = cfg["material_profile"] or "none"
    if cfg["material_source"]:
        material += f" ({cfg['material_source']})"
    rows = [
        "; " + "=" * 67,
        f"{MARKER} — non-periodic sparse infill orientation",
        f"; processed:            {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"; source:               {source_name}",
        "; seed_value:           [TRADE SECRET — withheld]",
        f"; material_profile:     {material}",
        f"; max_micro_variation:  {cfg['max_micro_variation']} deg",
        f"; overlap_percentage:   {cfg['overlap_percentage']} % "
        f"(sparse infill E x {1 + cfg['overlap_percentage'] / 100:.4f})",
        "; orientation_method:   slicer sparse_infill_rotate_template (coordinates unchanged)",
        f"; layers:               {len(layers)} ({raft} raft + {len(layers) - raft} object)",
        f"; infill_moves_scaled:  {sum(moves)}",
        f"; verification:         {'PASS' if verify['ok'] else 'FAIL'} — {verify['passed']}/"
        f"{verify['checked']} layers within tolerance",
        "; " + "-" * 67,
    ]
    for obj, o in enumerate(schedule):
        rows.append(f"{MARKER} | Layer {obj} | Base: {o['base']:.0f}° | "
                    f"Rotation: {o['rotation']:.4f}° | Micro: {o['micro']:+.4f}° | "
                    f"Final: {o['final']:.4f}°")
    for idx, reason in skipped:
        rows.append(f"{MARKER} | Skipped | Layer {idx - raft}: {reason}")
    rows.append("; " + "=" * 67)
    return [r + eol for r in rows]


# =============================================================================
# FILE HANDLING
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
    """Return (write_path, log_path, new_output_name or None)."""
    if not in_place:
        base, ext = os.path.splitext(input_path)
        return base + suffix + ext, base + suffix + ".log", None

    # Slicer mode: modify in place; ask the slicer to rename the exported file.
    output_name = os.environ.get("SLIC3R_PP_OUTPUT_NAME", "")
    new_name = None
    log_dir = os.path.dirname(input_path)
    if output_name:
        base, ext = os.path.splitext(os.path.basename(output_name))
        new_name = base if base.endswith(suffix) else base + suffix
        new_name += ext or ".gcode"
        out_dir = os.path.dirname(output_name)
        if os.path.isabs(output_name) and os.path.isdir(out_dir):
            log_dir = out_dir
    stem = os.path.splitext(new_name or os.path.basename(input_path))[0]
    return input_path, os.path.join(log_dir, stem + ".log"), new_name


def setup_logging(log_path):
    log.setLevel(logging.INFO)
    log.handlers.clear()
    log.addHandler(logging.NullHandler())   # no stray stderr output if the log file fails
    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(logging.Formatter("%(message)s"))
    log.addHandler(console)
    try:
        handler = logging.FileHandler(log_path, mode="w", encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(message)s",
                                               "%Y-%m-%d %H:%M:%S"))
        log.addHandler(handler)
    except OSError as exc:
        log.warning(f"cannot create log file {log_path}: {exc}")


# =============================================================================
# MAIN
# =============================================================================

def process(args):
    input_path = os.path.abspath(args.gcode)
    in_place = args.in_place or "SLIC3R_PP_OUTPUT_NAME" in os.environ
    write_path, log_path, new_name = plan_paths(input_path, args.suffix, in_place)
    setup_logging(log_path)
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

    cfg = build_config(args, lines)
    raft = int(read_slicer_settings(lines, ("raft_layers",)).get("raft_layers", "0") or 0)
    seed, max_var = cfg["seed_value"], cfg["max_micro_variation"]   # seed: TRADE SECRET
    schedule = [compute_orientation(i, seed, max_var) for i in range(len(layers) - raft)]
    log.info(f"Material: {cfg['material_profile'] or 'none'}"
             f"{' (' + cfg['material_source'] + ')' if cfg['material_source'] else ''}, "
             f"max micro-variation {max_var}°, overlap {cfg['overlap_percentage']}%")

    factor = 1.0 + cfg["overlap_percentage"] / 100.0
    new_lines, moves, overlap_warnings = apply_overlap(lines, len(layers), factor)
    for w in overlap_warnings:
        log.warning(w)
    if sum(moves) == 0:
        log.warning("No sparse infill detected — nothing to do; the G-code was not modified. "
                    "Check that sparse infill is enabled and the file has ;TYPE: comments.")
        return 0

    verify = verify_lines(lines, layers, cfg, args.tolerance)
    header = build_header(cfg, os.path.basename(input_path), layers, raft, schedule, moves,
                          verify["skipped"], verify, "\r\n" if lines[0].endswith("\r\n") else "\n")
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

    report(layers, raft, schedule, moves, verify, args.tolerance)
    if args.strict and not verify["ok"]:
        print("Orientation verification failed (--strict) — see the log for details:",
              log_path, file=sys.stderr)
        return 2
    return 0


def report(layers, raft, schedule, moves, verify, tolerance):
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
        (log.warning if line.startswith(("Problem", "Warning", "All layers", "Fix", "Errors"))
         else log.info)(f"  {line}")
    log.info("Per-layer orientation check:")
    for line in verify["table"]:
        log.info(f"  {line}")
    log.info("=" * 62)


def main():
    parser = argparse.ArgumentParser(
        description="Chunk 5: golden-angle post-processor — overlap, header and validation.")
    parser.add_argument("gcode", help="G-code file (the slicer passes it as the last argument)")
    parser.add_argument("--material", choices=sorted(MATERIAL_PROFILES),
                        help="material profile (default: detected from the slicer's filament type)")
    parser.add_argument("--max-variation", type=float,
                        help="max micro-variation in degrees (overrides material profile)")
    parser.add_argument("--overlap", type=float,
                        help=f"overlap percentage {OVERLAP_RANGE[0]:g}–{OVERLAP_RANGE[1]:g} "
                             "(overrides material profile)")
    parser.add_argument("--suffix", default=OUTPUT_SUFFIX,
                        help=f"output filename suffix (default {OUTPUT_SUFFIX})")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                        help=f"orientation tolerance in degrees (default {DEFAULT_TOLERANCE})")
    parser.add_argument("--strict", action="store_true",
                        help="exit with an error (blocking the slicer's export) if the "
                             "orientation check fails")
    parser.add_argument("--in-place", action="store_true",
                        help="modify the file in place, as when run by the slicer")
    parser.add_argument("--seed", type=int,
                        help="override the seed (TRADE SECRET — prefer the CONFIG block: the "
                             "slicer copies this command line into every exported file)")
    if len(sys.argv) < 2:
        parser.print_usage()
        sys.exit(1)
    args = parser.parse_args()
    try:
        sys.exit(process(args))
    except FatalError as exc:
        # Record it in the log file only; the console copy goes to stderr,
        # which the slicer shows in its error dialog.
        for handler in list(log.handlers):
            if type(handler) is logging.StreamHandler:
                log.removeHandler(handler)
        log.error(f"Error: {exc}")
        print(f"Golden angle post-processor error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
