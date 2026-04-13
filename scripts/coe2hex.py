#!/usr/bin/env python3
"""
coe2hex.py — Convert a Vivado .coe memory initialisation file to the
$readmemh-compatible .hex format used by simulation.

Usage:
    python3 coe2hex.py <input.coe> <output.hex>

Format rules (matches Vivado Block Memory Generator .coe spec):
  memory_initialization_radix=16;
  memory_initialization_vector=
  HEXWORD, HEXWORD, ..., HEXWORD;

Output:
  One 8-digit zero-padded hex word per line (no separators).
  The word count and ordering are preserved exactly.
  Words beyond those present in the .coe remain uninitialised in the
  simulator (treated as zero by the $readmemh load command when the
  array was zeroed in the initial block).

The .coe files remain the source of truth for FPGA initialisation;
this script only produces a simulation-loadable equivalent.

Exit codes: 0 = success, 1 = usage / file error.
"""

import sys
import re


def coe_to_hex(src: str, dst: str) -> int:
    try:
        with open(src, "r") as f:
            raw = f.read()
    except OSError as e:
        print(f"ERROR: cannot open '{src}': {e}", file=sys.stderr)
        return 1

    # Verify radix is 16 (only supported radix for this project).
    m = re.search(r"memory_initialization_radix\s*=\s*(\d+)", raw)
    if m and int(m.group(1)) != 16:
        print(f"ERROR: unsupported radix {m.group(1)} (only 16 is handled)",
              file=sys.stderr)
        return 1

    # Strip the two header tokens; everything remaining is hex data.
    body = re.sub(r"memory_initialization_radix\s*=\s*\d+\s*;", "", raw)
    body = re.sub(r"memory_initialization_vector\s*=", "", body)

    # Extract all hex tokens (each token = one 32-bit word).
    tokens = re.findall(r"[0-9A-Fa-f]+", body)
    if not tokens:
        print(f"ERROR: no hex data found in '{src}'", file=sys.stderr)
        return 1

    try:
        with open(dst, "w") as f:
            for t in tokens:
                # Pad / truncate to exactly 8 hex digits (32-bit word).
                f.write(t.zfill(8)[:8] + "\n")
    except OSError as e:
        print(f"ERROR: cannot write '{dst}': {e}", file=sys.stderr)
        return 1

    print(f"[coe2hex] {len(tokens):5d} words: {src} → {dst}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: python3 {sys.argv[0]} <input.coe> <output.hex>",
              file=sys.stderr)
        sys.exit(1)
    sys.exit(coe_to_hex(sys.argv[1], sys.argv[2]))
