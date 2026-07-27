#!/usr/bin/env bash
# ── Round-trip helper ────────────────────────────
# Handles both native (x86_64) and QEMU (aarch64) execution.
#
# Usage:
#   ./scripts/run-roundtrip.sh model.osim [output.osim]        # PyO3 (needs OpenSim)
#   ./scripts/run-roundtrip.sh --from-json model.json [out]    # JSON import (no OpenSim)
#
# On aarch64, uses qemu-x86_64 + x86_64 Python venv automatically
# if OPENSSIM_X86_PYTHON and OPENSSIM_X86_QEMU are set.

set -euo pipefail
cd "$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$0")/..")"

# ── Detect architecture and available tools ──
ARCH=$(uname -m)
HAS_QEMU=0
X86_PYTHON="${OPENSSIM_X86_PYTHON:-}"
X86_QEMU="${OPENSSIM_X86_QEMU:-}"

if command -v qemu-x86_64 &>/dev/null; then
    HAS_QEMU=1
    X86_QEMU="${X86_QEMU:-$(which qemu-x86_64)}"
fi

# ── Check args ──
if [ $# -lt 1 ]; then
    echo "Usage:"
    echo "  $0 <input.osim> [output.osim]        # PyO3 import (needs OpenSim)"
    echo "  $0 --from-json <input.json> [out]     # JSON import (no OpenSim)"
    exit 1
fi

if [ "$1" = "--from-json" ]; then
    # ── JSON import path (works everywhere, no OpenSim needed) ──
    JSON_FILE="$2"
    OUT_FILE="${3:-roundtrip_output.osim}"
    cargo run --bin roundtrip -- --from-json "$JSON_FILE" "$OUT_FILE"
    exit $?
fi

# ── PyO3 path (needs OpenSim) ──
INPUT_FILE="$1"
OUT_FILE="${2:-roundtrip_output.osim}"

if [ "$ARCH" = "x86_64" ]; then
    # Native x86_64 — just run
    cargo run --bin roundtrip -- "$INPUT_FILE" "$OUT_FILE"
    exit $?
fi

# ── aarch64 + QEMU path ──
if [ "$HAS_QEMU" -eq 1 ] && [ -n "$X86_PYTHON" ]; then
    echo "[run-roundtrip] Using QEMU for x86_64 OpenSim on $ARCH"
    JSON_FILE="${INPUT_FILE%.osim}.json"

    # Step 1: Extract to JSON under QEMU
    echo "[1/2] Extracting model to JSON (QEMU)..."
    "$X86_QEMU" "$X86_PYTHON" scripts/extract_opensim.py "$INPUT_FILE" "$JSON_FILE"

    # Step 2: Import from JSON and export (native Rust)
    echo "[2/2] Importing from JSON + exporting to .osim..."
    cargo run --bin roundtrip -- --from-json "$JSON_FILE" "$OUT_FILE"

    # Cleanup temp JSON
    rm -f "$JSON_FILE"
    echo "[done] $OUT_FILE"
    exit $?
fi

# ── No OpenSim available ──
echo "Error: No OpenSim available on $ARCH."
echo "Options:"
echo "  - Run with --from-json <file> to import from a pre-extracted JSON file"
echo "  - On aarch64, set OPENSSIM_X86_QEMU and OPENSSIM_X86_PYTHON for QEMU path"
echo "  - Use the Nix dev shell: nix develop (auto-configures QEMU on aarch64)"
exit 1
