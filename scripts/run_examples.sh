#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Running examples..."

# List of examples to run (path relative to build directory)
EXAMPLES=(
    "examples/getting_started"
    "examples/atmosphere_profile"
)

for exe in "${EXAMPLES[@]}"; do
    if [ -f "$PROJECT_ROOT/build/$exe" ]; then
        echo ""
        echo "=== Running $(basename $exe) ==="
        "$PROJECT_ROOT/build/$exe"
    else
        echo "Skipping $exe (not built)"
    fi
done

echo ""
echo "All examples completed."
