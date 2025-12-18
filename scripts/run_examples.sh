#!/usr/bin/env bash
set -e

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
