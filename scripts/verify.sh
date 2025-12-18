#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/verify_${TIMESTAMP}.log"

echo "Starting full verification..." | tee "$LOG_FILE"

# Run everything inside nix develop
cd "$PROJECT_ROOT"
nix develop --command bash -c "
    echo '=== Building ===' && \
    ./scripts/build.sh && \
    echo '=== Running Tests ===' && \
    ./scripts/test.sh && \
    echo '=== Running Examples ===' && \
    ./scripts/run_examples.sh
" 2>&1 | tee -a "$LOG_FILE"

ln -sf "verify_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/verify.log"
echo "Verification complete! Logs at logs/verify_${TIMESTAMP}.log"
