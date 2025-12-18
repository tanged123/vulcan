#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/verify_${TIMESTAMP}.log"

echo "Starting full verification..." | tee "$LOG_FILE"

echo "=== Building ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/build.sh" 2>&1 | tee -a "$LOG_FILE"

echo "=== Running Tests ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/test.sh" 2>&1 | tee -a "$LOG_FILE"

echo "=== Running Examples ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/run_examples.sh" 2>&1 | tee -a "$LOG_FILE"

ln -sf "verify_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/verify.log"
echo "Verification complete! Logs at logs/verify_${TIMESTAMP}.log"
