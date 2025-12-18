#!/usr/bin/env bash
set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Ensure logs directory exists
mkdir -p "$PROJECT_ROOT/logs"

# Create timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/ci_${TIMESTAMP}.log"

# Run build and test scripts inside the nix environment
echo "Running CI under Nix..."
cd "$PROJECT_ROOT"
nix develop --command bash -c "./scripts/build.sh --clean && ./scripts/test.sh" 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "ci_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/ci.log"

echo "CI Complete. Logs available at logs/ci_${TIMESTAMP}.log (symlinked to logs/ci.log)"
