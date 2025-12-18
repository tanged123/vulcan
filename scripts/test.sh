#!/usr/bin/env bash
set -e

# Ensure we have a build
if [ ! -d "build" ]; then
    echo "Build directory not found. Building..."
    ./scripts/build.sh
fi

# Rebuild to ensure latest changes
ninja -C build

# Run tests
mkdir -p logs
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="logs/tests_${TIMESTAMP}.log"

ctest --test-dir build -VV 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "tests_${TIMESTAMP}.log" logs/tests.log
