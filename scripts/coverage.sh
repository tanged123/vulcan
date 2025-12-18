#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/build/coverage"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/coverage_${TIMESTAMP}.log"

echo "Starting coverage build..." | tee "$LOG_FILE"

# Build with coverage
cmake -B build -G Ninja -DENABLE_COVERAGE=ON 2>&1 | tee -a "$LOG_FILE"
ninja -C build 2>&1 | tee -a "$LOG_FILE"

# Run tests
echo "Running tests..." | tee -a "$LOG_FILE"
ctest --test-dir build -VV 2>&1 | tee -a "$LOG_FILE"

# Generate coverage report
echo "Generating coverage report..." | tee -a "$LOG_FILE"
cd "$PROJECT_ROOT/build"

# Capture coverage data
llvm-profdata merge -sparse default.profraw -o coverage.profdata 2>/dev/null || true
llvm-cov export -format=lcov -instr-profile=coverage.profdata $(find . -name "test_*" -type f -executable) > coverage/coverage.info 2>/dev/null || \
lcov --capture --directory . --output-file coverage/coverage.info 2>&1 | tee -a "$LOG_FILE"

# Remove system headers and test files
lcov --remove coverage/coverage.info '/nix/*' '*/gtest/*' '*/tests/*' \
     --output-file coverage/coverage_clean.info 2>&1 | tee -a "$LOG_FILE" || true

echo "Coverage report generated at build/coverage/" | tee -a "$LOG_FILE"
ln -sf "coverage_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/coverage.log"
