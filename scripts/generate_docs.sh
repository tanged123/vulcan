#!/bin/bash
set -e

# Ensure doxygen is installed
if ! command -v doxygen &> /dev/null; then
    echo "Doxygen could not be found. Please install it."
    exit 1
fi

echo "Generating documentation..."
mkdir -p build/docs
doxygen Doxyfile
echo "Documentation generated in build/docs/html"
