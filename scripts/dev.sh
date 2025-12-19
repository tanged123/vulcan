#!/usr/bin/env bash
# Enter the Vulcan development environment
if [ $# -gt 0 ]; then
    nix develop --command "$@"
else
    nix develop
fi
