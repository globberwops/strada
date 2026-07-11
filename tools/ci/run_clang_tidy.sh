#!/usr/bin/env bash
# SPDX-License-Identifier: BSL-1.0
#
# Runs clang-tidy using the dev build compilation database.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

if [ ! -f "build/dev/compile_commands.json" ]; then
  cmake --preset dev
fi

if [ "$#" -gt 0 ]; then
  run-clang-tidy -p build/dev -extra-arg=-Wno-unknown-warning-option "$@"
else
  run-clang-tidy -p build/dev -extra-arg=-Wno-unknown-warning-option "src/.*|include/.*|tests/.*|examples/.*"
fi
