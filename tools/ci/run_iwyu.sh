#!/usr/bin/env bash
# SPDX-License-Identifier: BSL-1.0
#
# Runs include-what-you-use using the dev build compilation database.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

if [ ! -f "build/dev/compile_commands.json" ]; then
  cmake --preset dev
fi

IWYU_FLAGS=(-- -Wno-unknown-warning-option -Xiwyu --cxx17ns)

if [ "$#" -gt 0 ]; then
  # Resolve inputs to absolute paths
  ABS_ARGS=()
  for arg in "$@"; do
    ABS_ARGS+=("$(realpath "$arg")")
  done
  iwyu_tool -j 0 -p build/dev "${ABS_ARGS[@]}" "${IWYU_FLAGS[@]}"
else
  # Find all C++ source files (translation units) and run them in parallel
  mapfile -t SOURCES < <(find "$PWD/src" "$PWD/tests" "$PWD/examples" -type f -name "*.cpp")
  iwyu_tool -j 0 -p build/dev "${SOURCES[@]}" "${IWYU_FLAGS[@]}"
fi
