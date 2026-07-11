#!/usr/bin/env bash
# SPDX-License-Identifier: BSL-1.0
#
# Runs cppcheck using the dev build compilation database.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

if [ ! -f "build/dev/compile_commands.json" ]; then
  cmake --preset dev
fi

mkdir -p build/dev/cppcheck_cache

cppcheck --project=build/dev/compile_commands.json --cppcheck-build-dir=build/dev/cppcheck_cache --library=googletest --library=qt --suppress=missingFile
