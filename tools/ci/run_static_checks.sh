#!/usr/bin/env bash
# SPDX-License-Identifier: BSL-1.0
#
# This script runs code formatting, style, and static analysis checks.
# It configures the project to generate the compile commands database required by clang-tidy and cppcheck.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

CI_DIR="tools/ci"

# Workaround for git dubious ownership warnings in container environments
if [ -n "$GITHUB_WORKSPACE" ]; then
  git config --global --add safe.directory "$GITHUB_WORKSPACE"
fi

echo "=== [1/6] Configuring and building dev preset for autogen files ==="
cmake --preset dev
cmake --build --preset dev-debug

echo "=== [2/6] Preparing directories ==="
mkdir -p build/dev/cppcheck_cache

echo "=== [3/6] Running pre-commit formatting & commit-stage hooks ==="
pre-commit run --all-files

echo "=== [4/6] Running clang-tidy analysis ==="
"$CI_DIR/run_clang_tidy.sh"

echo "=== [5/6] Running cppcheck analysis ==="
"$CI_DIR/run_cppcheck.sh"

echo "=== [6/6] Running include-what-you-use check ==="
"$CI_DIR/run_iwyu.sh"

echo "=== Formatting and static analysis checks PASSED ==="
