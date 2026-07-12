#!/usr/bin/env bash
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

echo "=== [1/5] Configuring dev preset to generate compile database ==="
cmake --preset dev

echo "=== [2/5] Preparing directories ==="
mkdir -p build/dev/cppcheck_cache

echo "=== [3/5] Running pre-commit quick-checks & cppcheck ==="
pre-commit run --all-files

echo "=== [4/5] Running clang-tidy analysis ==="
"$CI_DIR/run_clang_tidy.sh"

echo "=== [5/5] Running include-what-you-use check ==="
"$CI_DIR/run_iwyu.sh"

echo "=== Formatting and static analysis checks PASSED ==="
