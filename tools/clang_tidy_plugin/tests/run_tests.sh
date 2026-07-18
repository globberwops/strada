#!/usr/bin/env bash
set -euo pipefail

# Get directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDPATH='' cd -- "$SCRIPT_DIR"

# 1. Build the plugin
echo "Building clang-tidy plugin..."
cmake -B ../build -S ..
cmake --build ../build

PLUGIN_PATH="../build/libstrada_tidy_plugin.so"

if [ ! -f "$PLUGIN_PATH" ]; then
  echo "Error: plugin library not found at $PLUGIN_PATH"
  exit 1
fi

# 2. Run clang-tidy and verify the warning count
echo "Checking warnings count..."
# Run clang-tidy and count warnings matching [strada-local-auto-style]
WARNINGS_COUNT=$(clang-tidy -load="$PLUGIN_PATH" -checks=-*,strada-local-auto-style test_local_auto_style.cpp -- -std=c++20 2>/dev/null | grep -c "\[strada-local-auto-style\]" || true)

EXPECTED_COUNT=13
if [ "$WARNINGS_COUNT" -ne "$EXPECTED_COUNT" ]; then
  echo "Error: Expected $EXPECTED_COUNT warnings, but got $WARNINGS_COUNT"
  exit 1
fi
echo "Warnings count verified: $WARNINGS_COUNT warnings detected."

# 3. Verify fix-its
echo "Verifying fix-its..."
cp test_local_auto_style.cpp test_local_auto_style.tmp.cpp

# Apply fixes
clang-tidy -load="$PLUGIN_PATH" -checks=-*,strada-local-auto-style -fix test_local_auto_style.tmp.cpp -- -std=c++20 >/dev/null 2>&1

# Diff against expected fixed version
if ! diff -u test_local_auto_style.tmp.cpp test_local_auto_style.fixed.cpp; then
  echo "Error: Fix-its output does not match expected output!"
  rm -f test_local_auto_style.tmp.cpp
  exit 1
fi

rm -f test_local_auto_style.tmp.cpp
echo "All tests passed successfully!"
