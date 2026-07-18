#!/usr/bin/env bash
# Runs clang-tidy using the dev build compilation database.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

# Build the custom clang-tidy plugin
echo "Building custom clang-tidy plugin..."
cmake -B tools/clang_tidy_plugin/build -S tools/clang_tidy_plugin
cmake --build tools/clang_tidy_plugin/build

PLUGIN_PATH="tools/clang_tidy_plugin/build/libstrada_tidy_plugin.so"

if [ ! -f "build/dev/compile_commands.json" ]; then
  cmake --preset dev
fi

if [ "$#" -gt 0 ]; then
  run-clang-tidy -p build/dev -load "$PLUGIN_PATH" -checks=strada-local-auto-style -extra-arg=-Wno-unknown-warning-option "$@"
elif [ -n "${CI:-}" ] || [ -n "${GITHUB_ACTIONS:-}" ]; then
  run-clang-tidy -p build/dev -load "$PLUGIN_PATH" -checks=strada-local-auto-style -extra-arg=-Wno-unknown-warning-option "^(src|include|tests|examples)/.*\.(cpp|hpp|cc|h)$"
else
  CLANG_TIDY_DIFF=$(which clang-tidy-diff-21.py || which clang-tidy-diff.py || find /usr/bin -name "clang-tidy-diff-*.py" | head -n 1)
  if [ -z "$CLANG_TIDY_DIFF" ]; then
    echo "Warning: clang-tidy-diff not found. Falling back to full run-clang-tidy."
    run-clang-tidy -p build/dev -load "$PLUGIN_PATH" -checks=strada-local-auto-style -extra-arg=-Wno-unknown-warning-option "^(src|include|tests|examples)/.*\.(cpp|hpp|cc|h)$"
  else
    MERGE_BASE=$(git merge-base main HEAD 2>/dev/null || echo "HEAD~1")
    if [ "$MERGE_BASE" = "$(git rev-parse HEAD 2>/dev/null)" ]; then
      DIFF_BASE="HEAD~1"
    else
      DIFF_BASE="$MERGE_BASE"
    fi
    echo "Running clang-tidy-diff against $DIFF_BASE..."
    git diff -U0 --no-color "$DIFF_BASE" | "$CLANG_TIDY_DIFF" -j "$(nproc)" -p1 -path build/dev -load "$PLUGIN_PATH" -checks=strada-local-auto-style -extra-arg=-Wno-unknown-warning-option -regex "^(src|include|tests|examples)/.*\.(cpp|hpp|cc|h)$"
  fi
fi
