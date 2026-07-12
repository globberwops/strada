#!/usr/bin/env bash
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
elif [ -n "${CI:-}" ] || [ -n "${GITHUB_ACTIONS:-}" ]; then
  # In CI, scan all C++ files
  mapfile -t SOURCES < <(find "$PWD/src" "$PWD/tests" "$PWD/examples" -type f -name "*.cpp")
  iwyu_tool -j 0 -p build/dev "${SOURCES[@]}" "${IWYU_FLAGS[@]}"
else
  # Locally, determine changed/unstaged/untracked C++ files since merge-base with main
  MERGE_BASE=$(git merge-base main HEAD 2>/dev/null || echo "HEAD~1")
  if [ "$MERGE_BASE" = "$(git rev-parse HEAD 2>/dev/null)" ]; then
    DIFF_BASE="HEAD~1"
  else
    DIFF_BASE="$MERGE_BASE"
  fi

  echo "Running IWYU against changed files compared to $DIFF_BASE..."

  # Get list of changed C++ files (modified, added, renamed, untracked)
  mapfile -t SOURCES < <(
    (
      git diff --name-only "$DIFF_BASE" -- 'src/*.cpp' 'tests/*.cpp' 'examples/*.cpp' 2>/dev/null || true
      git status --porcelain -- 'src/*.cpp' 'tests/*.cpp' 'examples/*.cpp' 2>/dev/null | grep '^??' | cut -c 4- || true
    ) | sort -u
  )

  # Filter out empty or whitespace-only lines
  VALID_SOURCES=()
  for src in "${SOURCES[@]}"; do
    if [ -n "$src" ] && [ -f "$src" ]; then
      VALID_SOURCES+=("$(realpath "$src")")
    fi
  done

  if [ "${#VALID_SOURCES[@]}" -eq 0 ]; then
    echo "No changed C++ source files found for IWYU."
  else
    iwyu_tool -j 0 -p build/dev "${VALID_SOURCES[@]}" "${IWYU_FLAGS[@]}"
  fi
fi
