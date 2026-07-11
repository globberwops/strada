#!/usr/bin/env bash
# SPDX-License-Identifier: BSL-1.0
#
# This script configures, builds, and runs tests for a specified CMake preset.
# Usage: ./build_and_test.sh <preset-name>
# Examples:
#   ./build_and_test.sh default-debug
#   ./build_and_test.sh default-release
#   ./build_and_test.sh san-debug
#   ./build_and_test.sh cov-debug

set -e

if [ -z "$1" ]; then
  echo "Error: No build preset specified."
  echo "Usage: $0 <preset-name>"
  exit 1
fi

PRESET=$1

# Determine the corresponding configure preset
case "$PRESET" in
  default-debug|default-release)
    CONF_PRESET="default"
    ;;
  san-debug)
    CONF_PRESET="san"
    ;;
  cov-debug)
    CONF_PRESET="cov"
    ;;
  *)
    echo "Error: Unknown build preset '$PRESET'."
    echo "Supported presets: default-debug, default-release, san-debug, cov-debug"
    exit 1
    ;;
esac

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

echo "=== Configuring project with preset '$CONF_PRESET' ==="
cmake --preset "$CONF_PRESET"

echo "=== Building project with preset '$PRESET' ==="
cmake --build --preset "$PRESET"

echo "=== Running test suite with preset '$PRESET' ==="
ctest --preset "$PRESET"

echo "=== Build and test execution for preset '$PRESET' PASSED ==="
