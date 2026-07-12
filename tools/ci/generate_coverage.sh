#!/usr/bin/env bash
# This script captures code coverage, filters out external/test files,
# generates an HTML report, and outputs a terminal summary.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

COV_DIR="build/cov"
INFO_FILE="${COV_DIR}/coverage.info"
FILTERED_INFO_FILE="${COV_DIR}/coverage_filtered.info"
REPORT_DIR="${COV_DIR}/coverage_report"

if [ ! -d "$COV_DIR" ]; then
  echo "Error: Coverage directory '$COV_DIR' does not exist."
  echo "Please run './build_and_test.sh cov-debug' first to compile and execute tests."
  exit 1
fi

echo "=== [1/4] Capturing coverage data using lcov ==="
lcov --capture --directory "$COV_DIR" --output-file "$INFO_FILE" --ignore-errors inconsistent,unused

echo "=== [2/4] Filtering out external headers, tests, and build files ==="
lcov --remove "$INFO_FILE" '/usr/*' '*/tests/*' '*/examples/*' '*/build/*' --output-file "$FILTERED_INFO_FILE" --ignore-errors inconsistent,unused

echo "=== [3/4] Generating HTML coverage report ==="
rm -rf "$REPORT_DIR"
genhtml "$FILTERED_INFO_FILE" --output-directory "$REPORT_DIR" --ignore-errors inconsistent,unused

echo "=== [4/4] Displaying terminal coverage summary using gcovr ==="
gcovr -r . --object-directory "$COV_DIR" --exclude '.*tests.*' --exclude '.*examples.*' --exclude '.*build.*'

echo ""
echo "=== Coverage report successfully generated! ==="
echo "HTML report location: ${REPORT_DIR}/index.html"
