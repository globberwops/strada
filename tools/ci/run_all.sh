#!/usr/bin/env bash
# Local orchestrator script to run format checking, builds, tests, and coverage sequentially.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

CI_DIR="tools/ci"

echo "============================================="
echo "   Running all local validation stages       "
echo "============================================="

echo ""
echo ">>> Stage 1: Formatting and Static Analysis"
"$CI_DIR/run_static_checks.sh"

echo ""
echo ">>> Stage 2: default-debug Build & Test"
"$CI_DIR/build_and_test.sh" default-debug

echo ""
echo ">>> Stage 3: default-release Build & Test"
"$CI_DIR/build_and_test.sh" default-release

echo ""
echo ">>> Stage 4: san-debug (ASan/UBSan) Build & Test"
"$CI_DIR/build_and_test.sh" san-debug

echo ""
echo ">>> Stage 5: cov-debug (Coverage) Build, Test & Report"
"$CI_DIR/build_and_test.sh" cov-debug
"$CI_DIR/generate_coverage.sh"

echo ""
echo "============================================="
echo "   ALL STAGES PASSED SUCCESSFULLY!           "
echo "============================================="
