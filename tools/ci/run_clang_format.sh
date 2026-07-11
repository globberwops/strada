#!/usr/bin/env bash
# SPDX-License-Identifier: BSL-1.0
#
# Runs clang-format on specified files, or scans files in the repository if none are given.

set -e

# Change directory to repository root
CDPATH='' cd -- "$(dirname -- "$0")/../.."

if [ "$#" -gt 0 ]; then
  clang-format -i "$@"
else
  find src include tests examples -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec clang-format -i {} +
fi
