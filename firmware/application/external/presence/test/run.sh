#!/usr/bin/env bash
# Builds and runs the host tests for presence_dsp with the native g++.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../../.." && pwd)"
OUT="${TMPDIR:-/tmp}/presence_dsp_test"
g++ -std=c++17 -O1 -Wall -Wextra -Werror \
    -I "$ROOT/firmware/test/include" -I "$HERE/.." \
    "$HERE/test_presence_dsp.cpp" "$HERE/../presence_dsp.cpp" -o "$OUT"
"$OUT" "$@"
