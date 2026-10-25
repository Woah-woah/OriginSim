#!/usr/bin/env bash
set -euo pipefail
pkg_dir=$(cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=${BUILD_DIR:-/tmp/adaptive_lio_registration_tests}
mkdir -p "$build_dir"
cmake -S "$pkg_dir/test" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j2
ctest --test-dir "$build_dir" --output-on-failure -V
