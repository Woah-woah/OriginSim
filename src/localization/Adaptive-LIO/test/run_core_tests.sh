#!/usr/bin/env bash
set -euo pipefail
pkg_dir=$(cd -- "$(dirname -- "$0")/.." && pwd)
eigen_dir=${EIGEN3_INCLUDE_DIR:-/usr/include/eigen3}
build_dir=${BUILD_DIR:-/tmp/adaptive_lio_core_tests}
mkdir -p "$build_dir"
"${CXX:-g++}" -std=c++17 -O1 -Wall -Wextra -pedantic -pthread \
  -I"$eigen_dir" -I"$pkg_dir/src" -I"$pkg_dir/thirdparty" \
  "$pkg_dir/test/test_core.cpp" -o "$build_dir/test_core"
"$build_dir/test_core"
"${CXX:-g++}" -std=c++17 -O1 -Wall -Wextra -pthread \
  -I"$eigen_dir" -I"$pkg_dir/src" -I"$pkg_dir/thirdparty" \
  "$pkg_dir/test/test_geometry.cpp" -o "$build_dir/test_geometry"
"$build_dir/test_geometry"
"${CXX:-g++}" -std=c++17 -O1 -Wall -Wextra -pthread \
  -I"$eigen_dir" -I"$pkg_dir/src" -I"$pkg_dir/thirdparty" \
  "$pkg_dir/test/test_imu_quality.cpp" -o "$build_dir/test_imu_quality"
"$build_dir/test_imu_quality"
