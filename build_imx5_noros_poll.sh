#!/usr/bin/env bash
# Build imx5_noros_poll (ROS-free IMX-5 SPI poller) next to the source, using the SDK bundled with the driver.
set -euo pipefail
SDK="${SDK:-$HOME/vio_ar0234_icm20948/imx5_ws/src/imx5_spi_driver/third_party/inertial-sense-sdk/src}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OBJ="$(mktemp -d)"
gcc -O2 -D_LINUX -I"$SDK" -c "$SDK/ISComm.c"   -o "$OBJ/ISComm.o"
gcc -O2 -D_LINUX -I"$SDK" -c "$SDK/data_sets.c" -o "$OBJ/data_sets.o"
g++ -O2 -std=c++17 -D_LINUX -I"$SDK" "$HERE/imx5_noros_poll.cpp" "$OBJ/ISComm.o" "$OBJ/data_sets.o" -o "$HERE/imx5_noros_poll"
rm -rf "$OBJ"
echo "built $HERE/imx5_noros_poll"
