#!/bin/bash
# Build script for mlx_stdout binary
# Run from the slave/src/ directory on the Raspberry Pi.
#
# Prerequisites:
#   1. BCM2835 library installed:
#        cd ../../drone_swarm_folder/drone_swarm/bcm2835-1.71
#        ./configure && make && sudo make install
#
#   2. MLX90640 library built:
#        cd ../../drone_swarm_folder/drone_swarm/lib/mlx90640-library
#        make
#
# Output: ../bin/mlx_stdout

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MLX_LIB="$REPO_ROOT/drone_swarm_folder/drone_swarm/lib/mlx90640-library"

echo "Building mlx_stdout..."
echo "  MLX lib: $MLX_LIB"

g++ -O2 -o "$SCRIPT_DIR/../bin/mlx_stdout" \
    "$SCRIPT_DIR/mlx_stdout.cpp" \
    -I"$MLX_LIB/headers" \
    -L"$MLX_LIB" \
    -lMLX90640_API \
    -lbcm2835

echo "Done: slave/bin/mlx_stdout"
echo "Run with: sudo ./bin/mlx_stdout"
