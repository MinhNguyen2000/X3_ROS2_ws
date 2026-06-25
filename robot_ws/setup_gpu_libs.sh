#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GPU_LIBS_DIR="$SCRIPT_DIR/gpu_libs"

mkdir -p "$GPU_LIBS_DIR"

# Copy instead of symlink - symlinks with absolute paths break inside the container
cp -L /usr/lib/aarch64-linux-gnu/libcudnn*.so* "$GPU_LIBS_DIR/"
cp -L /usr/lib/aarch64-linux-gnu/libnvinfer*.so* "$GPU_LIBS_DIR/"
cp -L /usr/lib/aarch64-linux-gnu/libnvonnxparser*.so* "$GPU_LIBS_DIR/"
cp -L /usr/lib/aarch64-linux-gnu/libnvcudla.so "$GPU_LIBS_DIR/"
cp -L /usr/lib/aarch64-linux-gnu/nvidia/* "$GPU_LIBS_DIR/"

echo "GPU libs copied to $GPU_LIBS_DIR"