#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

mkdir -p "$BUILD_DIR"

echo "[1/2] ビルド中..."
cmake --build "$BUILD_DIR" -- -j$(nproc)

echo "[2/2] 完了: $BUILD_DIR/test.uf2"
