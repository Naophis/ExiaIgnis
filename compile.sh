#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

mkdir -p "$BUILD_DIR"

echo "[1/2] CMake configure..."
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release

echo "[2/2] ビルド中..."
cmake --build "$BUILD_DIR" -- -j$(nproc)

# find uf2 file
UF2_FILE=$(find "$BUILD_DIR" -type f -name "*.uf2" | head -n 1)
if [ -z "$UF2_FILE" ]; then
    echo "エラー: UF2ファイルが見つかりませんでした。"
    exit 1
fi
echo "[3/3] 完了: $UF2_FILE"
