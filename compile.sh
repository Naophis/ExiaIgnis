#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

mkdir -p "$BUILD_DIR"

echo "[1/2] ビルド中..."
cmake --build "$BUILD_DIR" -- -j$(nproc)

# fined uf2 file
UF2_FILE=$(find "$BUILD_DIR" -type f -name "*.uf2" | head -n 1)
if [ -z "$UF2_FILE" ]; then
    echo "エラー: UF2ファイルが見つかりませんでした。"
    exit 1
fi
echo "[2/2] 完了: $UF2_FILE"
