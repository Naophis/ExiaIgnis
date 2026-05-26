#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_FILE="$SCRIPT_DIR/build/ExiaIgnis.bin"
PICOTOOL="$HOME/.local/bin/picotool"
# RP2350 flash base address
FLASH_OFFSET="0x10000000"

if [ ! -f "$BIN_FILE" ]; then
    echo "Error: バイナリが見つかりません。先にビルドしてください。"
    exit 1
fi

echo "[1/2] picotool direct load (differential)..."
if "$PICOTOOL" load -f -u -x "$BIN_FILE" -t bin -o "$FLASH_OFFSET"; then
    echo "Done!"
    exit 0
fi

echo "[2/2] direct load失敗。BOOTSELへ再起動して再試行..."
"$PICOTOOL" reboot -f -u || true
sleep 0.3
"$PICOTOOL" load -f -u -x "$BIN_FILE" -t bin -o "$FLASH_OFFSET"
echo "Done!"