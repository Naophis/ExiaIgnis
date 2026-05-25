#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
UF2_FILE="$(find "$SCRIPT_DIR/build" -maxdepth 1 -type f -name "*.uf2" | head -1)"
PICOTOOL="$HOME/.local/bin/picotool"

if [ -z "${UF2_FILE:-}" ] || [ ! -f "$UF2_FILE" ]; then
    echo "Error: UF2 が見つかりません。先にビルドしてください。"
    exit 1
fi

echo "[1/2] picotool direct load..."
if "$PICOTOOL" load -f -x "$UF2_FILE"; then
    echo "Done!"
    exit 0
fi

echo "[2/2] direct load失敗。BOOTSELへ再起動して再試行..."
"$PICOTOOL" reboot -f -u || true
sleep 0.3
"$PICOTOOL" load -f -x "$UF2_FILE"
echo "Done!"