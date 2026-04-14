#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# find uf2 file in build directory
UF2_FILE=$(find "$SCRIPT_DIR/build" -maxdepth 1 -type f -name "*.uf2" | head -1)
PICOTOOL="/home/naoto/.local/bin/picotool"

if [ ! -f "$UF2_FILE" ]; then
    echo "Error: $UF2_FILE が見つかりません。先にビルドしてください。"
    exit 1
fi

echo "[1/3] BOOTSELモードへ移行中..."
SERIAL_PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
echo "      -> シリアルポート検出: ${SERIAL_PORT:-なし}"
if [ -n "$SERIAL_PORT" ]; then
    echo "      -> $SERIAL_PORT を1200bpsでオープン (マジックリセット)"
    stty -F "$SERIAL_PORT" 1200
else
    echo "      -> シリアルポートが見つかりません。picotool でリブートを試みます..."
    "$PICOTOOL" reboot -f -u 2>/dev/null || {
        echo "      -> 失敗。BOOTSELボタンを押しながらUSBを接続してください。"
    }
fi

echo "[2/3] デバイスのマウント待機中..."
MOUNT_POINT=""
for i in $(seq 1 20); do
    for mp in \
        "/media/$USER/RP2350" \
        "/media/$USER/RPI-RP2" \
        "/run/media/$USER/RP2350" \
        "/run/media/$USER/RPI-RP2"; do
        if [ -d "$mp" ]; then
            MOUNT_POINT="$mp"
            break 2
        fi
    done
    sleep 0.5
done

echo "[3/3] 書き込み中..."
if [ -n "$MOUNT_POINT" ]; then
    echo "      -> $MOUNT_POINT へコピー: $(basename "$UF2_FILE")"
    cp "$UF2_FILE" "$MOUNT_POINT/"
    echo "Done! デバイスが自動的に再起動します。"
else
    echo "      -> マウントポイントが見つかりません。picotool で書き込みます..."
    "$PICOTOOL" load -f "$UF2_FILE" -x
    echo "Done!"
fi
