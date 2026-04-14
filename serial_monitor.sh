#!/bin/bash
# USB シリアルモニター
# 使用法: ./serial_monitor.sh [デバイス] [ボーレート]

DEVICE="${1:-/dev/ttyACM0}"
BAUD="${2:-115200}"

if [ ! -e "$DEVICE" ]; then
    echo "エラー: デバイス $DEVICE が見つかりません"
    echo "利用可能なデバイス:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  なし"
    exit 1
fi

echo "シリアルモニター起動: $DEVICE @ ${BAUD}bps"
echo "終了: Ctrl+C"
echo "---"

# ボーレート設定
stty -F "$DEVICE" "$BAUD" raw -echo

# 受信データをターミナルに出力
cat "$DEVICE"
