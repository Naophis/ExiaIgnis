#!/bin/bash
# USB シリアルモニター
# 使用法: ./serial_monitor.sh [デバイス] [ボーレート]

BAUD="${2:-115200}"

# ポート自動検出: 引数がなければ Raspberry Pi Pico (VID=2e8a) を探す
if [ -n "$1" ]; then
    DEVICE="$1"
else
    DEVICE=$(python3 -c "
from serial.tools import list_ports
ports = sorted(p.device for p in list_ports.comports() if p.vid == 0x2E8A)
print(ports[0] if ports else '')
" 2>/dev/null)
    if [ -z "$DEVICE" ]; then
        echo "エラー: Pico が見つかりません"
        echo "利用可能なデバイス:"
        ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  なし"
        exit 1
    fi
fi

if [ ! -e "$DEVICE" ]; then
    echo "エラー: デバイス $DEVICE が見つかりません"
    exit 1
fi

echo "シリアルモニター起動: $DEVICE @ ${BAUD}bps"
echo "終了: Ctrl+A → Ctrl+X  (picocom) / Ctrl+A → K (screen)"
echo "---"

if command -v picocom &>/dev/null; then
    picocom "$DEVICE" -b "$BAUD" --omap crcrlf
elif command -v screen &>/dev/null; then
    screen "$DEVICE" "$BAUD"
else
    echo "picocom / screen が見つかりません。どちらかをインストールしてください:"
    echo "  sudo apt install picocom"
    exit 1
fi
