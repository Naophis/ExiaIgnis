#!/bin/bash
# USB-CDC 自動再接続シリアルモニター
# デバイスが現れた瞬間から接続し、切断後も自動で待機・再接続する
# 使用法: ./serial_watch.sh [ボーレート]

BAUD="${1:-115200}"

find_pico_port() {
    python3 -c "
from serial.tools import list_ports
ports = sorted(p.device for p in list_ports.comports() if p.vid == 0x2E8A)
print(ports[0] if ports else '')
" 2>/dev/null
}

echo "Pico を待機中... (終了: Ctrl+C)"

while true; do
    # デバイスが現れるまでポーリング
    DEVICE=""
    while [ -z "$DEVICE" ]; do
        DEVICE=$(find_pico_port)
        [ -z "$DEVICE" ] && sleep 0.2
    done

    echo "[$(date '+%H:%M:%S')] 接続: $DEVICE @ ${BAUD}bps"

    # デバイスが開けるまで待つ（udev の権限設定が間に合わないことがある）
    while ! stty -F "$DEVICE" "$BAUD" raw -echo 2>/dev/null; do
        sleep 0.05
    done

    cat "$DEVICE" 2>/dev/null

    echo ""
    echo "[$(date '+%H:%M:%S')] 切断。再接続を待機中..."
done
