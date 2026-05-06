#!/usr/bin/env python3
"""
Pico へ USB CDC 経由でファイルを送受信するスクリプト。

使い方:
  python send_file.py <port> write <ローカルファイル> [<Pico上のファイル名>]
  python send_file.py <port> read  <Pico上のファイル名> [<保存先ファイル名>]
  python send_file.py <port> list
  python send_file.py <port> delete <Pico上のファイル名>

例:
  python send_file.py /dev/ttyACM0 write hardware.json
  python send_file.py /dev/ttyACM0 list
  python send_file.py /dev/ttyACM0 read hardware.json out.json
  python send_file.py /dev/ttyACM0 delete hardware.json

プロトコル (write):
  "filename@minified_json\n" を送信し "OK\n" を受信する。
"""

import sys
import os
import time
import json
import serial

TIMEOUT_SEC = 10  # コマンド応答タイムアウト


def open_port(port: str) -> serial.Serial:
    return serial.Serial(port, 115200, timeout=TIMEOUT_SEC)


def readline_skip_sensor(ser: serial.Serial) -> str:
    """センサーデータ行をスキップしてコマンド応答を返す。"""
    while True:
        raw = ser.readline()
        if not raw:
            raise TimeoutError("応答タイムアウト")
        line = raw.decode("utf-8", errors="replace").strip()
        # センサー出力行は無視
        if "ADC0:" in line or "Gx:" in line or "Enc0:" in line:
            continue
        return line


def cmd_write(ser: serial.Serial, local_path: str, remote_name: str) -> None:
    with open(local_path, "r", encoding="utf-8") as f:
        text = f.read()
    try:
        content = json.dumps(json.loads(text), separators=(",", ":"), ensure_ascii=False)
    except json.JSONDecodeError:
        # JSON でなければ改行を除いてそのまま使用
        content = text.replace("\n", " ").replace("\r", "")

    message = f"{remote_name}@{content}\n"
    ser.write(message.encode("utf-8"))
    ser.flush()
    response = readline_skip_sensor(ser)
    if response == "OK":
        print(f"OK: {len(content)} バイトを '{remote_name}' に書き込みました")
    else:
        print(f"失敗: {response}", file=sys.stderr)
        sys.exit(1)


def cmd_read(ser: serial.Serial, remote_name: str, local_path: str) -> None:
    ser.write(f"READ:{remote_name}\n".encode())
    ser.flush()
    size_line = readline_skip_sensor(ser)
    if size_line.startswith("ERR"):
        print(f"失敗: {size_line}", file=sys.stderr)
        sys.exit(1)
    size = int(size_line)
    data = ser.read(size)
    if len(data) != size:
        print(f"失敗: データ受信不完全 ({len(data)}/{size} バイト)", file=sys.stderr)
        sys.exit(1)
    ok = readline_skip_sensor(ser)
    if ok == "OK":
        with open(local_path, "wb") as f:
            f.write(data)
        print(f"OK: {size} バイトを '{local_path}' に保存しました")
    else:
        print(f"失敗: {ok}", file=sys.stderr)
        sys.exit(1)


def cmd_list(ser: serial.Serial) -> None:
    ser.write(b"LIST\n")
    ser.flush()
    files = []
    while True:
        line = readline_skip_sensor(ser)
        if line == "OK":
            break
        if line.startswith("ERR"):
            print(f"失敗: {line}", file=sys.stderr)
            sys.exit(1)
        files.append(line)
    if files:
        print(f"{'ファイル名':<32} サイズ")
        print("-" * 44)
        for f in files:
            name, _, sz = f.partition(":")
            print(f"{name:<32} {sz} bytes")
    else:
        print("(ファイルなし)")


def cmd_delete(ser: serial.Serial, remote_name: str) -> None:
    ser.write(f"DELETE:{remote_name}\n".encode())
    ser.flush()
    response = readline_skip_sensor(ser)
    if response == "OK":
        print(f"OK: '{remote_name}' を削除しました")
    else:
        print(f"失敗: {response}", file=sys.stderr)
        sys.exit(1)


def main() -> None:
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    port    = sys.argv[1]
    command = sys.argv[2]

    with open_port(port) as ser:
        time.sleep(0.1)          # ポート安定待ち
        ser.reset_input_buffer()

        if command == "write":
            if len(sys.argv) < 4:
                print("Usage: write <local_file> [<remote_name>]", file=sys.stderr)
                sys.exit(1)
            local  = sys.argv[3]
            remote = sys.argv[4] if len(sys.argv) > 4 else os.path.basename(local)
            cmd_write(ser, local, remote)

        elif command == "read":
            if len(sys.argv) < 4:
                print("Usage: read <remote_name> [<local_file>]", file=sys.stderr)
                sys.exit(1)
            remote = sys.argv[3]
            local  = sys.argv[4] if len(sys.argv) > 4 else remote
            cmd_read(ser, remote, local)

        elif command == "list":
            cmd_list(ser)

        elif command == "delete":
            if len(sys.argv) < 4:
                print("Usage: delete <remote_name>", file=sys.stderr)
                sys.exit(1)
            cmd_delete(ser, sys.argv[3])

        else:
            print(f"不明なコマンド: {command}")
            print(__doc__)
            sys.exit(1)


if __name__ == "__main__":
    main()
