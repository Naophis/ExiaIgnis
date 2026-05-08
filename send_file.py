#!/usr/bin/env python3
"""
Pico へ USB CDC 経由でファイルを操作するスクリプト。

使い方:
  python send_file.py [<port>] <command> [args...]

ポートを省略すると Raspberry Pi Pico (VID=0x2E8A) を自動検出します。

コマンド一覧:
  write <ローカルファイル> [<Pico上のファイル名>]
      YAML/JSON をマイコンへ転送して LittleFS に書き込む。
      ファイル名省略時は拡張子を .json に変換した名前を使用。
      例: python send_file.py write hardware.yaml
          python send_file.py write profiles.yaml profiles.hf

  read <Pico上のファイル名> [<保存先ファイル名>]
      LittleFS のファイルをローカルに保存する。
      例: python send_file.py read hardware.json

  show <Pico上のファイル名>
      LittleFS のファイル内容をターミナルに表示する (保存なし)。
      例: python send_file.py show hardware.json

  list
      LittleFS に保存されているファイルの一覧を表示する。
      例: python send_file.py list

  delete <Pico上のファイル名>
      指定したファイルを LittleFS から削除する。
      例: python send_file.py delete hardware.json

  deleteall
      LittleFS を再フォーマットして全ファイルを消去する。
      例: python send_file.py deleteall
"""

import json
import os
import sys
import time

import serial
from serial.tools import list_ports

try:
    import yaml as _yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

TIMEOUT_SEC = 10
PICO_VID    = 0x2E8A  # Raspberry Pi
COMMANDS    = {"write", "read", "list", "delete", "deleteall", "show"}


def find_pico_port() -> str | None:
    """Raspberry Pi Pico (VID=0x2E8A) のポートを自動検出する。"""
    ports = sorted(p.device for p in list_ports.comports() if p.vid == PICO_VID)
    return ports[0] if ports else None


def resolve_port(argv: list[str]) -> tuple[str, list[str]]:
    """(port, remaining_args) を返す。先頭引数がコマンドなら自動検出。"""
    if argv and argv[0] not in COMMANDS:
        return argv[0], argv[1:]
    port = find_pico_port()
    if port is None:
        print("エラー: Pico が見つかりません。ポートを引数で指定してください。",
              file=sys.stderr)
        sys.exit(1)
    return port, argv


def open_port(port: str) -> serial.Serial:
    return serial.Serial(port, 115200, timeout=TIMEOUT_SEC)


def readline_skip_sensor(ser: serial.Serial) -> str:
    """センサーデータ行・デバイスデバッグ出力をスキップしてコマンド応答を返す。"""
    while True:
        raw = ser.readline()
        if not raw:
            raise TimeoutError("応答タイムアウト")
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        # センサーデータ行をスキップ
        if "ADC0:" in line or "Gx:" in line or "Enc0:" in line:
            continue
        # "[main]" "[config]" 等デバイス側デバッグ出力をスキップ
        if line.startswith("["):
            continue
        return line


def cmd_write(ser: serial.Serial, local_path: str, remote_name: str) -> None:
    if local_path.endswith((".yaml", ".yml")):
        if not _YAML_AVAILABLE:
            print("エラー: pyyaml が必要です  pip install pyyaml", file=sys.stderr)
            sys.exit(1)
        with open(local_path, "r", encoding="utf-8") as f:
            content = json.dumps(_yaml.safe_load(f), separators=(",", ":"),
                                 ensure_ascii=False)
    else:
        with open(local_path, "r", encoding="utf-8") as f:
            text = f.read()
        try:
            content = json.dumps(json.loads(text), separators=(",", ":"),
                                 ensure_ascii=False)
        except json.JSONDecodeError:
            content = text.replace("\n", " ").replace("\r", "")

    ser.write(f"{remote_name}@{content}\n".encode("utf-8"))
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


def cmd_show(ser: serial.Serial, remote_name: str) -> None:
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
    if ok != "OK":
        print(f"失敗: {ok}", file=sys.stderr)
        sys.exit(1)
    text = data.decode("utf-8", errors="replace")
    try:
        print(json.dumps(json.loads(text), indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(text)


def cmd_deleteall(ser: serial.Serial) -> None:
    ser.write(b"DELETEALL\n")
    ser.flush()
    response = readline_skip_sensor(ser)
    if response == "OK":
        print("OK: 全ファイルを消去しました")
    else:
        print(f"失敗: {response}", file=sys.stderr)
        sys.exit(1)


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    port, args = resolve_port(sys.argv[1:])
    if not args:
        print(__doc__)
        sys.exit(1)

    command = args[0]
    print(f"ポート: {port}")

    with open_port(port) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        if command == "write":
            if len(args) < 2:
                print("Usage: write <local_file> [<remote_name>]", file=sys.stderr)
                sys.exit(1)
            local = args[1]
            if len(args) > 2:
                remote = args[2]
            else:
                base = os.path.basename(local)
                remote = (os.path.splitext(base)[0] + ".json"
                          if base.endswith((".yaml", ".yml")) else base)
            cmd_write(ser, local, remote)

        elif command == "read":
            if len(args) < 2:
                print("Usage: read <remote_name> [<local_file>]", file=sys.stderr)
                sys.exit(1)
            remote = args[1]
            local  = args[2] if len(args) > 2 else remote
            cmd_read(ser, remote, local)

        elif command == "list":
            cmd_list(ser)

        elif command == "delete":
            if len(args) < 2:
                print("Usage: delete <remote_name>", file=sys.stderr)
                sys.exit(1)
            cmd_delete(ser, args[1])

        elif command == "show":
            if len(args) < 2:
                print("Usage: show <remote_name>", file=sys.stderr)
                sys.exit(1)
            cmd_show(ser, args[1])

        elif command == "deleteall":
            cmd_deleteall(ser)

        else:
            print(f"不明なコマンド: {command}\n", file=sys.stderr)
            print(__doc__)
            sys.exit(1)


if __name__ == "__main__":
    main()
