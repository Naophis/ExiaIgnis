#!/usr/bin/env python3
"""
tools/param_tuner/profile の YAML ファイルを選択して Pico に転送するスクリプト。

使い方:
  python send_profile.py [port] [subdir]

引数:
  port   : シリアルポート (省略時は Raspberry Pi Pico を自動検出)
  subdir : profile/ 以下のサブディレクトリ名 (省略時は直下)

例:
  python send_profile.py                  # 自動検出, profile/ 直下
  python send_profile.py hf               # 自動検出, profile/hf/
  python send_profile.py /dev/ttyACM1 hf  # ポート指定

Pico 上のファイル名は自動で .json に変換します (例: hardware.yaml → hardware.json)。
"""

import json
import os
import sys
import time

import serial
from serial.tools import list_ports
import yaml

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROFILE_ROOT = os.path.join(SCRIPT_DIR, "tools", "param_tuner", "profile")
TIMEOUT_SEC  = 10
PICO_VID     = 0x2E8A


def find_pico_port() -> str | None:
    ports = sorted(p.device for p in list_ports.comports() if p.vid == PICO_VID)
    return ports[0] if ports else None

SKIP_PREFIXES = (
    "ADC0:", "Gx:", "Enc0:", "[timing]", "[sensor]", "[motion]",
    "[power]", "[planning]", "[control]", "[sys]", "===", "[main]", "[config]",
)


def open_port(port: str) -> serial.Serial:
    return serial.Serial(port, 115200, timeout=TIMEOUT_SEC)


def readline_skip_sensor(ser: serial.Serial) -> str:
    while True:
        raw = ser.readline()
        if not raw:
            raise TimeoutError("応答タイムアウト")
        line = raw.decode("utf-8", errors="replace").strip()
        if any(line.startswith(p) for p in SKIP_PREFIXES):
            continue
        return line


def yaml_to_json(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return json.dumps(data, separators=(",", ":"), ensure_ascii=False)


def send_file(ser: serial.Serial, remote_name: str, content: str) -> bool:
    ser.write(f"{remote_name}@{content}\n".encode("utf-8"))
    ser.flush()
    return readline_skip_sensor(ser) == "OK"


def list_yaml_files(profile_dir: str) -> list[tuple[str, str]]:
    entries = []
    for name in sorted(os.listdir(profile_dir)):
        path = os.path.join(profile_dir, name)
        if os.path.isfile(path) and name.endswith((".yaml", ".yml")):
            entries.append((name, path))
    return entries


def main() -> None:
    # 引数解析: /dev/ で始まる場合はポート指定、それ以外はサブディレクトリ
    args = sys.argv[1:]
    port   = None
    subdir = ""
    for a in args:
        if a.startswith("/dev/") or a.startswith("COM"):
            port = a
        else:
            subdir = a

    if port is None:
        port = find_pico_port()
        if port is None:
            print("エラー: Pico が見つかりません。ポートを引数で指定してください。",
                  file=sys.stderr)
            sys.exit(1)

    print(f"ポート: {port}")
    profile_dir = os.path.join(PROFILE_ROOT, subdir) if subdir else PROFILE_ROOT

    files = list_yaml_files(profile_dir)
    if not files:
        print(f"YAMLファイルが見つかりません: {profile_dir}")
        sys.exit(1)

    rel = os.path.relpath(profile_dir, SCRIPT_DIR)
    print(f"プロファイル一覧 [{rel}]:")
    for i, (name, _) in enumerate(files):
        print(f"  {i + 1:2}. {name}")

    print("\n送信するファイルの番号を入力してください")
    print("(例: 1,3  または  all)")
    try:
        sel = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\nキャンセル")
        sys.exit(0)

    if sel.lower() == "all":
        indices = list(range(len(files)))
    else:
        indices = []
        for part in sel.split(","):
            part = part.strip()
            if part.isdigit() and 1 <= int(part) <= len(files):
                indices.append(int(part) - 1)

    if not indices:
        print("選択なし")
        sys.exit(0)

    # YAML → JSON 変換
    converted: list[tuple[str, str]] = []
    for name, path in [files[i] for i in indices]:
        remote = os.path.splitext(name)[0] + ".json"
        try:
            content = yaml_to_json(path)
            converted.append((remote, content))
            print(f"  変換 OK: {name} → {remote}  ({len(content)} bytes)")
        except Exception as e:
            print(f"  変換エラー: {name}: {e}")

    if not converted:
        print("送信するファイルがありません")
        sys.exit(1)

    print(f"\nポート {port} に送信します...")
    with open_port(port) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()
        for remote, content in converted:
            if send_file(ser, remote, content):
                print(f"  OK: {remote}")
            else:
                print(f"  失敗: {remote}")


if __name__ == "__main__":
    main()
