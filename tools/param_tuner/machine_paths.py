#!/usr/bin/env python3
"""機体ごとのパラメータの場所を決める(CLI・検査スクリプト用)。

機体(個体)ごとのパラメータは tools/param_tuner/machines/<機体>/profile/ にあり、
登録簿は tools/param_tuner/machines.yaml(Param Console のヘッダーの「機体設定」が書き換える)。

どの機体を使うかは、上から順に:
  1. 引数で渡した名前
  2. 環境変数 EXIA_MACHINE
  3. (detect=True のとき)つないでいる基板の USB シリアル番号が登録されている機体
  4. machines.yaml の default

シェルから:
  python3 tools/param_tuner/machine_paths.py            # 既定の機体の profile の場所を出す
  python3 tools/param_tuner/machine_paths.py 1st        # 機体を指定
  python3 tools/param_tuner/machine_paths.py --list     # 機体の一覧
  EXIA_MACHINE=1st python3 tools/path_sim/check_time_path.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MACHINES_YAML = os.path.join(HERE, "machines.yaml")
MACHINES_DIR = os.path.join(HERE, "machines")
PICO_VID = 0x2E8A


def registry() -> dict:
    import yaml

    if not os.path.exists(MACHINES_YAML):
        sys.exit(f"エラー: {MACHINES_YAML} がありません(Param Console で機体を追加してください)")
    with open(MACHINES_YAML, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    machines = [m for m in (doc.get("machines") or []) if isinstance(m, dict) and m.get("id")]
    ids = [str(m["id"]) for m in machines]
    default = doc.get("default") if doc.get("default") in ids else (ids[0] if ids else None)
    return {"default": default, "machines": machines, "ids": ids}


def machine_for_serial(serial: str | None) -> str | None:
    if not serial:
        return None
    for m in registry()["machines"]:
        if serial in [str(s) for s in (m.get("serials") or [])]:
            return str(m["id"])
    return None


def connected_board() -> tuple[str | None, str | None]:
    """つないでいる Pico の (USB シリアル番号, 登録されている機体)。無ければ (None, None)。"""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None, None
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if p.vid == PICO_VID and p.serial_number:
            return p.serial_number, machine_for_serial(p.serial_number)
    return None, None


def resolve_machine(name: str | None = None, detect: bool = False) -> str:
    reg = registry()
    if not reg["ids"]:
        sys.exit("エラー: machines.yaml に機体がありません(Param Console で機体を追加してください)")
    explicit = name or os.environ.get("EXIA_MACHINE")
    if explicit:
        if explicit not in reg["ids"]:
            sys.exit(f"エラー: 機体 \"{explicit}\" は登録されていません(登録済み: {', '.join(reg['ids'])})")
        return explicit
    if detect:
        serial, machine = connected_board()
        if machine:
            return machine
        if serial and len(reg["ids"]) > 1:
            # 別の機体のパラメータを黙って送らない
            sys.exit(
                f"エラー: つないでいる基板 ({serial}) はどの機体にも登録されていません。\n"
                f"  機体を指定する(EXIA_MACHINE=<機体> か引数)か、Param Console で基板を登録してください。"
                f"\n  登録済みの機体: {', '.join(reg['ids'])}"
            )
    return reg["default"]


def profile_dir(name: str | None = None, detect: bool = False, quiet: bool = False) -> str:
    """その機体の profile ディレクトリ(以前の tools/param_tuner/profile に当たる)。"""
    machine = resolve_machine(name, detect)
    path = os.path.join(MACHINES_DIR, machine, "profile")
    if not quiet:
        print(f"機体: {machine} ({os.path.relpath(path)})", file=sys.stderr)
    return path


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--list":
        reg = registry()
        for m in reg["machines"]:
            mark = "*" if m["id"] == reg["default"] else " "
            print(f"{mark} {m['id']}  serials: {', '.join(str(s) for s in (m.get('serials') or [])) or '-'}")
    else:
        print(profile_dir(args[0] if args else None, quiet=True))
