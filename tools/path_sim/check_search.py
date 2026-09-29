#!/usr/bin/env python3
"""探索(search_sim = ファームの adachi.cpp / logic.cpp そのもの)の結果が、基準と同じかを確かめる。
探索まわりのソースを、結果を変えないつもりで直したとき(高速化など)に回す。

    python3 check_search.py            # 基準(search_ref.json)と比べる
    python3 check_search.py --update   # いまの結果を基準にする(探索の動きを意図して変えたとき)
    python3 check_search.py --update --sim experiments/emu/search_sim_old
                                       # 変更前のソースで作った探索シミュレータを基準にする

比べるのは search_sim の出力全体(判断ごとの位置・動き・時刻・サブゴールの区画・地図の変化・候補の経路)。
走行パラメータ(profile の yaml)は、基準を取ったときのものを search_ref.json に一緒に残してあり、
比べるときもそれを使う(Param Console でパラメータを変えても基準は使える)。
迷路(maze_data)が基準を取ったときと違うと比べられないので、その場合は理由を出す。
"""
import argparse, glob, hashlib, json, os, re, subprocess, sys
from multiprocessing import Pool

import yaml

H = os.path.dirname(os.path.abspath(__file__))
PT = os.path.join(H, "..", "param_tuner")
REF = os.path.join(H, "search_ref.json")


def profile_files():
    files = {}
    for f in ["system.yaml", "hardware.yaml"]:
        files[f.replace("yaml", "txt")] = json.dumps(yaml.safe_load(open(f"{PT}/profile/{f}")))
    for f in sorted(glob.glob(f"{PT}/profile/hf/*.yaml")):
        files[os.path.basename(f).replace("yaml", "hf")] = json.dumps(yaml.safe_load(open(f)))
    return files


def mazes():
    out = []
    for f in sorted(glob.glob(f"{PT}/maze_data/*.yaml")):
        d = yaml.safe_load(open(f))["maze_data"]
        n = d["maze_size"]
        w = d["wall"]
        truth = [w[x * n + y] & 15 for y in range(n) for x in range(n)]
        out.append((os.path.basename(f)[:-5], truth, d["goal"]))
    return out


FILES = profile_files()  # いまの走行パラメータ(--update のとき、ほかのスクリプトから使うとき)
MAZES = mazes()
SIM = f"{H}/build/search_sim"
RUN_FILES = FILES


def sha(obj):
    return hashlib.sha1(json.dumps(obj, sort_keys=True).encode()).hexdigest()


def run(a):
    mi, sim, files = a
    name, truth, goals = MAZES[mi]
    inp = {"files": files, "truth": truth, "goals": goals}
    maze = sha({"truth": truth, "goals": goals})
    r = subprocess.run([sim], input=json.dumps(inp).encode(), capture_output=True, timeout=1200)
    try:
        o = json.loads(r.stdout)
    except Exception:
        return name, {"input": maze, "error": f"rc={r.returncode}"}
    return name, {
        "input": maze, "output": sha(o), "total_time": o.get("total_time"), "steps": len(o.get("steps", [])),
        "end_reason": o.get("end_reason"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--update", action="store_true", help="いまの結果を基準にする")
    ap.add_argument("--sim", default=SIM, help="探索シミュレータ(既定は build/search_sim = いまのファーム)")
    args = ap.parse_args()
    subprocess.check_call(["make", "-s"], cwd=H)
    sim = os.path.abspath(args.sim)
    if args.update:
        files = FILES
    else:
        if not os.path.exists(REF):
            sys.exit("基準がありません(--update で作る)")
        saved = json.load(open(REF))
        files, ref = saved["files"], saved["mazes"]
    with Pool() as p:
        now = dict(p.map(run, [(mi, sim, files) for mi in range(len(MAZES))]))
    if args.update:
        err = [n for n, v in now.items() if "error" in v]
        if err:
            sys.exit(f"探索シミュレータが失敗したので基準にしません: {err}")
        json.dump({"files": files, "mazes": now}, open(REF, "w"), indent=1, sort_keys=True)
        print(f"基準を取り直しました: {len(now)} 迷路  探索時間の合計 {sum(v.get('total_time') or 0 for v in now.values()):.0f} s")
        return
    bad = stale = 0
    for name, v in sorted(now.items()):
        r = ref.get(name)
        if r is None or r.get("input") != v.get("input"):
            stale += 1
            print(f"   {name}: 迷路が基準と違う(または基準に無い)ので比べられません")
        elif r.get("output") != v.get("output"):
            bad += 1
            print(f"NG {name}: 基準 {r.get('total_time')} s / {r.get('steps')} 回、いま {v.get('total_time')} s / {v.get('steps')} 回 {v.get('error', '')}")
    print(f"{len(now)} 迷路: 同じ {len(now) - bad - stale} / 違う {bad} / 比べられない {stale}")
    if stale:
        print("  探索の動きは変えていないはずなら、変更前のソースで --update してから比べ直してください")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
