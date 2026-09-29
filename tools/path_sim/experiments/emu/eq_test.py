#!/usr/bin/env python3
"""eq_test.cpp を変更前の写し(../frozen2)といまのファームでビルドし、乱数の状態での結果を比べる。
    python3 eq_test.py [状態の数]
境界外アクセスの検査(AddressSanitizer + _GLIBCXX_ASSERTIONS)も付けて回す。"""
import os, subprocess, sys
H = os.path.dirname(os.path.abspath(__file__))
R = os.path.abspath(os.path.join(H, "..", "..", "..", ".."))
Z = os.path.join(H, "..", "frozen2")
n = sys.argv[1] if len(sys.argv) > 1 else "3000"
common = f"-I{H} -I{R}/tools/path_sim/stub -I{R} -I{R}/include -I{R}/include/action"
fl = "-std=gnu++20 -O1 -g -w -fsanitize=address,undefined -fno-sanitize-recover=undefined -D_GLIBCXX_ASSERTIONS"
subprocess.check_call(f"g++ {fl} -I{Z} {common} {H}/eq_test.cpp {Z}/logic.cpp -o {H}/eq_old", shell=True)
subprocess.check_call(f"g++ {fl} -I{R}/include/search {common} {H}/eq_test.cpp {R}/src/search/logic.cpp -o {H}/eq_fw", shell=True)
out = {}
for k in ("old", "fw"):
    r = subprocess.run(["timeout", "900", f"{H}/eq_{k}", n], capture_output=True, text=True)
    if r.returncode != 0:
        print(f"eq_{k}: 異常終了 rc={r.returncode}\n{r.stderr[-3000:]}")
    out[k] = r.stdout.strip().split("\n")
bad = [(a, b) for a, b in zip(out["old"], out["fw"]) if a != b]
inc = [x for x in bad if x[0].split()[2] == "0"]
walked = sum(int(x.split()[4]) for x in out["old"])
print(f"{len(out['old'])} / {len(out['fw'])} 状態: 違う {len(bad)}(うち、壁の食い違い・外周抜けのある地図 {len(inc)})  経路をたどれた表 {walked} / {3 * len(out['old'])}")
for a, b in bad[:10]:
    print("  変更前", a, " いま", b)
sys.exit(1 if bad or len(out["old"]) != len(out["fw"]) else 0)
