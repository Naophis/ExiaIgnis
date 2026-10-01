"""run.sh 用: 標準入力の PAIR(Python 版)と CPP(C++ 版)の組を突き合わせる"""
import sys
py, cpp = [], []
for line in sys.stdin:
    f = line.split()
    if f[0] not in ("PAIR", "CPP"):
        continue
    (py if f[0] == "PAIR" else cpp).append((f[1], float(f[2]), float(f[3]), float(f[4]), int(f[5])))
print(f"pairs: python {len(py)} / c++ {len(cpp)}")
bad = 0
for a, b in zip(py, cpp):
    if a[0] != b[0] or abs(a[2] - b[2]) > 0.001 or abs(a[1] - b[1]) > 0.01 or abs(a[3] - b[3]) > 0.001 or a[4] != b[4]:
        bad += 1
        print("  mismatch", a, b)
if len(py) != len(cpp):
    bad += 1
print("一致" if bad == 0 else f"不一致 {bad}")
