"""Cortex-M33 向けの bench_*.elf を Unicorn で動かし、bench_begin(id) 〜 bench_end(id) の間の命令数を数える。
   --prof: 関数ごとの内訳(その関数の中で実行した命令数)"""
import sys, struct, subprocess, bisect, argparse, collections
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "pylib"))  # pip install --target pylib unicorn
from unicorn import *
from unicorn.arm_const import *

def symbols(elf):
    out = subprocess.check_output(["arm-none-eabi-nm", "-n", "-C", elf]).decode()
    syms = []
    for l in out.split("\n"):
        p = l.split(None, 2)
        if len(p) == 3 and p[1] in "TtWw":
            syms.append((int(p[0], 16) & ~1, p[2]))
    return syms

def run(elf, snap, first=0, count=20, stride=10, prof=False):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
    uc.mem_map(0x20000000, 12 << 20)
    uc.mem_map(0x21000000, 4 << 20)
    data = open(elf, "rb").read()
    phoff = struct.unpack_from("<I", data, 28)[0]; phentsize, phnum = struct.unpack_from("<HH", data, 42)
    for i in range(phnum):
        t, off, va, pa, fsz, msz = struct.unpack_from("<IIIIII", data, phoff + i * phentsize)
        if t == 1 and fsz: uc.mem_write(va, data[off:off + fsz])
    uc.mem_write(0x21000000, open(snap, "rb").read())
    syms = symbols(elf)
    addr = {n: a for a, n in syms}
    a_begin = [a for a, n in syms if n.startswith("bench_begin")][0]
    a_end = [a for a, n in syms if n.startswith("bench_end")][0]
    a_res = int([l.split()[0] for l in subprocess.check_output(["arm-none-eabi-nm", elf]).decode().split("\n") if l.endswith(" g_result")][0], 16)
    starts = [a for a, n in syms]; names = [n for a, n in syms]
    state = {"id": 0, "n": 0}
    windows = collections.defaultdict(list)
    profile = collections.defaultdict(lambda: collections.Counter())
    # ブロックごとに命令数を数える(Thumb: 先頭の 5 ビットが 11101 / 11110 / 11111 なら 32 ビット命令)
    ninstr = {}
    def count_block(address, size):
        c = ninstr.get(address)
        if c is None:
            raw = bytes(uc.mem_read(address, size)); i = 0; c = 0
            while i < size:
                hw = raw[i] | (raw[i + 1] << 8)
                i += 4 if (hw >> 11) in (0b11101, 0b11110, 0b11111) else 2
                c += 1
            ninstr[address] = c
        return c
    def on_block(uc, address, size, ud):
        if address == a_begin:
            state["id"] = uc.reg_read(UC_ARM_REG_R0); state["n"] = 0; state["on"] = True
            return
        if address == a_end:
            if state.get("on"):
                windows[state["id"]].append(state["n"])
            state["on"] = False
            return
        if state.get("on"):
            c = count_block(address, size)
            state["n"] += c
            if prof:
                i = bisect.bisect_right(starts, address) - 1
                profile[state["id"]][names[i]] += c
    uc.hook_add(UC_HOOK_BLOCK, on_block)
    uc.reg_write(UC_ARM_REG_SP, 0x20000000 + (12 << 20) - 64)
    uc.reg_write(UC_ARM_REG_R0, first); uc.reg_write(UC_ARM_REG_R1, count); uc.reg_write(UC_ARM_REG_R2, stride)
    uc.reg_write(UC_ARM_REG_LR, 0x20B00001)
    uc.emu_start(addr["bench_main"] | 1, 0x20B00000)
    done, mismatch = struct.unpack("<ii", uc.mem_read(a_res, 8))
    return windows, profile, done, mismatch

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("elf"); ap.add_argument("snap")
    ap.add_argument("--first", type=int, default=0); ap.add_argument("--count", type=int, default=20); ap.add_argument("--stride", type=int, default=15)
    ap.add_argument("--prof", action="store_true")
    a = ap.parse_args()
    w, p, done, mismatch = run(a.elf, a.snap, a.first, a.count, a.stride, a.prof)
    import statistics
    print(f"{a.elf} {a.snap}: {done} 状態  元の結果との不一致 {mismatch}")
    for wid, label in ((1, "update() の中身(表づくり + 経路)"), (3, "update() の中身(地図が前回と同じときの近道)"), (2, "exec() の歩数マップ")):
        v = w[wid]
        if not v: continue
        print(f"  {label}: 命令数 平均 {statistics.mean(v):,.0f} 最小 {min(v):,} 最大 {max(v):,}")
        if a.prof:
            tot = sum(p[wid].values())
            for name, c in p[wid].most_common(14):
                print(f"      {c / tot * 100:5.1f} %  {c / len(v):>10,.0f}  {name[:90]}")
