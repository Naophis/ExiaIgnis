"""関数の中のどの行に命令数が集まっているか(-g 付きの ELF、addr2line で行へ直す)"""
import sys, os, struct, subprocess, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "pylib"))
from unicorn import *
from unicorn.arm_const import *
import emu
elf, snap, wid = sys.argv[1], sys.argv[2], int(sys.argv[3])
uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
uc.mem_map(0x20000000, 12 << 20); uc.mem_map(0x21000000, 4 << 20)
data = open(elf, "rb").read()
phoff = struct.unpack_from("<I", data, 28)[0]; phentsize, phnum = struct.unpack_from("<HH", data, 42)
for i in range(phnum):
    t, off, va, pa, fsz, msz = struct.unpack_from("<IIIIII", data, phoff + i * phentsize)
    if t == 1 and fsz: uc.mem_write(va, data[off:off + fsz])
uc.mem_write(0x21000000, open(snap, "rb").read())
syms = emu.symbols(elf); addr = {n: a for a, n in syms}
a_begin = [a for a, n in syms if n.startswith("bench_begin")][0]; a_end = [a for a, n in syms if n.startswith("bench_end")][0]
st = {"on": False, "n": 0}
blocks = collections.Counter(); sizes = {}
def on_block(uc, address, size, ud):
    if address == a_begin: st["on"] = uc.reg_read(UC_ARM_REG_R0) == wid; st["n"] += st["on"]; return
    if address == a_end: st["on"] = False; return
    if st["on"]: blocks[address] += 1; sizes[address] = size
uc.hook_add(UC_HOOK_BLOCK, on_block)
uc.reg_write(UC_ARM_REG_SP, 0x20000000 + (12 << 20) - 64)
uc.reg_write(UC_ARM_REG_R0, 0); uc.reg_write(UC_ARM_REG_R1, 12); uc.reg_write(UC_ARM_REG_R2, 40)
uc.reg_write(UC_ARM_REG_LR, 0x20B00001)
uc.emu_start(addr["bench_main"] | 1, 0x20B00000)
# 命令ごとのアドレスへ展開
ins = collections.Counter()
for a, c in blocks.items():
    raw = bytes(uc.mem_read(a, sizes[a])); i = 0
    while i < sizes[a]:
        hw = raw[i] | (raw[i + 1] << 8)
        ins[a + i] += c
        i += 4 if (hw >> 11) in (0b11101, 0b11110, 0b11111) else 2
addrs = sorted(ins)
out = subprocess.run(["arm-none-eabi-addr2line", "-e", elf, "-f", "-C", "-i"] + [hex(a) for a in addrs], capture_output=True, text=True).stdout.split("\n")
# -i だと 1 アドレスに複数行(インライン展開)。最初の(いちばん内側の)行を使う
lines = collections.Counter(); funcs = collections.Counter()
res = subprocess.run(["arm-none-eabi-addr2line", "-e", elf, "-f", "-C"] + [hex(a) for a in addrs], capture_output=True, text=True).stdout.split("\n")
for k, a in enumerate(addrs):
    fn = res[2 * k]; loc = res[2 * k + 1]
    loc = loc.split(" (")[0]
    f = os.path.basename(loc.split(":")[0]); ln = loc.split(":")[-1]
    lines[f"{f}:{ln}"] += ins[a]
tot = sum(ins.values()); n = st["n"]
print(f"合計 {tot / n:,.0f} 命令 / 回({n} 回)")
for k, c in lines.most_common(int(sys.argv[4]) if len(sys.argv) > 4 else 30):
    print(f"  {c / tot * 100:5.1f} %  {c / n:>9,.0f}  {k}")
