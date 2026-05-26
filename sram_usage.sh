#!/bin/bash
# SRAM 配置量確認: .time_critical.* セクションのサイズ一覧
python3 << 'EOF'
import re
from collections import defaultdict

sections = []
with open('build/ExiaIgnis.elf.map') as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i].rstrip()
    m = re.match(r'\s+(\.time_critical\.\S+)\s*$', line)
    if m:
        section_name = m.group(1)
        if i+1 < len(lines):
            m2 = re.match(r'\s+(0x[0-9a-f]+)\s+(0x[0-9a-f]+)', lines[i+1])
            if m2:
                addr = int(m2.group(1), 16)
                size = int(m2.group(2), 16)
                if addr >= 0x20000000:
                    sections.append((section_name, addr, size))
    i += 1

groups = defaultdict(int)
for name, addr, size in sections:
    parts = name.split('.')
    group = '.'.join(parts[:3]) if len(parts) >= 3 else name
    groups[group] += size

total = sum(groups.values())
print(f"{'Section':<45} {'Bytes':>7}  {'KB':>6}")
print('-' * 62)
for group, size in sorted(groups.items(), key=lambda x: -x[1]):
    print(f"{group:<45} {size:>7}  {size/1024:>5.1f}KB")
print('-' * 62)
print(f"{'TOTAL':<45} {total:>7}  {total/1024:>5.1f}KB")
print(f"{'SRAM remaining (520KB)':<45} {520*1024-total:>7}  {(520*1024-total)/1024:>5.1f}KB")
EOF
