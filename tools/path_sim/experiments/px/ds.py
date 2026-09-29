"""データセット: 迷路(重複除去)と共通の入力。"""
import json, os, glob, re, collections
import yaml
R = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..")); PT = f"{R}/tools/param_tuner"
H = os.path.dirname(os.path.abspath(__file__))

def profile_files():
    files = {}
    for f in ["system.yaml", "hardware.yaml"]:
        files[f.replace("yaml", "txt")] = json.dumps(yaml.safe_load(open(f"{PT}/profile/{f}")))
    for f in sorted(glob.glob(f"{PT}/profile/hf/*.yaml")):
        files[os.path.basename(f).replace("yaml", "hf")] = json.dumps(yaml.safe_load(open(f)))
    return files

def parse_text(t):
    return [int(x) & 15 for x in re.split(r"[\s,]+", t) if x]

def wall(w, n, x, y, d):
    if x < 0 or y < 0 or x >= n or y >= n: return True
    b = {"N": 1, "E": 2, "W": 4, "S": 8}[d]
    if w[x * n + y] & b: return True
    nx, ny, bb = {"N": (x, y + 1, 8), "S": (x, y - 1, 1), "E": (x + 1, y, 4), "W": (x - 1, y, 2)}[d]
    if nx < 0 or ny < 0 or nx >= n or ny >= n: return True
    return (w[nx * n + ny] & bb) != 0

def reach(w, n):
    seen = {(0, 0)}; q = collections.deque([(0, 0)])
    while q:
        x, y = q.popleft()
        for d, dx, dy in (("N", 0, 1), ("E", 1, 0), ("S", 0, -1), ("W", -1, 0)):
            if wall(w, n, x, y, d): continue
            p = (x + dx, y + dy)
            if p not in seen: seen.add(p); q.append(p)
    return seen

def detect_goal(w, n):
    best = None; rc = reach(w, n)
    for k in (3, 2):
        for x0 in range(n - k + 1):
            for y0 in range(n - k + 1):
                if x0 == 0 and y0 == 0: continue
                ok = all(not (x < x0 + k - 1 and wall(w, n, x, y, "E")) and not (y < y0 + k - 1 and wall(w, n, x, y, "N")) for x in range(x0, x0 + k) for y in range(y0, y0 + k))
                if not ok: continue
                op = sum((not wall(w, n, x0 + i, y0, "S")) + (not wall(w, n, x0 + i, y0 + k - 1, "N")) + (not wall(w, n, x0, y0 + i, "W")) + (not wall(w, n, x0 + k - 1, y0 + i, "E")) for i in range(k))
                if 1 <= op <= 2:
                    key = (0 if (x0, y0) in rc else 1, op, -k, x0, y0)
                    if best is None or key < best[0]: best = (key, [[x, y] for x in range(x0, x0 + k) for y in range(y0, y0 + k)])
    return best[1] if best else None

def edges(w, n):
    """壁のある辺の集合(両側のどちらかが壁なら壁)"""
    s = set()
    for x in range(n):
        for y in range(n):
            if wall(w, n, x, y, "N"): s.add((x, y, "N"))
            if wall(w, n, x, y, "E"): s.add((x, y, "E"))
    return s

def load_all():
    """(name, group, n, walls(.maze の並び), goals) の一覧。重複(壁の違いが 3 枚以下)は先のものを残す。"""
    src = []
    for f in sorted(glob.glob(f"{PT}/maze_data/32MM*.yaml")):
        d = yaml.safe_load(open(f))["maze_data"]; src.append((os.path.basename(f)[:-5], "contest32", d["wall"], d["goal"]))
    for f in sorted(glob.glob(f"{PT}/maze_data/japan*.yaml")):
        d = yaml.safe_load(open(f))["maze_data"]
        src.append((os.path.basename(f)[:-5], "contest32" if d["maze_size"] == 32 else "contest16", d["wall"], d["goal"]))
    for f in ["japan2025_final.maze"]:
        src.append((f[:-5], "contest32", parse_text(open(f"{PT}/maze_logs/{f}").read()), None))
    for f in ["higashi2024.yaml", "kansai2025.yaml", "maze.yaml"]:
        src.append((f[:-5], "regional", parse_text(open(f"{PT}/profile/{f}").read()), None))
    for f in sorted(os.listdir(f"{PT}/maze_logs")):
        if f.endswith(".maze") and f != "japan2025_final.maze":
            src.append((f[:-5], "log", parse_text(open(f"{PT}/maze_logs/{f}").read()), None))
    out = []; seen = []
    for name, group, w, g in src:
        w = [x & 15 for x in w]; n = int(round(len(w) ** 0.5))
        if n * n != len(w): print("skip(size)", name, len(w)); continue
        e = edges(w, n)
        dup = next((nm for nm, n2, e2 in seen if n2 == n and len(e ^ e2) <= 3), None)
        if dup: print(f"dup {name} = {dup} (違い {len(e ^ next(e2 for nm, n2, e2 in seen if nm == dup))} 枚)"); continue
        goals = g or detect_goal(w, n)
        if not goals: print("skip(no goal)", name); continue
        rc = reach(w, n)
        if not any(tuple(p) in rc for p in goals): print("skip(unreachable)", name); continue
        seen.append((name, n, e)); out.append({"name": name, "group": group, "n": n, "walls": w, "goals": goals, "reach": len(rc)})
    return out

def fw_map(m, flags=0xF0):
    n = m["n"]; w = m["walls"]; mp = [0] * (n * n)
    for x in range(n):
        for y in range(n): mp[x + y * n] = (w[x * n + y] & 15) | flags
    return mp

if __name__ == "__main__":
    ms = load_all()
    for m in ms: print(f'{m["group"]:10s} {m["name"]:32s} {m["n"]}x{m["n"]} goal {len(m["goals"])} @{m["goals"][0]} 到達可能 {m["reach"]}')
    print(len(ms), "mazes", collections.Counter(m["group"] for m in ms))
