"""評価の共通部分。evaluate(vps) → times[case][k]"""
import json, os, subprocess, math, sys, time, hashlib
VERSION = 1
from multiprocessing import Pool
import ds
H = os.path.dirname(os.path.abspath(__file__))
os.makedirs(f"{H}/cache", exist_ok=True)
os.makedirs(f"{H}/fail", exist_ok=True)
EXECS = [1, 3, 5, 11, 16]
KEEP = lambda m: m["group"] in ("contest32", "contest16", "regional") or m["name"] == "20260927_155341"
MAZES = [m for m in ds.load_all.__wrapped__()] if hasattr(ds.load_all, "__wrapped__") else None
_files = None
def init():
    global MAZES, _files, CASES
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        MAZES = [m for m in ds.load_all() if KEEP(m)]
    _files = ds.profile_files()
    CASES = [(mi, ex) for mi in range(len(MAZES)) for ex in EXECS]
init()
FW = [1, 2, 3, 4, 5]
def _exec(ci, ids, table, want_path, exe="px_path", extra=None):
    mi, ex = CASES[ci]; m = MAZES[mi]
    inp = {"files": _files, "map": ds.fw_map(m), "goals": m["goals"], "exec": ex, "patterns": ids, "vp_table": table, "want_path": want_path}
    if extra: inp.update(extra)
    try:
        r = subprocess.run([exe if exe.startswith("/") else f"{H}/{exe}"], input=json.dumps(inp).encode(), capture_output=True, timeout=120 + 2 * len(ids))
        o = json.loads(r.stdout)
        if o.get("ok"): return o["results"], None
        return None, "error: " + str(o.get("error"))
    except subprocess.TimeoutExpired:
        return None, "timeout"
    except Exception as e:
        return None, f"rc={r.returncode} {type(e).__name__}"
def _run(args):
    ci, ids, table, want_path = args[:4]
    exe = args[4] if len(args) > 4 else "px_path"; extra = args[5] if len(args) > 5 else None
    key = hashlib.md5(json.dumps([CASES[ci][0], MAZES[CASES[ci][0]]["name"], CASES[ci][1], ids, table, want_path, exe, extra, VERSION]).encode()).hexdigest()
    cf = f"{H}/cache/{key}.json"
    if os.path.exists(cf):
        return json.load(open(cf))
    res, err = _exec(ci, ids, table, want_path, exe, extra)
    if res is None:
        if len(ids) == 1:
            json.dump({"case": CASES[ci], "maze": MAZES[CASES[ci][0]]["name"], "id": ids, "vp": table, "err": err, "extra": extra}, open(f"{H}/fail/{key}.json", "w"))
            res = [{"id": ids[0], "ok": False, "fail": err}]
        else:
            h = len(ids) // 2
            if table:
                a = _run((ci, [100 + k for k in range(h)], table[:h], want_path, exe, extra))
                b = _run((ci, [100 + k for k in range(len(ids) - h)], table[h:], want_path, exe, extra))
            else:
                a = _run((ci, ids[:h], [], want_path, exe, extra)); b = _run((ci, ids[h:], [], want_path, exe, extra))
            res = a + b
    json.dump(res, open(cf, "w"))
    return res
POOL = None
def evaluate(vps, fw_ids=(), chunk=64, want_path=False, exe="px_path", extra=None, cases=None):
    """vps: 値のリストのリスト。fw_ids: ファームのパターン番号。戻り値: res[case] = [result...](fw_ids → vps の順)"""
    global POOL
    if POOL is None: POOL = Pool(min(22, os.cpu_count()))
    jobs = []
    for ci in (range(len(CASES)) if cases is None else cases):
        if fw_ids: jobs.append((ci, list(fw_ids), [], want_path, exe, extra))
        for s in range(0, len(vps), chunk):
            part = vps[s:s + chunk]
            jobs.append((ci, [100 + k for k in range(len(part))], part, want_path, exe, extra))
    out = POOL.map(_run, jobs, chunksize=1)
    res = [[] for _ in CASES]
    for (ci, *_), r in zip(jobs, out): res[ci].extend(r)
    return res
def T(r): return r["time"] if r.get("ok") else math.inf
