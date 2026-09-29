"""ランダム探索: St1=1 に正規化した 5 個の比を振る。結果は rs_<tag>.pkl"""
import sys, math, random, pickle, time, zlib
import numpy as np
import ev
def sample(rng):
    s2 = rng.uniform(0.05, 1.0); s3 = rng.uniform(0.05, 1.0)
    d1 = math.exp(rng.uniform(math.log(0.25), math.log(3.0)))
    d2 = rng.uniform(0.05, 1.0); d3 = rng.uniform(0.05, 1.0)
    return [1.0, s2, s2 * s3, d1, d1 * d2, d1 * d2 * d3]
FWV = {1: [7, 2, 1, 7 * 1.41421356 / 2, 7 * 1.41421356 / 2 * 3 / 5, 7 * 1.41421356 / 2 * 2 / 5],
       2: [1, 1, 1, 1, 1, 1],
       3: [180, 180, 180, 180 * 1.41421356 / 2, 180 * 1.41421356 / 2, 180 * 1.41421356 / 2],
       4: [7, 3, 2, 7 * 1.41421356 / 2, 7 * 1.41421356 / 2 * 3 / 5, 7 * 1.41421356 / 2 * 2 / 5],
       5: [0.5, 1 / 3, 0.2, 1.41421356, 1.41421356, 1.41421356]}
def pack(res, n):
    C = len(ev.CASES)
    t = np.full((n, C), np.inf); ms = np.zeros((n, C)); base = np.full((n, C), np.inf); sig = np.zeros((n, C), dtype=np.int64); other = np.zeros((n, C))
    for ci in range(C):
        for k, r in enumerate(res[ci]):
            if r.get("ok"):
                t[k, ci] = r["time"]; base[k, ci] = r["base_time"]; sig[k, ci] = zlib.crc32(r["sig"].encode())
            ms[k, ci] = r.get("ms", 0); other[k, ci] = r.get("other", 0)
    return {"t": t, "ms": ms, "base": base, "sig": sig, "other": other}
if __name__ == "__main__":
    n = int(sys.argv[1]); seed = int(sys.argv[2]); tag = sys.argv[3]
    rng = random.Random(seed)
    vps = [sample(rng) for _ in range(n)]
    t0 = time.time()
    res = ev.evaluate(vps, fw_ids=ev.FW)
    d = pack(res, n + 5)
    d["vps"] = [FWV[i] for i in ev.FW] + vps
    d["cases"] = [(ev.MAZES[mi]["name"], ex) for mi, ex in ev.CASES]
    pickle.dump(d, open(f"rs_{tag}.pkl", "wb"))
    print("done", n, "patterns", round(time.time() - t0), "s")
