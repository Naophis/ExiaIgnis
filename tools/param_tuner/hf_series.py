#!/usr/bin/env python3
"""WALL_OFF中の高頻度(4kHz相当)壁切れサンプル系列をログCSVから復元して出す。

firmware側(sensing_task.cpp, 2026-09-15)は1kHzのログ行に「前1msのhfサンプル
(最大4点)」を hf_d0..3(距離[mm]) / hf_x0..3(行取得時のglobal_pos.distからの
相対位置[mm]) として載せ、絶対位置復元用に gpos_hi(100mm単位)/gpos_lo を出す。
このスクリプトはそれを1本の時系列に展開し、

  1. WALL_OFF区間のhfサンプルを位置順に並べた表(x_abs, d)
  2. hf_edge_rel(検出後の進行距離)から逆算した壁切れ位置 x_edge
  3. 1msあたりのサンプル数(hf_seqの連続性とhf_cntから、欠落の有無)

を表示する。--plot でmatplotlibの図(距離 vs 位置、min-holdと検出点)も出す。

使い方:
    python3 tools/param_tuner/hf_series.py logs/20260915_xxxxxx.csv
    python3 tools/param_tuner/hf_series.py --plot logs/2026*.csv
"""
import sys
import numpy as np
import pandas as pd

REQ = ["motion_state", "hf_cnt", "hf_seq", "gpos_hi", "gpos_lo", "hf_edge_rel",
       "hf_d0", "hf_d1", "hf_d2", "hf_d3", "hf_x0", "hf_x1", "hf_x2", "hf_x3"]
WALL_OFF = 6


def expand(df: pd.DataFrame) -> pd.DataFrame:
    """1kHz行 → hfサンプル行(位置順)。"""
    gpos = df["gpos_hi"].values * 100.0 + df["gpos_lo"].values
    rows = []
    for i, r in df.iterrows():
        n = int(r["hf_cnt"])
        for k in range(min(n, 4)):
            rows.append(dict(
                row=int(r["index"]) if "index" in df else i,
                slot=k,
                x_abs=gpos[i] + float(r[f"hf_x{k}"]),
                d=float(r[f"hf_d{k}"]),
                seq=int(r["hf_seq"]),
                motion_state=int(r["motion_state"]),
            ))
    out = pd.DataFrame(rows)
    if len(out):
        out = out.sort_values(["seq", "slot"]).reset_index(drop=True)
    return out


def analyze(path: str, plot: bool = False) -> None:
    df = pd.read_csv(path)
    missing = [c for c in REQ if c not in df.columns]
    if missing:
        print(f"{path}: hf列がありません({missing[:3]}...)。2026-09-15以降のfirmwareログが必要です")
        return
    wo = df[df["motion_state"] == WALL_OFF]
    if not len(wo):
        print(f"{path}: WALL_OFF区間なし")
        return
    s, e = int(wo.index[0]), int(wo.index[-1])
    # hf行は「前1ms」のサンプルなので1行後ろまで含める
    seg = df.iloc[max(s - 1, 0): e + 3]
    hf = expand(seg)
    gpos = df["gpos_hi"].values * 100.0 + df["gpos_lo"].values

    # 検出位置: hf_edge_rel>0 の最初の行で x_edge = gpos - hf_edge_rel
    det = df.iloc[s: e + 3]
    det = det[det["hf_edge_rel"] > 0]
    x_edge = float(gpos[det.index[0]] - det["hf_edge_rel"].iloc[0]) if len(det) else float("nan")

    side_col = df["hf_side"] if "hf_side" in df.columns else None
    side = ""
    if side_col is not None:
        sv = side_col.iloc[s: e + 1]
        sv = sv[sv >= 0]
        if len(sv):
            side = "  注視側=" + ("left45" if int(sv.iloc[0]) == 0 else "right45")
    print(f"== {path}")
    print(f"   WALL_OFF rows {s}..{e} ({e - s + 1} tick)  hfサンプル {len(hf)} 点  "
          f"1msあたり平均 {len(hf) / max(e - s + 1, 1):.2f}{side}")
    seqs = seg["hf_seq"].values
    gaps = int(np.sum(np.diff(seqs) != 1))
    print(f"   hf_seq 不連続 {gaps} 箇所  hf_cnt 内訳 {seg['hf_cnt'].value_counts().sort_index().to_dict()}")
    if np.isfinite(x_edge):
        print(f"   検出位置 x_edge = {x_edge:.2f} mm  (WALL_OFF開始 {gpos[s]:.2f} から +{x_edge - gpos[s]:.2f})")
    else:
        print("   検出なし(hf_edge_rel が立っていない)")
    if len(hf):
        print("   x_abs      d      Δx(前サンプル)   min-hold")
        mh = np.inf
        prev = None
        for _, r in hf.iterrows():
            if r["d"] < 100:
                mh = min(mh, r["d"])
            dx = (r["x_abs"] - prev) if prev is not None else 0.0
            mark = " <-- edge" if np.isfinite(x_edge) and prev is not None and prev < x_edge <= r["x_abs"] else ""
            print(f"   {r['x_abs']:8.2f} {r['d']:7.2f}   {dx:6.2f}        {mh if np.isfinite(mh) else float('nan'):7.2f}{mark}")
            prev = r["x_abs"]

    if plot and len(hf):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(hf["x_abs"], hf["d"], ".-", label="hf (4kHz)")
        if np.isfinite(x_edge):
            ax.axvline(x_edge, color="r", ls="--", label=f"edge {x_edge:.2f}")
        ax.set_xlabel("global_pos.dist [mm]")
        ax.set_ylabel("dist [mm]")
        ax.set_title(path.split("/")[-1])
        ax.grid(True, alpha=0.3)
        ax.legend()
        plt.tight_layout()
        plt.show()


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    plot = "--plot" in sys.argv
    if not args:
        print(__doc__)
        return 1
    for p in args:
        analyze(p, plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
