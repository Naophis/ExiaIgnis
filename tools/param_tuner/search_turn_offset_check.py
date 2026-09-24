#!/usr/bin/env python3
"""探索スラローム(Normal)の front/back オフセットのずれを、探索走行ログから旋回ごとに推定するツール。

対象は SLA_FRONT_STR(3) → SLALOM(4) → SLA_BACK_STR(14) の並びで、|ang|≈90°・v<350 の旋回。
探索の Normal ターンは profiles.yaml の `normal:` インデックスが指すファイル(現状 list[1] =
t_400.hf)の値で走る。t_300.yaml の normal は `# not used`。現在値を出したいときは
--yaml でそのファイルを渡す。

基準点 D(探索の直進/旋回の切り替わり点)はファームのセンサー基準で定義する:
  - 前壁が 1 セル先のとき front_mid が offset.yaml front_dist_offset(95.5) を読む位置
  - 直進の壁切れ検知から search_wall_off_*_dist_offset(52.5/56.5) 手前
両者はこのログで互いに整合していた(壁切れ補正後の直進の次の旋回で前壁 96.8)。
旋回は D(入口)から D'(出口)へ、front + アーク + back で移る。

旋回ごとに出す量(単位 mm、断りが無ければ実走距離ベース):
  e_in     入口縦誤差。前壁読み(front_d)が front_ref を横切った位置と front 直進開始
           tick の差。+ = D を過ぎて先に居た。読みの LUT 傾きに依らない交差法。
           ms=1 末尾〜ms=3 で交差が見つからなければ末尾 12tick の直線で外挿(extrap)。
  elat_in  入口横誤差(右正)。判断時の外側 45° 読み − 45(ファームの back 補正と同じ量、
           th_offset_dist=52 未満のとき; outer)。無ければ入口通路の両壁(判断点の
           45〜80mm 手前; corr)。
  fcmd     front 直進の実走距離(= yaml front + 前壁補正(±9 clamp) − adachi 計算中の移動 + 端数)
  adx/ady  アークの前進/横変位。dist 増分と kim_theta(入口通路基準)で積分。
  bcmd     back 直進の実走距離(= yaml back + 外壁補正(±6) + 旋回中壁切れ補正(±6) + 端数)
  yaw      back 直進中の kim_theta 平均 [deg](CCW 正、出口方向基準)
  off      back 直進中の横位置(右正)。両壁なら (l45−r45)/2、片壁なら l45−45 / 45−r45。
           ヨーの見かけ分は off + lat_k·yaw で戻す(control_law の lat_now は左正で
           lat_now − k·Δang、右正にすると符号が反転する)。
  out_lat  off を旋回の外側正に直したもの。+ = 大回り。
  e_out    出口縦誤差。− = D' の手前で back が終わっている。3 通りで取り、取れた方法を method に出す:
             nextfront  連続旋回: 次の front 直進で前壁が front_ref を横切るまでの距離
             woff       次の直進で壁切れ補正が発火(ideal_dist の急落 + 発火側 45° > noexist_th)
                        した場合、発火セルまでの実走長 L に対し (発火セル数)·90 − L。
                        1 セル直進なら直進全長との差。それ以前のセルは補正なし(≈90)前提
             chain      補正なしの直進(長さ 90n)を挟んだ次の旋回の e_in に n·90 − 直進長 を足す
  F*       この旋回で入口誤差が 0 だったとして出口を中心線に乗せる front
           = (e_in + fcmd + adx − out_lat) − adx_mean。e_in が無い旋回は出さない。
  G_B      入口中心線から D' までの横距離 = s·elat_in + bcmd + ady − e_out(s: 右旋回 +1)

推奨値:
  front  F* の中央値。入口縦誤差はファームが前壁で補正する前提の値なので、生の out_lat 平均
         (−out_lat が直接残差)とは入口誤差の残り(clamp 超過分など)だけ違う。両方出す。
  back   Δback = −mean(e_out_adj)。e_out_adj = e_out − (s·elat_in + c_wall) で、外壁補正が
         入った旋回では c_wall = −s·elat_in なので e_out そのもの。旋回中の ±6 補正は
         位置誤差を検知して入ったものなのでそのまま含める。--yaml があれば back + Δback を出す。

チェック用:
  G_F + G_B は幾何的には 90(D が境界から p だけずれても (45−p)+(45+p))。ログの値が 90 から
  大きく外れるなら、アーク中のエンコーダ距離が実変位と合っていない(内輪が逆転する r=22 では
  スクラブが大きい)か、センサー基準 D が前後で非対称。推奨値はセンサー基準 D に対する
  もので、探索の次動作(前壁補正・壁切れ)が参照するのも同じ D なので実用上はこれで良い。
  vin      旋回中の内輪速度最小 / ideal_v。負なら内輪が逆転している。
  sat      旋回中 |duty| > 99 の tick 数。

使い方:
    python3 tools/param_tuner/search_turn_offset_check.py logs/20260925_002647.csv
    python3 tools/param_tuner/search_turn_offset_check.py logs/2026092*.csv --summary \\
        --yaml tools/param_tuner/profile/hf/t_400.yaml
    python3 tools/param_tuner/search_turn_offset_check.py logs/20260925_002647.csv --dump 1429
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

D = 180.0 / np.pi
DT = 0.001
CELL = 90.0
FRONT_REF = 95.5      # offset.yaml front_dist_offset
LAT_K = 0.9           # hardware.yaml start_align.lat_k [mm/deg]
TH_OFFSET = 52.0      # offset.yaml th_offset_dist
WALL_REF = 45.0       # offset.yaml sla_wall_ref_l/r
BACK_CLAMP = 6.0      # offset.yaml normal_sla_offset_back
NOEXIST_TH = 49.0     # offset.yaml wall_off_hold_noexist_th_l/r(探索壁切れの発火しきい値)

FRONT_STR, SLALOM, BACK_STR, STRAIGHT = 3, 4, 14, 1
NEED = ["motion_state", "ideal_v", "ideal_w", "ideal_dist", "dist", "kim_theta",
        "front_d", "left45_d", "right45_d", "v_l", "v_r", "duty_l", "duty_r"]


def segments(ms):
    out = []
    s = 0
    n = len(ms)
    for i in range(1, n + 1):
        if i == n or ms[i] != ms[s]:
            out.append((s, i, int(ms[s])))
            s = i
    return out


def travel_axis(v):
    """ideal_v から tick ごとの走行距離の累積 [mm]"""
    return np.cumsum(v * DT)


def front_cross(df, i_dec, i_end, front_ref):
    """front_d が front_ref を横切る位置を探す。
    戻り値 (e, method): e = −(判断 tick から交差までの距離)。+ = 交差を過ぎていた。"""
    lo = max(0, i_dec - 60)
    v = df.ideal_v.values[lo:i_end]
    fd = df.front_d.values[lo:i_end]
    x = travel_axis(v)
    x = x - x[i_dec - lo]
    valid = (fd > 60) & (fd < 175)
    if valid.sum() < 5:
        return np.nan, "nofront"
    idx = np.flatnonzero(valid)
    for a, b in zip(idx[:-1], idx[1:]):
        if b - a > 3:
            continue
        if (fd[a] - front_ref) * (fd[b] - front_ref) <= 0 and fd[a] != fd[b]:
            xc = x[a] + (front_ref - fd[a]) * (x[b] - x[a]) / (fd[b] - fd[a])
            return -xc, "cross"
    sel = idx[idx >= (i_dec - lo)][-12:]
    if len(sel) < 6:
        sel = idx[-12:]
    p = np.polyfit(x[sel], fd[sel], 1)
    if p[0] > -0.3:
        return np.nan, "flat"
    return -(front_ref - p[1]) / p[0], "extrap"


def lateral_window(l, r, yaw, lat_k):
    """右正の横位置と壁モード。両壁優先、片壁は 45 基準。"""
    okl = (l > 15) & (l < 80)
    okr = (r > 15) & (r < 80)
    both = okl & okr
    if both.sum() >= 3:
        return float(np.mean((l[both] - r[both]) / 2 + lat_k * yaw[both])), "B"
    if (okl & ~okr).sum() >= 3:
        m = okl & ~okr
        return float(np.mean(l[m] - WALL_REF + lat_k * yaw[m])), "L"
    if (okr & ~okl).sum() >= 3:
        m = okr & ~okl
        return float(np.mean(WALL_REF - r[m] + lat_k * yaw[m])), "R"
    return np.nan, "-"


def entry_lateral(df, fs, left):
    """判断時の外側 45°(ファームと同じ)→ 無ければ入口通路の両壁。(elat_in, c_wall, mode)"""
    l0 = df.left45_d.values[fs]
    r0 = df.right45_d.values[fs]
    outer = r0 if left else l0
    if 10 < outer < TH_OFFSET:
        e = (WALL_REF - r0) if left else (l0 - WALL_REF)
        c_wall = float(np.clip(WALL_REF - outer, -BACK_CLAMP, BACK_CLAMP))
        return float(e), c_wall, "outer"
    lo = max(0, fs - 250)
    v = df.ideal_v.values[lo:fs]
    back = np.cumsum(v[::-1] * DT)[::-1]  # 判断点までの残距離
    w = (back > 45) & (back < 80)
    l = df.left45_d.values[lo:fs][w]
    r = df.right45_d.values[lo:fs][w]
    both = (l > 15) & (l < 80) & (r > 15) & (r < 80)
    if both.sum() >= 3:
        e = float(np.mean((l[both] - r[both]) / 2))
        if abs(e) > 6:  # 通路で 6mm 超はあり得ない(柱・壁端を見ている)
            return np.nan, 0.0, "corr?"
        return e, 0.0, "corr"
    return np.nan, 0.0, "-"


def exit_longitudinal(df, segs, k, front_ref, cell, noexist_th=NOEXIST_TH):
    """旋回 k(segs のインデックス)の出口縦誤差。(e_out, method)"""
    if k + 2 >= len(segs):
        return np.nan, "-"
    ns, ne, nst = segs[k + 2]
    if nst == FRONT_STR:
        e, m = front_cross(df, ns, ne, front_ref)
        return (e, "nextfront/" + m) if np.isfinite(e) else (np.nan, "nextfront/" + m)
    if nst != STRAIGHT:
        return np.nan, f"ms{nst}"
    sg = df.iloc[ns:ne]
    x = travel_axis(sg.ideal_v.values)
    idl = sg.ideal_dist.values
    l45 = sg.left45_d.values
    r45 = sg.right45_d.values
    drops = np.flatnonzero(np.diff(idl) < -10)
    slen = float(x[-1])
    # ideal_dist の急落 = 新コマンド。セル途中(ideal_dist<75)で発火側 45° が noexist_th を
    # 超えていれば壁切れ補正の発火。それ以前の急落はセル境界の通常コマンド(≈90 ずつ)。
    m = 0
    for j, i in enumerate(drops):
        fired_r = noexist_th < r45[i] < 180
        fired_l = noexist_th < l45[i] < 180
        if idl[i] < 75 and (fired_r or fired_l):
            end = float(x[drops[j + 1]]) if j + 1 < len(drops) else slen
            side = "R" if fired_r else "L"
            return (m + 1) * cell - end, f"woff{side}@{x[i]:.0f}"
        m += 1
    if k + 3 < len(segs) and segs[k + 3][2] == FRONT_STR:
        e, mm = front_cross(df, segs[k + 3][0], segs[k + 3][1], front_ref)
        n = int(round(slen / cell))
        if np.isfinite(e) and n >= 1:
            return e + n * cell - slen, f"chain{n}/{mm}"
        return np.nan, f"chain{n}/{mm}"
    return np.nan, "str-noinfo"


def analyze(df, name, a):
    ms = df.motion_state.values
    segs = segments(ms)
    rows = []
    for k, (s, e, st) in enumerate(segs):
        if st != SLALOM or e - s < 50 or k < 1 or k + 1 >= len(segs):
            continue
        if segs[k - 1][2] != FRONT_STR or segs[k + 1][2] != BACK_STR:
            continue
        seg = df.iloc[s:e]
        ang = seg.ideal_w.sum() * DT * D
        v0 = float(seg.ideal_v.iloc[0])
        if abs(round(abs(ang))) != 90 or v0 > a.v_max:
            continue
        left = ang > 0
        sgn = -1 if left else 1  # 入口横誤差(右正)→ 出口縦誤差の符号
        fs, fe, _ = segs[k - 1]
        bs, be, _ = segs[k + 1]

        e_in, m_in = front_cross(df, fs, s, a.front_ref)
        elat_in, c_wall, m_lat = entry_lateral(df, fs, left)
        fcmd = float(df.dist.values[fe - 1])
        bcmd = float(df.dist.values[be - 1])

        th = np.deg2rad(seg.kim_theta.values)
        dd = np.diff(seg.dist.values, prepend=seg.dist.values[0])
        adx = float(np.sum(dd * np.cos(th)))
        ady = float(abs(np.sum(dd * np.sin(th))))

        bk = df.iloc[bs:be]
        off, m_off = lateral_window(bk.left45_d.values, bk.right45_d.values,
                                    bk.kim_theta.values, a.lat_k)
        yaw = float(bk.kim_theta.mean())
        out_lat = off if left else -off

        e_out, m_out = exit_longitudinal(df, segs, k, a.front_ref, a.cell)
        # 入口横誤差のうちファームが補正しなかった分を除く
        ent = (sgn * elat_in if np.isfinite(elat_in) else 0.0) + c_wall
        e_out_adj = e_out - ent if np.isfinite(e_out) else np.nan

        gf = e_in + fcmd + adx - out_lat
        gb = (sgn * elat_in if np.isfinite(elat_in) else np.nan) + bcmd + ady - e_out
        inner = seg.v_l if left else seg.v_r
        sat = int(((seg.duty_l.abs() > 99) | (seg.duty_r.abs() > 99)).sum())
        rows.append(dict(
            log=name, idx=s, dir="L" if left else "R", v=int(round(v0)),
            e_in=e_in, m_in=m_in, elat_in=elat_in, m_lat=m_lat,
            fcmd=fcmd, adx=adx, ady=ady, bcmd=bcmd, yaw=yaw,
            off=off, m_off=m_off, out_lat=out_lat,
            e_out=e_out, e_out_adj=e_out_adj, m_out=m_out,
            GF=gf, GB=gb, Fstar=np.nan,
            vin=float(inner.min() / v0), sat=sat))
    return rows


def load_yaml_normal(path):
    import yaml
    with open(path) as f:
        y = yaml.safe_load(f)
    n = y["normal"]
    return {"front": {"L": float(n["front"]["left"]), "R": float(n["front"]["right"])},
            "back": {"L": float(n["back"]["left"]), "R": float(n["back"]["right"])}}


def dump(df, idx, a):
    ms = df.motion_state.values
    segs = segments(ms)
    k = next((i for i, (s, e, st) in enumerate(segs) if s == idx and st == SLALOM), None)
    if k is None:
        print(f"idx {idx} から始まる SLALOM が無い")
        return
    fs = segs[k - 1][0] if k >= 1 else idx
    be = segs[k + 1][1] if k + 1 < len(segs) else segs[k][1]
    end = min(len(df), be + 70)
    cols = ["motion_state", "ideal_v", "ideal_dist", "dist", "kim_theta",
            "front_d", "left45_d", "right45_d", "v_l", "v_r", "duty_sen"]
    d = df.iloc[max(0, fs - 30):end][cols].copy()
    d["duty_sen"] = d["duty_sen"] * D
    d = d.rename(columns={"motion_state": "ms", "duty_sen": "dsen_deg"})
    x = travel_axis(df.ideal_v.values[max(0, fs - 30):end])
    d.insert(1, "x_from_front", np.round(x - x[fs - max(0, fs - 30)], 1))
    pd.set_option("display.width", 300)
    pd.set_option("display.max_rows", 1000)
    print(f"turn idx {idx}: front {segs[k-1][0]}-{segs[k-1][1]-1}, slalom {idx}-{segs[k][1]-1}, "
          f"back {segs[k+1][0]}-{segs[k+1][1]-1}. x_from_front は front 直進開始 tick を 0 とした距離")
    print(d.round(2).to_string())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--summary", action="store_true", help="方向ごとの集計と推奨値を出す")
    ap.add_argument("--yaml", help="現在の normal の front/back を読む slalom yaml (例 profile/hf/t_400.yaml)")
    ap.add_argument("--dir", choices=["L", "R"], help="方向で絞る")
    ap.add_argument("--dump", type=int, metavar="IDX", help="この idx から始まる旋回を tick 単位で表示 (ログ1本)")
    ap.add_argument("--front-ref", type=float, default=FRONT_REF,
                    help=f"前壁の基準読み offset.yaml front_dist_offset (default {FRONT_REF})")
    ap.add_argument("--lat-k", type=float, default=LAT_K, help=f"ヨー→見かけ横ずれ [mm/deg] (default {LAT_K})")
    ap.add_argument("--cell", type=float, default=CELL)
    ap.add_argument("--v-max", type=float, default=350, help="この速度以下の 90° 旋回だけを Normal とみなす")
    a = ap.parse_args()

    if a.dump is not None:
        if len(a.logs) != 1:
            print("--dump はログ1本を指定する")
            return 1
        dump(pd.read_csv(a.logs[0]), a.dump, a)
        return 0

    rows = []
    for f in a.logs:
        df = pd.read_csv(f)
        miss = [c for c in NEED if c not in df]
        if miss:
            print(f"skip {f}: 列がない {miss}")
            continue
        rows += analyze(df, os.path.basename(f).replace(".csv", ""), a)
    if not rows:
        print("Normal 旋回(3→4→14, |ang|=90, v<%.0f)が無い" % a.v_max)
        return 1
    t = pd.DataFrame(rows)
    if a.dir:
        t = t[t.dir == a.dir]
    for d in "LR":
        m = t.dir == d
        if m.any():
            t.loc[m, "Fstar"] = t.loc[m, "GF"] - t.loc[m, "adx"].mean()
    t = t.sort_values(["dir", "log", "idx"])
    pd.set_option("display.width", 320)
    pd.set_option("display.max_rows", 500)
    show = t.drop(columns=["GF", "GB"]).copy()
    print(show.round(2).to_string(index=False))

    if not a.summary:
        return 0

    cur = load_yaml_normal(a.yaml) if a.yaml else None
    print()
    print("summary (単位 mm。F*: 入口誤差 0 で出口が中心に乗る front。Δback = −mean(e_out_adj))")
    for d in "LR":
        x = t[t.dir == d]
        if x.empty:
            continue
        fs = x.Fstar.dropna()
        eo = x.e_out_adj.dropna()
        ol = x.out_lat.dropna()
        print(f"  {d}: n={len(x)}  F*: n={len(fs)} med={fs.median():6.2f} mean={fs.mean():6.2f} sd={fs.std(ddof=0):4.2f}"
              f"  | out_lat mean={ol.mean():6.2f} sd={ol.std(ddof=0):4.2f} (n={len(ol)})"
              f"  | e_out_adj: n={len(eo)} mean={eo.mean():6.2f} med={eo.median():6.2f} sd={eo.std(ddof=0):4.2f}"
              f"  | G_F+G_B med={(x.GF + x.GB).median():6.1f} (幾何なら 90)")
        if fs.empty or eo.empty:
            print("     推奨値なし(e_in か e_out が取れた旋回が無い。前壁・壁切れが無い走行)")
            continue
        if cur:
            f0 = cur["front"][d]
            b0 = cur["back"][d]
            print(f"     front: {f0:.2f} -> {fs.median():.2f} (Δ{fs.median() - f0:+.2f})"
                  f"   back: {b0:.2f} -> {b0 - eo.mean():.2f} (Δ{-eo.mean():+.2f})")
        else:
            print(f"     front -> {fs.median():.2f}   back -> 現在値 {-eo.mean():+.2f}")
    print()
    print("入口縦誤差が大きい(clamp ±9 超え)旋回が多いなら、まず back を直して出口を D' に乗せると"
          "次旋回の前壁補正が効く範囲に入る。F* はその状態を前提にした値。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
