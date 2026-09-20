#!/usr/bin/env python3
"""エンコーダ(AS5147P)の角度依存誤差を低速の直進ログから同定し、
左右それぞれの補正テーブル(64点、単位 count)を出すツール。

考え方:
  走行中は v_c=(v_l+v_r)/2 が速度PIDへ戻るので、エンコーダ誤差の同相成分は
  機体の実運動へ化ける。片輪ずつ「滑らかな真値」を仮定して残差を取ると
  左右の誤差が混ざる。そこで左右差とジャイロだけを使う:

      C*(F_r - F_l) = T*∫w dt + eps*∫v_c dt - C*e_r(th_r) - C*e_l(th_l) + const

  F_* は生角度をアンラップした前進方向の累積 count、C=pi*D/16384 [mm/count]、
  e_*(th) は「測定角 - 真の角」[count]。速度PIDが入れる同相の揺れは差で消え、
  実際のヨーの揺れはジャイロが測っているので引ける(低速=スリップ小が前提)。
  e_* を 1..K 次の高調波で置き、T・eps と同時に最小二乗する。ジャイロのゼロ点や
  ゆっくりしたスリップは、両辺に同じ移動平均ハイパスを掛けて落とす。

  使うのは低速(v=400 前後)の直進だけ。1 本の直進では左右の位相関係がほぼ固定
  (生角度で th_l+th_r=一定)なので e_r と e_l を分離できないが、機体を置き直す
  たびに左右の相対位相が変わるので、複数本をプールすれば分離できる(cond で確認)。
  AS5147P は絶対角なので、電源を入れ直したログ同士もそのままプールできる。
  超信地は渡しても当てはめから除外する: 4 輪のスクラブで左右差とジャイロが合わず
  (ハイパス後の残差 RMS が直進の約 5 倍、車輪角に同期しない)、混ぜると直進側の
  1 本抜き検証が悪化する(20260920_2214xx/2215xx で確認)。
  1 次成分にはタイヤの偏心(振れ)も入る。地面基準の「車輪角→進んだ距離」の補正
  としては正しいが、タイヤ/磁石を付け直したら取り直すこと。
  ログの v_l_enc / v_r_enc は補正前の生角度なので、補正の有効/無効に関係なく
  校正し直せる。目安は 8 本以上。

使い方:
    # 同定してテーブルを書き出す(profile/hf/enc_lut.yaml -> 機体では /enc_lut.hf)
    python3 tools/param_tuner/enc_lut_fit.py -o tools/param_tuner/profile/hf/enc_lut.yaml logs/2026..._*.csv
    # 補正を有効にして走ったログで、ファームが同じテーブルを同じ向きで引いているか照合
    python3 tools/param_tuner/enc_lut_fit.py --check tools/param_tuner/profile/hf/enc_lut.yaml logs/2026..._*.csv
"""
import argparse
import datetime
import os

import numpy as np
import pandas as pd

ENC_RES = 16384
TIRE_D = 14.05          # [mm] hardware.yaml の tire。C のスケールにしか効かない
C = np.pi * TIRE_D / ENC_RES
DT = 0.001              # [s] ログは 1kHz
K = 8                   # 高調波の次数
HP_WIN = 401            # ハイパス用の移動平均窓 [sample]。車輪 2〜3 回転ぶん
N_LUT = 64


def unwrap_cum(raw):
    d = np.diff(raw.astype(float), prepend=float(raw[0]))
    d[d < -ENC_RES / 2] += ENC_RES
    d[d > ENC_RES / 2] -= ENC_RES
    return np.cumsum(d)


def highpass(x):
    """移動平均を引く。端は捨てる('valid')。x は (n,) か (n, m)。"""
    k = np.ones(HP_WIN) / HP_WIN
    h = HP_WIN // 2
    if x.ndim == 1:
        return x[h:-h] - np.convolve(x, k, mode="valid")
    return np.column_stack([highpass(x[:, j]) for j in range(x.shape[1])])


def harm_cols(theta):
    cols = []
    for k in range(1, K + 1):
        cols += [np.cos(k * theta), np.sin(k * theta)]
    return np.column_stack(cols)


def load(path):
    d = pd.read_csv(path)
    need = {"v_l_enc", "v_r_enc", "w_lp", "v_l", "v_r"}
    if not need.issubset(d.columns) or len(d) < 3 * HP_WIN:
        return None
    raw_l = d["v_l_enc"].values
    raw_r = d["v_r_enc"].values
    f_l = unwrap_cum(raw_l)             # 左: 生角度が増える向きが前進
    f_r = -unwrap_cum(raw_r)            # 右: 生角度が減る向きが前進
    th_l = raw_l / ENC_RES * 2 * np.pi
    th_r = raw_r / ENC_RES * 2 * np.pi
    y = C * (f_r - f_l)
    w_int = np.cumsum(d["w_lp"].values) * DT
    v_int = np.cumsum((d["v_l"].values + d["v_r"].values) / 2) * DT
    a = np.column_stack([w_int, v_int, -C * harm_cols(th_r), -C * harm_cols(th_l)])
    kind = "pivot" if d["ideal_w"].abs().max() > 1 else "straight"
    return dict(path=path, kind=kind, y=highpass(y), a=highpass(a),
                revs=(abs(f_l[-1]) / ENC_RES, abs(f_r[-1]) / ENC_RES))


def solve(runs):
    a = np.vstack([r["a"] for r in runs])
    y = np.concatenate([r["y"] for r in runs])
    scale = np.linalg.norm(a, axis=0)
    coef, *_ = np.linalg.lstsq(a / scale, y, rcond=None)
    cond = np.linalg.cond(a / scale)
    return coef / scale, cond


def rms_um(runs, coef, use_harm):
    """ハイパス後の左右差残差 RMS [um]。use_harm=False は T/eps だけ引いた素の値。"""
    out = []
    for r in runs:
        c = coef.copy()
        if not use_harm:
            c[2:] = 0
        out.append(r["y"] - r["a"] @ c)
    e = np.concatenate(out)
    return 1000 * np.sqrt(np.mean(e ** 2))


def lut_from(coef_h):
    th = np.arange(N_LUT) / N_LUT * 2 * np.pi
    return harm_cols(th) @ coef_h


def amp_deg(coef_h):
    a = coef_h.reshape(K, 2)
    return np.hypot(a[:, 0], a[:, 1]) * 360.0 / ENC_RES


def lut_apply(raw, table):
    """ファームの SensingTask::correct_enc() と同じ引き方(上位 6bit + 線形補間)。"""
    raw = raw.astype(np.int64)
    idx = (raw >> 8) & (N_LUT - 1)
    frac = (raw & 0xFF) / 256.0
    t = np.asarray(table, dtype=float)
    return raw - (t[idx] + (t[(idx + 1) & (N_LUT - 1)] - t[idx]) * frac)


def speed(angle, sign):
    d = np.diff(angle, prepend=angle[0])
    d[d < -ENC_RES / 2] += ENC_RES
    d[d > ENC_RES / 2] -= ENC_RES
    return sign * d * C / DT


def check(lut_path, files):
    """ログの v_l/v_r(ファームが補正後の角度から計算)を、生角度からの再計算と比べる。
    テーブルを当てた再計算のほうが合えば、ファームは同じテーブルを同じ向きで引いている。"""
    import yaml
    with open(lut_path) as f:
        y = yaml.safe_load(f)
    print(f"{lut_path}: enc_lut_enable={y.get('enc_lut_enable')}")
    print("ログの輪速との差 RMS [mm/s]   左: 補正なし / ありで再計算    右: 補正なし / あり")
    for path in files:
        d = pd.read_csv(path)
        m = (d["v_l"].abs() > 100) & (d["v_r"].abs() > 100)
        m &= m.shift(1, fill_value=False)
        if m.sum() < 100:
            continue
        out = []
        for col, enc, tab, sign in (("v_l", "v_l_enc", "enc_lut_l", 1), ("v_r", "v_r_enc", "enc_lut_r", -1)):
            raw = d[enc].values
            v_fw = d[col].values
            for ang in (raw.astype(float), lut_apply(raw, y[tab])):
                out.append(np.sqrt(np.mean((speed(ang, sign) - v_fw)[m.values] ** 2)))
        verdict = "補正あり" if (out[1] < out[0] and out[3] < out[2]) else "補正なし(または不一致)"
        print(f"  {path[-19:]}   {out[0]:6.2f} / {out[1]:6.2f}        {out[2]:6.2f} / {out[3]:6.2f}"
              f"   -> ファームは{verdict}")


def write_yaml(path, lut_l, lut_r, files, summary):
    lines = [
        "# エンコーダ角度依存誤差の補正テーブル。tools/param_tuner/enc_lut_fit.py が生成する",
        "# (手で編集しない)。機体へは /enc_lut.hf として送る。",
        f"# 生角度 {ENC_RES // N_LUT} count 刻み {N_LUT} 点、単位 count。"
        "補正後の角度 = 生角度 - table[生角度](点間は線形補間)。",
        "# 磁石・タイヤを付け直したら取り直すこと。",
        f"# generated: {datetime.datetime.now():%Y-%m-%d %H:%M}  logs: "
        + " ".join(os.path.basename(f) for f in files),
    ] + [f"# {s}" for s in summary] + [
        "enc_lut_enable: 1  # 0 で補正なし",
        "enc_lut_l: [" + ", ".join(f"{v:.2f}" for v in lut_l) + "]",
        "enc_lut_r: [" + ", ".join(f"{v:.2f}" for v in lut_r) + "]",
        "",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"\nwrote {path}")


def main(files, out_path=None):
    loaded = [r for r in (load(f) for f in files) if r is not None]
    for r in loaded:
        note = "  (超信地は除外)" if r["kind"] == "pivot" else ""
        print(f"{r['path'][-19:]}  {r['kind']:8s} n={len(r['y'])}  "
              f"revs L {r['revs'][0]:.1f} / R {r['revs'][1]:.1f}{note}")
    runs = [r for r in loaded if r["kind"] == "straight"]
    if not runs:
        print("使える直進ログがない")
        return

    coef, cond = solve(runs)
    if cond > 20:
        print(f"warning: cond {cond:.0f}。左右の相対位相が似たログしかなく e_r/e_l の"
              "分離が弱い。機体を置き直して本数を増やすこと")
    h_r, h_l = coef[2:2 + 2 * K], coef[2 + 2 * K:]
    print(f"\ntread {coef[0]:.2f} mm  (e_r-e_l) {coef[1] * 100:+.2f} %  cond {cond:.1f}")
    print(f"左右差残差 RMS: 補正なし {rms_um(runs, coef, False):.1f} um -> "
          f"補正あり {rms_um(runs, coef, True):.1f} um")
    print("次数        " + " ".join(f"{k:6d}" for k in range(1, K + 1)))
    print("右 振幅[deg] " + " ".join(f"{v:6.3f}" for v in amp_deg(h_r)))
    print("左 振幅[deg] " + " ".join(f"{v:6.3f}" for v in amp_deg(h_l)))

    # 汎化: 1 本抜いて当てはめ、抜いた 1 本の残差がどれだけ減るか
    loo_r, loo_l = [], []
    if len(runs) >= 3:
        print("\n1本抜き検証 (抜いたログの左右差残差 RMS [um])")
        for i, r in enumerate(runs):
            rest = runs[:i] + runs[i + 1:]
            c, _ = solve(rest)
            loo_r.append(lut_from(c[2:2 + 2 * K]))
            loo_l.append(lut_from(c[2 + 2 * K:]))
            c_eval = c.copy()
            c_eval[:2] = coef[:2]
            before, after = rms_um([r], c_eval, False), rms_um([r], c_eval, True)
            print(f"  {r['path'][-19:]} {r['kind']:8s} {before:6.1f} -> {after:6.1f}"
                  f"  ({(1 - after / before) * 100:4.0f}% 減)")

    lut_r, lut_l = lut_from(h_r), lut_from(h_l)
    if loo_r:
        print(f"1本抜き間のテーブルのばらつき(各点 max-min の中央値): "
              f"右 {np.median(np.ptp(np.array(loo_r), axis=0)):.1f} count / "
              f"左 {np.median(np.ptp(np.array(loo_l), axis=0)):.1f} count")
    print(f"\nテーブル p-p: 右 {np.ptp(lut_r):.1f} count ({np.ptp(lut_r) * 360 / ENC_RES:.2f} deg)"
          f" / 左 {np.ptp(lut_l):.1f} count ({np.ptp(lut_l) * 360 / ENC_RES:.2f} deg)")
    # ファームと同じ 64 点線形補間にしたときの、高調波モデルからのずれ
    th = np.arange(ENC_RES) / ENC_RES * 2 * np.pi
    interp_err = max(np.abs(np.arange(ENC_RES) - lut_apply(np.arange(ENC_RES), t) - harm_cols(th) @ h).max()
                     for t, h in ((lut_r, h_r), (lut_l, h_l)))
    print(f"64点線形補間の誤差: 最大 {interp_err:.2f} count")

    if out_path:
        summary = [
            f"straight runs: {len(runs)}  residual {rms_um(runs, coef, False):.1f} -> "
            f"{rms_um(runs, coef, True):.1f} um  cond {cond:.1f}",
            "amp[deg] k=1..4  R: " + " ".join(f"{v:.3f}" for v in amp_deg(h_r)[:4])
            + "  L: " + " ".join(f"{v:.3f}" for v in amp_deg(h_l)[:4]),
        ]
        write_yaml(out_path, lut_l, lut_r, [r["path"] for r in runs], summary)
    else:
        print("\n(-o profile/hf/enc_lut.yaml でテーブルを書き出す)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("-o", "--out", help="補正テーブルの書き出し先 (profile/hf/enc_lut.yaml)")
    ap.add_argument("--check", metavar="ENC_LUT_YAML",
                    help="同定せず、ログの輪速がこのテーブルで補正されているかを照合する")
    a = ap.parse_args()
    if a.check:
        check(a.check, a.logs)
    else:
        main(a.logs, a.out)
