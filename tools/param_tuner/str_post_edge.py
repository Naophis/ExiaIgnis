#!/usr/bin/env python3
"""直進の壁なし区間の横位置を、左右の柱の立ち下がりの位置の差から出す(2026-10-02)。

直進では左右の柱が同じ位置にあるので、右へずれると左の縁は手前・右の縁は先へ動く:
    読み = (x_R − x_L)/2 = k0 + gain·δ + kappa·ψ   (δ = 実際の横位置、+ は右。ψ = 迷路に対する向き)
斜め(dia_post_edge.py)と違って柱の間隔も走行距離の倍率も要らない。gain は 1 でなく約 0.4
(7 本 41 区間の組の差を gain·(ジャイロの横の移動) + κ·Δψ で当てると 0.37〜0.41。ビームが進行方向
から約 68° に向いている形)。1 のままだと見かけの ψ0 が g·ψ0 − (1 − g)·ψ_g と向きで動き、
目標に着いた後も 0.3〜0.9mm/区画ずれ続けた(020709 / 020735 / 020807)。

縁 = 45° LED1 の生値が「直前の山の rel_thr(50%)」を上から下へ切った点(直線補間)。固定の
生値だと左右のゲイン差(左の柱の山 230〜240 raw、右 130〜150)と柱までの距離で切る位置が
変わるので、山に対する割合にする(直進の柱の後ろは谷まで落ちるので、斜めで相対値を避けた
理由=縁の後ろに次の壁が見える、は起きない)。下地(頂点の手前 rise_max(78mm)の間の最小、
1mm ごとの最小の輪)は柱かどうかの判定に使う(しきい値に入れると 004431 の残差 0.12 → 0.27)。
壁の切れ目(500 raw の段)の 50% を切る点は柱の山の 50% より約 5mm 手前になる(ビーム幅
σ≈9mm のモデルで 5.8mm、20261002_004431 で 左 6.4 / 右 3.9)ので、壁の終わりは縁に
しない: 山が下地の倍以上(contrast_min)で、頂点の手前に下地近く(+30%)の読みがあった山だけ
柱とみなす(壁は 78mm 以上続いて下地が無い)。山の頂点から fall_max(30mm)以内に切ること。

firmware(include/planning/dia_post_edge_detector.hpp の rel_thr > 0 の経路、
SensorProcessor::update_str_post_edge)と同じ手順(StrPostDetector)。ファームと同じく
1 tick に 4 回読んでいる側は S1〜S3、読んでいなければ S0 を使う。
向き: ジャイロの純積分 ang_kf_sum(壁のスナップで動かない)を使う。

出力(ログごと): 組の数、δ の平均と σ、ψ0(δ − ジャイロの向きの積分 の最小二乗の傾き)、
直線を引いた後の残差、次の組が来る直前の推測(読みの座標)と来た組の差 pred、ファームの
spe_delta との差 fw_diff、k0 = 壁からの引き継ぎ(両壁の区間の終わりの (L45 − R45)/2 + k0)を
ジャイロで最初の組まで進めた推測と来た組の δ の差 + k0。壁の終わりで機体が平行(psi_we ≈ 0 =
引き継いだ ψ0 と柱から出した ψ0 の差)な走行の値を使う。

  python3 str_post_edge.py logs/20261002_004431.csv ... [-v] [--k0 K0]
  python3 str_post_edge.py --dump logs/x.csv   # ホスト検証用(tests/str_post_edge_host)
"""
import argparse
import math
import sys

import numpy as np
import pandas as pd

L45_GAIN = (741.680791, 73.580924)   # sensor.yaml gain.L45 (a, b): dist = a/ln(raw) − b
R45_GAIN = (488.975344, 39.834352)   # gain.R45
PITCH = 90.0


def raw_of_dist(dist, gain):
    """距離 [mm] → 生値(calc_sensor_val の逆: raw = exp(a/(dist + b)))"""
    a, b = gain
    return math.exp(a / (dist + b))


KRING = 96   # 下地の輪(1mm ごとの最小)。rise_max 以上


class Params:
    rel_thr = 0.5
    post_dist_max = 120.0  # [mm] 山の距離の上限 → peak_min(raw、側ごと)。遠い側の柱は表より弱い(013403)
    low_ratio = 0.3        # 下地 + これ ×(山 − 下地)以下の読みが頂点の手前にあること
    contrast_min = 0.5     # (山 − 下地)/山 がこれ以上なら柱
    rise_max = 78.0        # [mm] x_low から縁まで(柱 25〜60)。下地を見る窓
    fall_max = 30.0        # [mm] 山の頂点から縁まで
    tol = 20.0             # [mm] 組(|x_R − x_L|)と同じ側(90 の整数倍)の許容
    gate_kmax = 3          # 同じ側の間隔が 90 × これを超えたら整数倍の確認をしない
    # 読み(縁の差/2)= k0 + gain·δ + kappa·ψ。7 本 41 区間の組の差の当てはめ(2026-10-02)で gain 0.37〜0.41、
    # kappa 64。gain 1 だと見かけの ψ0 が g·ψ0 − (1 − g)·ψ_g と向きで動く
    gain = 0.4
    kappa = 64.0           # [mm/rad、読み]。検知器では kappa/gain(実際の mm/rad)
    k0 = -1.5              # [mm、読み] 真ん中にいるときの読み
    head_gain = 75.0       # [mm/rad] 制御へ渡す横位置に足す向きの分(壁の読みの向きの感度に合わせる)
    wall_seed = 1          # 両壁の区間の終わりで横位置と ψ0 を引き継ぐ
    seed_min_len = 40.0    # [mm]
    seed_tau = 10.0        # [mm]
    seed_dev = 1.5         # [mm] どちらかの 45° が指数平均からこれ以上離れたら区間の終わり
    psi0_w = 8100.0        # [mm^2] ψ0 の事前値の重み

    def __init__(self):
        self.peak_min = [raw_of_dist(self.post_dist_max, L45_GAIN), raw_of_dist(self.post_dist_max, R45_GAIN)]


class Side:
    def __init__(self):
        self.has_prev = False
        self.prev_x = self.prev_y = self.prev_c = self.prev_psi = 0.0
        self.peak = 0.0
        self.peak_x = 0.0
        self.base = 0.0
        self.has_low = False
        self.x_low = 0.0
        self.has_edge = False
        self.edge_x = 0.0
        self.ring_bin = [-1000000] * KRING
        self.ring_y = [0.0] * KRING


class StrPostDetector:
    """dia_post_edge_detector.hpp の rel_thr > 0 の経路と同じ(ψ0 の最小二乗・κ・now_delta も)"""
    LEFT, RIGHT = 0, 1

    def __init__(self, p):
        self.p = p
        self.kappa_true = p.kappa / p.gain
        self.arm()

    def arm(self):
        self.s = [Side(), Side()]
        self.has_last = False
        self.last_side = 0
        self.last_x = self.last_c = self.last_psi = 0.0
        self.n_pairs = 0
        self.delta = self.delta_gyro = self.pos = self.c_pos = 0.0
        self.eps_deg = 0.0
        self.psi0 = 0.0
        self.n_psi = 0
        self.ls = [0, 0.0, 0.0, 0.0, 0.0, 0.0]  # n, x0, sx, sy, sxx, sxy
        self.prior_w = 0.0
        self.prior_b0 = 0.0
        self.edges = []  # (side, x) 採った縁(解析用)
        self.seeds = []  # (x, delta, psi0) 壁からの引き継ぎ(解析用)
        if self.p.psi0_w > 0:
            self.set_psi0_prior(0.0, self.p.psi0_w)

    def set_psi0_prior(self, b0, w):
        self.prior_b0, self.prior_w = b0, w
        self.recompute_psi0()

    def recompute_psi0(self):
        n, _, sx, sy, sxx, sxy = self.ls
        den = n * sxx - sx * sx
        num = n * sxy - sx * sy
        if self.prior_w > 0:
            self.psi0 = (num + n * self.prior_w * self.prior_b0) / (den + n * self.prior_w) if n >= 1 else self.prior_b0
            self.n_psi = n + 1
        elif n >= 2 and den > 1e-3:
            self.psi0 = num / den
            self.n_psi = n - 1

    def seed(self, x, c, psi, delta, psi0, w):
        """壁からの引き継ぎ(ファームの DiaPostEdgeDetector::seed と同じ)。delta は実際の mm"""
        self.delta = delta
        self.delta_gyro = delta - self.kappa_true * psi
        self.pos, self.c_pos = x, c
        if self.n_pairs < 1:
            self.n_pairs = 1
        self.seeds.append((x, delta, psi0))
        self.set_psi0_prior(psi0, w)

    def accept_same_side(self, s, e):
        if not s.has_edge:
            return True
        g = e - s.edge_x
        k = round(g / PITCH)
        if self.p.gate_kmax > 0 and k > self.p.gate_kmax:
            return True
        return k >= 1 and abs(g - k * PITCH) <= self.p.tol

    def update(self, side, x, y, c, psi):
        """dia_post_edge_detector.hpp の update_rel と同じ"""
        p, s = self.p, self.s[side]
        paired = False
        if s.has_prev and s.peak >= p.peak_min[side]:
            thr = p.rel_thr * s.peak
            if s.prev_y >= thr and y < thr:
                f = (s.prev_y - thr) / (s.prev_y - y)
                e = s.prev_x + (x - s.prev_x) * f
                ec = s.prev_c + (c - s.prev_c) * f
                epsi = s.prev_psi + (psi - s.prev_psi) * f
                if (s.has_low and s.peak - s.base >= p.contrast_min * s.peak and e - s.peak_x <= p.fall_max
                        and e - s.x_low <= p.rise_max and self.accept_same_side(s, e)):
                    s.has_edge = True
                    s.edge_x = e
                    self.edges.append((side, e))
                    paired = self.pair(side, e, ec, epsi)
                    self.has_last = True
                    self.last_side = side
                    self.last_x, self.last_c, self.last_psi = e, ec, epsi
                s.peak = 0.0
        b = math.floor(x)
        i = b % KRING
        if s.ring_bin[i] != b:
            s.ring_bin[i] = b
            s.ring_y[i] = y
        elif y < s.ring_y[i]:
            s.ring_y[i] = y
        if y > s.peak:
            s.peak = y
            s.peak_x = x
            pb = math.floor(x)
            base = y
            for bb in range(pb - 1, pb - int(p.rise_max) - 1, -1):
                j = bb % KRING
                if s.ring_bin[j] == bb and s.ring_y[j] < base:
                    base = s.ring_y[j]
            s.base = base
            lvl = base + p.low_ratio * (y - base)
            s.has_low = False
            for bb in range(pb - 1, pb - int(p.rise_max) - 1, -1):
                j = bb % KRING
                if s.ring_bin[j] == bb and s.ring_y[j] <= lvl:
                    s.has_low = True
                    s.x_low = bb + 1.0
                    break
        s.has_prev = True
        s.prev_x, s.prev_y, s.prev_c, s.prev_psi = x, y, c, psi
        return paired

    def pair(self, side, e, ec, epsi):
        if not self.has_last or self.last_side == side:
            return False
        g = e - self.last_x
        if abs(g) > self.p.tol:
            return False
        d_read = -g * 0.5 if self.last_side == self.RIGHT else g * 0.5
        d = (d_read - self.p.k0) / self.p.gain  # 実際の横位置 [mm]
        mid = 0.5 * (e + self.last_x)
        cmid = 0.5 * (ec + self.last_c)
        dg = d - self.kappa_true * 0.5 * (epsi + self.last_psi)
        if self.n_pairs >= 1 and mid > self.pos:
            self.eps_deg = math.degrees(math.atan2(d - self.delta, mid - self.pos))
        ls = self.ls
        if ls[0] == 0:
            ls[1] = mid
        lx, ly = mid - ls[1], dg - cmid
        ls[0] += 1
        ls[2] += lx
        ls[3] += ly
        ls[4] += lx * lx
        ls[5] += lx * ly
        self.recompute_psi0()
        self.delta, self.delta_gyro, self.pos, self.c_pos = d, dg, mid, cmid
        self.n_pairs += 1
        return True

    def now_delta(self, x, c):
        d = self.delta_gyro - self.kappa_true * self.psi0 + (c - self.c_pos)
        if self.n_psi >= 1:
            d += self.psi0 * (x - self.pos)
        return d

    def now_delta_ctrl(self, x, c, psi_now):
        """制御へ渡す形(車軸 + head_gain·いまの向き、実際の mm)。ファームの str_post.dnow はこちら"""
        return self.now_delta(x, c) + self.p.head_gain * (psi_now + self.psi0)

    def now_delta_pred(self, x, c, psi_now):
        """次の組の δ(delta、向きの補正前)の推測 = 車軸 + κ·いまの向き"""
        return self.now_delta(x, c) + self.kappa_true * (psi_now + self.psi0)


def straight_segments(ms, dist):
    """motion_state == 1(STRAIGHT)の連続区間 [(i0, i1)]。dist が戻った所(次のモーション)で切る"""
    out, st = [], None
    for i, m in enumerate(ms):
        cut = st is not None and i > st and dist[i] < dist[i - 1] - 1.0
        if m == 1 and st is None:
            st = i
        elif (m != 1 or cut) and st is not None:
            out.append((st, i - 1))
            st = i if (m == 1 and cut) else None
    if st is not None:
        out.append((st, len(ms) - 1))
    return out


def run_segment(det, q, ticks, x, c, on_pair=None, on_tick=None, on_seed=None):
    """区間を検知器に流す(ファームの update_str_post_edge と同じ順: 壁の引き継ぎ → サンプル)。
    tick の最初に on_tick(det, i)、組ができたら on_pair(det, i)、引き継ぎで on_seed(det, i) を呼ぶ。"""
    p = det.p
    ang = np.radians(q["ang_kf_sum"].values.astype(float))
    l45 = q["left45_d"].values.astype(float)
    r45 = q["right45_d"].values.astype(float)
    wall_active = hold = False
    x0 = x_prev = ema_ang = ema_l = ema_r = 0.0
    for i, row in enumerate(ticks):
        if on_tick:
            on_tick(det, i)
        if p.wall_seed:
            both = 30.0 < l45[i] < 60.0 and 30.0 < r45[i] < 60.0
            if not both:
                hold = False
            close = False
            if both and not hold:
                if not wall_active:
                    wall_active, x0, ema_ang, ema_l, ema_r, x_prev = True, x[i], ang[i], l45[i], r45[i], x[i]
                elif abs(l45[i] - ema_l) > p.seed_dev or abs(r45[i] - ema_r) > p.seed_dev:
                    close = hold = True
                else:
                    a = min(1.0, (x[i] - x_prev) / p.seed_tau) if p.seed_tau > 0 else 1.0
                    ema_ang += (ang[i] - ema_ang) * a
                    ema_l += (l45[i] - ema_l) * a
                    ema_r += (r45[i] - ema_r) * a
                    x_prev = x[i]
            elif wall_active:
                close = True
            if close:
                wall_active = False
                if x_prev - x0 >= p.seed_min_len:
                    cw = c[i] + (-ang[i]) * (x_prev - x[i])
                    det.seed(x_prev, cw, -ema_ang, 0.5 * (ema_l - ema_r), ema_ang, p.psi0_w)
                    if on_seed:
                        on_seed(det, i)
        for side, xs, y, cs, ps in row:
            if det.update(side, xs, y, cs, ps) and on_pair:
                on_pair(det, i)


def sample_stream(d):
    """1 tick ごとに [(side, x, y, c, psi)] を左 → 右の順で(ファームと同じ)。
    x = dist + v·(読んだ時刻 − 600us)、c = ∫ψ_g dx(ψ_g = −ang_kf_sum [rad]、右向き +)。"""
    has_wo = all(c in d for c in ("wo_n", "wo_l1", "wo_tl1", "wo_r1", "wo_tr1"))
    x = d["dist"].values.astype(float)
    v = d["ideal_v"].values.astype(float)
    psi_g = -np.radians(d["ang_kf_sum"].values.astype(float))
    c = np.zeros(len(x))
    for i in range(1, len(x)):
        c[i] = c[i - 1] + psi_g[i] * (x[i] - x[i - 1])
    ticks = []
    for i in range(len(x)):
        r = d.iloc[i]
        row = []
        for side, raw_col, pre in ((0, "left45", "l"), (1, "right45", "r")):
            if has_wo and int(r["wo_n"]) == 4 and r[f"wo_t{pre}1"] > 0:
                for q in (1, 2, 3):
                    t = r[f"wo_t{pre}{q}"]
                    if t > 0:
                        xs = x[i] + v[i] * (t - 600) * 1e-6
                        row.append((side, xs, float(r[f"wo_{pre}{q}"]), c[i] + psi_g[i] * (xs - x[i]), psi_g[i]))
            else:
                row.append((side, x[i], float(r[raw_col]), c[i], psi_g[i]))
        ticks.append(row)
    return ticks, x, c


def analyze(path, p, k0, verbose):
    d = pd.read_csv(path)
    rows = []
    for i0, i1 in straight_segments(d["motion_state"].values, d["dist"].values):
        q = d.iloc[i0:i1 + 1].reset_index(drop=True)
        if q["dist"].iloc[-1] < 150:
            continue
        p.k0 = k0
        det = StrPostDetector(p)
        ticks, x, c = sample_stream(q)
        pairs, preds = [], []  # preds: 組ができる直前の読みの座標の推測と、来た組の δ の差
        pred_at = {}

        def on_tick(det, i):
            if det.n_pairs >= 1:
                row = ticks[i]
                pred_at[i] = det.now_delta_pred(row[0][1], row[0][3], row[0][4])

        def on_pair(det, i):
            pairs.append((det.pos, det.delta, det.delta_gyro, det.psi0, det.n_psi))
            if i in pred_at:
                preds.append(det.delta - pred_at[i])
        run_segment(det, q, ticks, x, c, on_pair=on_pair, on_tick=on_tick)
        # ファームの組(spe_delta、spe_seq が進んだ tick)との照合
        fw = []
        if "spe_seq" in q:
            sq = q["spe_seq"].values
            for i in np.where(np.diff(sq) != 0)[0] + 1:
                fw.append((float(q["dist"].iloc[i]), float(q["spe_delta"].iloc[i])))
        fw_diff = [fd - pp[1] for fx, fd in fw for pp in pairs if abs(fx - pp[0]) < 20]
        # k0: 壁からの引き継ぎ(実際の横位置)をジャイロで最初の組まで進めた推測と、来た組の δ の差
        # (実際の mm)に gain を掛けて読みの座標へ戻し、入れた k0 に足す。壁の終わりで機体が平行
        # (psi_we ≈ 0 = 壁の区間の終わりの ang_kf_sum と ψ0 の差)な走行の値を使う。
        k0_est = psi_we = float("nan")
        d_wall = det.seeds[0][1] if det.seeds else float("nan")
        if det.seeds and preds and pairs and pairs[0][0] > det.seeds[0][0]:
            k0_est = k0 + p.gain * preds[0]
            psi_we = math.degrees(det.seeds[0][2] - det.psi0)
        D = np.array([pp[1] for pp in pairs])
        P = np.array([pp[0] for pp in pairs])
        if len(pairs) >= 2:
            slope, dd0 = np.polyfit(P, D, 1)
            res = float(np.std(D - (dd0 + slope * P), ddof=2)) if len(D) > 2 else float("nan")
        else:
            slope = dd0 = res = float("nan")
        row = dict(file=path.split("/")[-1], seg=f"{i0}-{i1}", n=len(pairs), nL=sum(1 for s, _ in det.edges if s == 0),
                   nR=sum(1 for s, _ in det.edges if s == 1),
                   d_mean=float(D.mean()) if len(D) else float("nan"), d_std=float(D.std(ddof=1)) if len(D) > 1 else float("nan"),
                   eps=math.degrees(math.atan(slope)) if len(pairs) >= 2 else float("nan"), res=res,
                   psi0=math.degrees(det.psi0) if det.n_psi >= 1 else float("nan"),
                   pred=float(np.sqrt(np.mean(np.square(preds)))) if preds else float("nan"),
                   fw_n=len(fw), fw_diff=float(np.max(np.abs(fw_diff))) if fw_diff else float("nan"),
                   d_wall=d_wall, psi_we=psi_we, k0=k0_est,
                   v=float(q["ideal_v"].max()), hf=float((q["wo_n"] == 4).mean()) if "wo_n" in q else 0.0)
        rows.append(row)
        if verbose:
            print(f"--- {row['file']} seg {row['seg']} v {row['v']:.0f} hf {row['hf']:.2f}")
            print("  L edges: " + " ".join(f"{e:7.2f}" for s, e in det.edges if s == 0))
            print("  R edges: " + " ".join(f"{e:7.2f}" for s, e in det.edges if s == 1))
            print("  pairs (pos: δ, δ_gyro, ψ0 deg): " + " ".join(f"{pp[0]:6.1f}:{pp[1]:+5.2f},{pp[2]:+5.2f},{math.degrees(pp[3]):+5.2f}" for pp in pairs))
            if det.seeds:
                print("  壁からの引き継ぎ (x: δ_wall, ψ0 deg): " + " ".join(f"{sx:6.1f}:{sd:+5.2f},{math.degrees(sp0):+5.2f}" for sx, sd, sp0 in det.seeds))
            if preds:
                print("  次の組の推測の誤差(組 − 推測): " + " ".join(f"{e:+.2f}" for e in preds))
            if fw:
                print("  firmware spe_delta: " + " ".join(f"{fx:6.1f}:{fd:+5.2f}" for fx, fd in fw))
    return rows


def dump(path, p):
    """ホスト検証用: 1 行 1 サンプル "seg side x y c psi"、区間の最初に "seg ARM"。
    続けて期待する組 "PAIR seg pos delta psi0_deg n_psi"。"""
    d = pd.read_csv(path)
    print(f"PARAM {p.rel_thr} {p.peak_min[0]:.4f} {p.peak_min[1]:.4f} {p.low_ratio} {p.contrast_min} "
          f"{p.rise_max} {p.fall_max} {p.tol} {p.gate_kmax} {p.kappa / p.gain} {p.psi0_w} {p.gain} {p.k0}")
    for i0, i1 in straight_segments(d["motion_state"].values, d["dist"].values):
        q = d.iloc[i0:i1 + 1].reset_index(drop=True)
        if q["dist"].iloc[-1] < 150:
            continue
        seg = f"{path.split('/')[-1]}:{i0}"
        det = StrPostDetector(p)
        ticks, x, c = sample_stream(q)
        print(f"{seg} ARM")
        # サンプルは検知器に入れる直前に出す(update を包む)
        orig = det.update

        def upd(side, xs, y, cs, ps, orig=orig):
            print(f"{seg} {side} {xs:.5f} {y:.1f} {cs:.6f} {ps:.7f}")
            return orig(side, xs, y, cs, ps)
        det.update = upd
        orig_seed = det.seed

        def sd(xw, cw, psi, delta_read, psi0, w, orig_seed=orig_seed):
            print(f"{seg} SEED {xw:.5f} {cw:.6f} {psi:.7f} {delta_read:.5f} {psi0:.7f}")
            return orig_seed(xw, cw, psi, delta_read, psi0, w)
        det.seed = sd
        run_segment(det, q, ticks, x, c,
                    on_pair=lambda det, i: print(f"PAIR {seg} {det.pos:.4f} {det.delta:.5f} {math.degrees(det.psi0):.5f} {det.n_psi}"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--k0", type=float, default=Params.k0, help="[mm、読み] 真ん中 にいるときの読み(左右センサーの取り付け差)")
    ap.add_argument("--kappa", type=float, default=Params.kappa, help="[mm/rad、読み]")
    ap.add_argument("--gain", type=float, default=Params.gain, help="機体が 1mm 動いたときの読みの変化")
    ap.add_argument("--rel-thr", type=float, default=Params.rel_thr)
    ap.add_argument("--dump", action="store_true", help="ホスト検証用のサンプル列を出す")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    p = Params()
    p.kappa = a.kappa
    p.gain = a.gain
    p.rel_thr = a.rel_thr
    p.k0 = a.k0
    if a.dump:
        for f in a.files:
            dump(f, p)
        return
    rows = []
    for f in a.files:
        rows += analyze(f, p, a.k0, a.verbose)
    if rows:
        df = pd.DataFrame(rows)
        pd.set_option("display.width", 200)
        print(df.round(3).to_string(index=False))
        ok = df[df.n >= 2]
        if len(ok):
            print(f"\nδ の平均 {ok.d_mean.mean():+.2f} (σ {ok.d_mean.std(ddof=1) if len(ok) > 1 else 0:.2f})  "
                  f"組ごとの残差 {ok.res.mean():.2f}  次の組の推測の誤差 rms {ok.pred.mean():.2f}  "
                  f"k0 の推定 {ok.k0.mean():+.2f} (n {ok.k0.notna().sum()}、壁の終わりの向き ψ_true {ok.psi_we.abs().mean():.2f}°)")


if __name__ == "__main__":
    main()
