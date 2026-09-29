# 経路コストの実験(2026-09-29)

最短走行の経路生成(`MainTask::path_run()` の「右」)と、探索のサブゴール選び(`Adachi::update()`)を、
重みパターンの値・段数・方式を変えて比べたときの道具一式。ファームのソースは読むだけで、書き換えた写しは
`build.py` がこのフォルダの中に作る(`.gitignore` 済み)。ファームには何も入れていない。

迷路は `tools/param_tuner/maze_data`・`profile`・`maze_logs` から重複(壁の違い 3 枚以下)を除いた 21 本
(`px/ds.py`)、走行モードは run_prf の 1 / 3 / 5 / 11 / 16(`px/ev.py` の `EXECS`)= 105 ケース。

## 使い方

```bash
python3 px/build.py && python3 px2/build.py     # 先にファームを cmake configure しておく(ArduinoJson)
cd px
python3 verify.py        # 実験用ビルドが tools/path_sim/build/path_sim と同じ結果か
python3 rs.py 3000 1 a   # 重み 6 個を 3000 通り(約 8 分、rs_a.pkl)
python3 an.py a; python3 an2.py a
python3 t2.py            # 表にした重み / あとから安い値で直す版 / 分岐候補の幅
python3 phys.py          # 加速度とターン時間から作った 16 段の表
python3 opt.py           # タイム最小の探索と現行の比較(opt.pkl)
python3 opt2.py          # 速い版の確認と計算時間
python3 tr.py            # ファームの経路がタイム最小の探索の辺でなぞれるか
python3 se.py [名前…]    # 探索のサブゴールの選び方の比較(se.pkl に覚える。名前を渡すとそれだけやり直す)
python3 dpcost.py        # 探索の途中で呼んだときの計算量
```

## 何があるか

| ファイル | 中身 |
|---|---|
| `px/build.py` | 重みを外から入れられるようにした `logic.hpp` / `adachi.cpp` / `search_main.cpp` の写しを作ってビルド |
| `px/px_path.cpp` | 重みパターンをまとめて評価(1 パターンぶんの path_create → timebase_path_create) |
| `px2/build.py` | 上に加えて: 重みを 16 段の表に、あとから安い値で直す版、分岐候補の幅、素の経路の記録、探索へのフック |
| `px2/opt_core.hpp` | タイム最小の探索の試作(分かりやすさ優先) |
| `px2/opt_core2.hpp` | 同じ探索の速い版(整数の鍵・表引き・ハッシュ表・2 分ヒープ・残り時間の下限)。ファームへ持っていくならこちら |
| `px2/px_opt.cpp` | 上の 2 つを呼ぶコマンド。素の経路をファームの変換と `calc_goal_time` に通す確認も持つ |

## タイム最小の探索の考え方

経路を「直線 + ターン」の区間の列として、区間を辺にした最短経路問題を解く。辺の重みは `calc_goal_time()` の
1 区間ぶんと同じ計算(`seg_raw()` がその写し)。節点は「ターンを終えた位置・向き・直進か斜めか・出口速度・
次の区間への約束」。約束は `calc_goal_time()` の先読み(次が 直線 > 0 + Large / Orval なら速いターン)を
辺の重みへ入れるためのもの。辺の作り方は `convert_large_path()` / `diagonalPath()` の規則の写し:

- ターン 1 個 = Large、同じ向き 2 個 = Orval
- 向きが交互に続く組 = 斜め。入口は Dia45(最初が同じ向き 2 個なら Dia135)、出口も同じ。途中の同じ向き 2 個は Dia90
- 同じ向き 3 個(その場で 270° 回る形)は Normal が残るので対象外

求めた経路は素の経路(path_create が作る形)に直して、ファームの変換と `calc_goal_time()` に通し、探索の
タイムと同じになることを確かめている(105 ケースすべて一致)。ファームの 5 パターンが出した経路 525 本も
この探索の辺でなぞれて同じタイムになる。

## 注意

- `adachi.hpp` は隣の `logic.hpp` を読むので、`logic.hpp` にメンバーを足す実験では `adachi.hpp` の写しも
  `inc/` に置く(`px2/build.py`)。置かないと、その翻訳単位だけ元のクラス配置で `set_param()` が展開され、
  重みが切り替わらない(一度これで探索の比較を取り違えた)。
- `calc_goal_time()` は最後の直線(最後のターン〜ゴール)を足していない。`count_final: false` がファームと
  同じ数え方、`true` が足した値。
