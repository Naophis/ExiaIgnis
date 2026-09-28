# maze_data

迷路タブ(Param Console)の「過去の迷路 (maze_data)」と、経路・探索シミュレータの検証に使う迷路。
形式はどれも `maze_data:` の下に `wall`(idx = x * maze_size + y、下位 4bit が壁: N=1 / E=2 / W=4 / S=8)と
`goal`(ゴール区画の [x, y] の並び)、`maze_size`、`max_step_val`。

## japan20xx*.yaml

以前からあった全日本の迷路(出典不明)。`hef` は 32×32、`hes` は 16×16。

## 32MM20xxHX.yaml / 32_*.yaml

[kerikun11/micromouse-maze-data](https://github.com/kerikun11/micromouse-maze-data)(MIT License、
Copyright (c) 2020 Ryotaro Onuki、`LICENSE-micromouse-maze-data`)の `data/32*.maze` を
`../maze_ascii_convert.py` で変換したもの(2026-09-29)。ゴールは元の `G` の区画。

- `32MM2008HX`〜`32MM2023HX`: 全日本(2020 年は元データに無い)。2008・2009 年はゴールが 1 区画、2010 年は 2×2、2011 年以降は 3×3。
- `32_DFS_01` / `32_fake` / `32_farm` / `32_no_wall` / `32_test_01`: 元リポジトリの試験用の迷路。
- `32_unknown.maze` は未知の壁を含み、この形式では表せないので取り込んでいない。

既存の `japan20xxhef.yaml` と同じ年があるので、評価で両方を数えないこと:

| 年 | 既存との関係 |
|---|---|
| 2010 / 2011 / 2012 / 2014 / 2015 / 2018 / 2019 | 壁・ゴールとも完全に同じ |
| 2013 | 壁 3 枚が違う((8,9)-(8,10)、(9,3)-(9,4)、(15,12)-(16,12))。どちらが正しいかは未確認 |
| 2016 | 壁 3 枚が違う((11,27)-(11,28)、(29,3)-(29,4)、(29,4)-(30,4))。同上 |
| 2017 | 壁 1 枚が違う((30,0)-(31,0))。同上 |
