# DitherPwm — Hardware PWM + DMA 時間方向ディザによる高分解能 duty

`include/driver/dither_pwm.hpp` / `src/driver/dither_pwm.cpp` / `examples/dither_pwm_example.cpp`

STM32G4 HRTIM のようにエッジそのものを sub-ns にするのではなく、
**PWM 周期ごとに compare 値を N / N+1 で切り替え、複数周期の平均で
1 count 未満の duty を表現する**。RP2350 の Hardware PWM をキャリアに使い、
compare 更新は DMA だけで完結させる。CPU は制御周期ごとにサンプル列を
書き足すだけで、DMA の再起動経路には入らない。

```
control loop (Core1, 1〜4 kHz)
  ↓ fractional duty (Q16.16 count)
  ↓ update(): 1次誤差拡散で samples_per_tick 個の CC 語を生成
SRAM ring (256 語, 1024 B 整列)  ←── CPU は DMA read 位置の lead 先に書く
  ↓ DMA (1 ch / slice, DREQ = その slice の wrap, RING_SIZE で自動ラップ, ENDLESS)
slice.cc (A | B<<16)               ←── 1 周期 1 転送、次の wrap で latch
  ↓
Hardware PWM 出力
```

## 1. 仕様確認(RP2350 datasheet / Pico SDK 2.2.0)

実装前に確認した項目と結論。すべて成立する。

| 項目 | 根拠 | 結論 |
|---|---|---|
| CC/TOP は double-buffer、書き込みは次の wrap で反映 | datasheet 12.5.2.3 / 12.5 機能一覧「Wrap and level registers are double buffered and can be changed race-free」/ SDK `pwm_set_chan_level` doc | 周期途中に何度 CC を書いてもグリッチしない。同一周期内の複数書き込みは最後の 1 つだけ有効 |
| wrap ごとに 1 サイクルの DREQ | datasheet 12.5.2.7「The same pulse which sets the interrupt flag in INTR is also available as a one-cycle data request … the DMA can efficiently stream data to a PWM slice at a rate of one transfer per counter period」 | データシートがこの用途を明記。`pwm_get_dreq(slice)` = `DREQ_PWM_WRAPn` |
| CC は 32 bit で A/B 同居 | regs `PWM_CHx_CC` (A=[15:0], B=[31:16]) | 1 転送で A/B が同時に更新される(H ブリッジの方向反転もグリッチなし) |
| 複数 slice の同時開始 | 12.5.2.8「Use this register (EN) to start and stop several slices simultaneously. If two slices with the same output frequency start at the same time, they run in perfect lockstep」 | `hw_set_bits(&pwm_hw->en, mask)` で同一サイクル開始。DREQ も同時に出る |
| DMA read アドレスの自動ラップ | `channel_config_set_ring(c, false, 10)`: 下位 10 bit だけが変化(1024 B 境界に整列必須) | リングを一周しても DMA は止まらない |
| 無限転送 | `TRANS_COUNT.MODE = 0xF ENDLESS`(12.6.2.2、RP2350 追加)。`dma_encode_endless_transfer_count()` | 完了イベントが無い = CPU が再起動する経路が存在しない |
| DREQ は 6 bit 飽和クレジット | 12.6.4.2 | DMA が遅れてもクレジットが溜まりあとで追いつく。転送は落ちないが同一周期内でまとめて書かれ、最後以外は無効(= サンプル飛び) |
| クレジット残の観測/クリア | `CHx_DBG_CTDREQ`: read でカウンタ、write でクリア+ハンドシェイク再初期化 | 起動時にクリア。動作中は `dreq_backlog_max` として監視 |
| DMA 優先度 | `channel_config_set_high_priority()`; バス側は BUSCTRL.BUS_PRIORITY | CC 更新チャネルのみ高優先度にする |

**成立しない/ハードでは保証できない点(近似せず、設計で扱う):**

- **相間の同一 wrap 反映は「DMA が周期内に間に合う限り」成立**する。HRTIM の
  update-enable のような全相一括 latch ビットは無い。各 slice の DMA が同じ
  wrap で DREQ を受け、数サイクル以内に両方の CC を書き、次の wrap で同時に
  latch される。DMA が 1 周期(1500 サイクル)以上遅れた場合だけ 1 周期ずれる。
  `stats.skew_max` と `dreq_backlog_max` で監視できる。
- **DMA underrun のハードフラグは無い**(DREQ 駆動の書き込みでは「不足」という
  概念が無く、単に古い CC が出続ける)。検出はソフトで行う(§4)。
- **ping-pong の DMA chaining は使わない**。RP2350 では ENDLESS + RING で「1 本の
  DMA が永久に回る」ので、chaining で再ロードする必要が無い。chaining 方式は
  再ロード用チャネルという可動部品が増える(RP2040 向け。`bldc_actuator.cpp` の
  chained control block がそれ)。要求の本質「CPU が再起動のクリティカルパスに
  入らない」はリング方式の方が強く満たす。

## 2. リングと更新アルゴリズム

```
サンプル番号(単調増加):
 ... 消費済み | read | ← lead → | commit(前tick) | guard(前tick) | 1周前の古いデータ ...
```

`update()`(制御周期ごと 1 回):

1. 全 slice の `READ_ADDR` を同一時点で読み、リング index に変換。共通の
   `read_count` は最も進んでいる slice に合わせる(遅れている slice に対しても
   `start ≥ read+lead` が保たれる = 安全側)。
2. `start = max(前 tick の commit 末尾, read + lead)`、
   `end = read + lead + samples_per_tick`。
3. 変調器の状態を「前 tick の commit 末尾」の snapshot から復元し、
   `[start, end)` を新しい指令で埋める(commit)。末尾で snapshot を更新。
4. 続けて `[end, end+guard)` に同じ指令で書き足す(guard)。次 tick が間に合えば
   上書きされ、遅れればそのまま出る(= 最後の duty を保持)。
5. `start > 前 commit 末尾` だった場合、DMA が guard を `delta` サンプル食って
   いるので、変調器を guard の指令で `delta` 回進めてから commit する
   (誤差拡散の連続性を保つ)。`delta > guard` なら 1 周前の古いデータが再生された
   = underrun としてカウント。

**レイテンシ**: 指令 → 出力 = `lead_samples` 周期 + 1 周期(latch)。既定 lead=4 で
100 kHz なら 50 µs(ホストシミュレーションで確認)。これは制御ループから見える遅れなので
大きくしすぎない。現行の CC 直書き(次 wrap で反映 = 最大 10 µs)より 40 µs 増える。

**リング長の考え方**(既定 256 サンプル = 2.56 ms @100 kHz):

| 制御周期 | samples_per_tick | lead | guard(既定=M) | 1 tick で触る範囲 | 余裕 |
|---|---|---|---|---|---|
| 4 kHz | 25 | 4 | 25 | 54 | 202 サンプル(2.0 ms) |
| 1 kHz | 100 | 4 | 100 | 204 | 52 サンプル(0.5 ms) |

制約は `lead + M + guard < 256`。1 kHz で guard を長くしたい場合は
`kRingSizeBits` を 11(512 サンプル)にする(メモリ +1 KB/slice)。

**CPU コスト**: 1 サンプルあたり変調 2 回 + pack + store ≈ 8〜10 命令。
4 kHz/25+25 サンプル × 2 slice ≈ 6 µs/250 µs ≈ 2.5 %。1 kHz/100+100 × 2 ≈ 27 µs/ms。
guard を短くすれば比例して減る。

## 3. 安全性

| 事象 | 挙動 |
|---|---|
| CPU が遅れる(≤ guard) | DMA は guard 領域(最後の指令)を出し続ける。復帰時に変調器を fast-forward して連続 |
| CPU が遅れる(> guard) | 1 周前(2.56 ms 前)の duty 列が再生される。PWM は途切れない。`underrun_events` 加算 |
| CPU が完全停止 | DMA はリングを永久に回す(最後に書いた 2.56 ms 分の duty を繰り返す) |
| DMA がバス競合で遅れる | PWM は前回 CC のまま継続。クレジットが溜まり後でまとめて転送(サンプル飛び) |
| DMA が停止(abort 等) | CC は最後の値のまま。`stall_ticks` 加算 |
| 異常値 | リングに書く値は `max_level`(既定 TOP+1)でクランプ。GPIO HIGH 固定は起きない |
| 停止したい | `force_static(idx, 0, 0)`(リング全体と CC を 0)、または `stop(true)`(DMA abort → CC=0 → 2 周期待って slice 停止) |

PIO 方式で問題になる「FIFO underrun で SM が `pull block` に止まり GPIO が
不定状態で固まる」経路は存在しない。

## 4. 検出

`DitherStats`(slice ごと):

- `late_samples / late_max`: DMA が guard に食い込んだ量(CPU 遅延の大きさ)
- `underrun_events / underrun_samples`: guard を超えた回数/量
- `stall_ticks`: read index が動かなかった tick(PWM 停止か DMA 停止)
- `dreq_backlog_max`: `DBG_CTDREQ` の最大値。0 が正常、≥1 は DMA が周期内に
  間に合わなかった証拠
- `skew_max`: slice 間の read index ずれ。0 なら全相が同一 wrap で更新されている

LoggingTask に `late_max` / `dreq_backlog_max` / `skew_max` を 1 kHz で流せば
実走行中の余裕が定量化できる。

## 5. DMA / バス競合について(現状のファーム)

- DMA を使う既存モジュール: SPI1(ジャイロ/エンコーダ/バッテリ ADC、
  `sensing_task.cpp` / `as5147p.cpp`)。1 kHz ごとに数バイトの短いバースト。
  BLDC 用 6 チャネル(`bldc_actuator.cpp`)は現在未使用。USB は自前 DMA で
  システム DMA を使わない。PIO(DShot/AM32)は FIFO 直叩き。
- CC 更新チャネルは `high_priority=true`(既定)。転送は 100 kHz × 2 slice × 4 B
  = 800 kB/s と極小で、優先度を上げても他に影響しない。
- PWM と SPI は同じ APB ブリッジ配下なので同時アクセスで数サイクル待つが、
  1 周期 1500 サイクルに対して無視できる。
- リングは必ず SRAM(`g_ring`, .bss)。PSRAM(`psram_heap`)に置くと XIP 経由になり
  DMA が待たされる。`init()` でアドレス範囲を検査している。
- Core1 制御 IRQ の `update()` は `.time_critical.dither_pwm` に置く(SRAM 実行)。

## 6. ホストシミュレーション(実機不要、実施済み)

`tests/dither_pwm_host/run.sh` は Pico SDK のレジスタ/関数を `stub/` で模擬し、
実機と同じ `src/driver/dither_pwm.cpp` を x86_64 でそのまま動かす。
「DMA = 1 周期ごとにリング語を CC へ転送して read_addr++(1024 B ラップ)」
「PWM = 周期 k に書かれた CC は周期 k+1 の出力」を再現している。2026-09-13 に全 5 テスト PASS。

| テスト | 内容 | 結果 |
|---|---|---|
| T1 | 定常指令 6 種の平均・値域・レイテンシ | 平均誤差 ≤ 1e-4 count、値は常に {N, N+1}、新指令の初出は tick + lead(4) + 1 周期 |
| T2 | 制御 tick を 1 回/2 回スキップ | 1 回: guard で最後の duty を保持、累積誤差 < 1 count(連続性維持)。2 回: underrun 1 回/25 サンプルを正しく計上、出力は途切れず |
| T3 | 2 slice 同時駆動 | 出力列が完全一致、skew_max=0 |
| T4 | M=100(1 kHz)、force_static、stop→start | 平均一致、force_static は 2 周期以内に反映、stop で CC=0、再開後も統計クリーン |
| T5 | tick ジッタ ±3 周期 × 400 tick | underrun 0、累積誤差 < 1 count |

## 7. 実機検証手順(examples/dither_pwm_example.cpp)

```
cmake --build build --target dither_pwm_example
~/.local/bin/picotool load -f -u -x build/dither_pwm_example.bin -t bin -o 0x10000000
./serial_monitor.sh
```

既定ピンは左モータ(GPIO4/5 = slice 2)と右モータ(GPIO6/7 = slice 3)。
低 duty(4 %前後)だが通電はするので、ドライバの nSLEEP を落とすかモータを外す。

1. **[S] 起動時の自己検査**(ハード不要): 変調器を 65536 サンプル回し、
   平均が指令に一致(err < 1e-4 count)し、25 周期パターンで `501` が 9 個
   均等に散ることを USB 出力で確認。
2. **[H1] オシロ**: GPIO4 の H 幅を infinite persistence で見ると立下りが
   1 count 差の 2 本に分かれる。100 kHz では 1 count = 6.67 ns で見づらいので
   `SLOW_SCOPE_MODE 1`(clkdiv 16、1 count = 107 ns)で確認する。
   `REF_ON_B 1` にすると GPIO5 にディザ無しの整数部が出るので、A と直接比較できる。
3. **[H2] 平均電圧**: GPIO4 に RC(10 kΩ + 1 µF)を付け DC を DMM で読む。
   指令 60.00 / 60.25 / 60.50 / 60.75 / 61.00 count(2 s ごと)で
   3.3 V × 0.25/1500 = 0.55 mV 刻みに動く。10〜15 s のランプ区間は
   1/256 count 刻みなので滑らかに上がる(通常 PWM ならこの区間は 1 段の階段)。
4. **[H3] ロジアナ**(≥150 MS/s): 1 ms 分の H 幅ヒストグラム。N と N+1 の
   比率が小数部に一致。
5. **[F] fail-static**: 15 s サイクルごとに制御 tick を 12 回(3 ms @4 kHz)
   飛ばす。出力は途切れず、`late`/`underrun` が増えることを stats で確認。
   スコープでは PWM が継続し duty が保持されるのが見える。
6. **[M] slice 間同期**: GPIO4 と GPIO6 を 2 ch で見て立上りが一致し、
   `skew_max=0` であること。

## 8. ExiaIgnis 本体への組み込み方針(未実施)

`MotorActuator` を DitherPwm ベースに置き換える。

- `init()`: `DitherPwm::init()`(slice_L, slice_R、top=clk/MotorHz−1、
  samples_per_tick = ceil(MotorHz / 制御周波数)、lead 4、guard = M)→ `start()`。
  現行の `pwm_set_wrap` / `pwm_set_enabled` 直叩きは廃止。
- `apply(duty_l, duty_r)`: `level_q16 = |duty|/100 × (top+1) × 65536` を
  符号に応じて A/B に振り分けて `set_levels_q16()`、最後に `update()` を 1 回。
  現行の `pwm_set_chan_level()` は削除(DMA に上書きされるため残すと無意味)。
- `motor_disable()`: `stop(true)`。`motor_enable()`: `start()`。
- `motor_debug_mode` / `sys_id` の最終段オーバーライドは `set_levels_q16` の
  手前で値を差し替えるだけ(`set_next_duty()` の構造は変えない)。
- 制御周期を 4 kHz に上げる場合は PlanningTask の TIMER1 周期と
  `samples_per_tick` を合わせるだけ。
- 注意: MPQ6612A の最小オン時間 200 ns(100 kHz で duty 2 %)より下は
  ディザしても意味が無い。`max_level` ではなく下限側のクランプは
  ControlLaw 側の責務のまま。
