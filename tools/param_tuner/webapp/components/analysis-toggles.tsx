"use client";

// 解析トグルのチップ行。プロットタブと詳細ログ解析ページで同じものを使う。
// 設定の実体は lib/use-analysis.ts の useAnalysisSettings()。
// 縦に積むとプロット領域を削るので、1行に横並び(有効化した解析だけパラメータが
// 横に展開し、足りなければ折り返す)を崩さないこと。

import { Button } from "@/components/ui/button";
import { TROUGH_COLUMNS, type AnalysisSettings } from "@/lib/use-analysis";

export function AnalysisToggles({
  settings: s,
  // 旋回出口の「集計 直近 N 本」(複数ログ横断)を出すか。詳細ページは1本を
  // 精査する画面なので出さない。
  showTurnExitSummary = false,
  summaryBusy = false,
  onRefreshSummary,
}: {
  settings: AnalysisSettings;
  showTurnExitSummary?: boolean;
  summaryBusy?: boolean;
  onRefreshSummary?: () => void;
}) {
  return (
    <>
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="analyze_sensor_drop.py 相当: 指定 motion_state 中のセンサー距離の急落(low/high しきい値)を検出">
            <input type="checkbox" checked={s.drop.enabled} onChange={(e) => s.drop.setEnabled(e.target.checked)} />
            ドロップ
          </label>
          {s.drop.enabled && (
            <>
              <label className="flex items-center gap-1">
                motion_state
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.drop.motionState}
                  onChange={(e) => s.drop.setMotionState(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                low
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.drop.low}
                  onChange={(e) => s.drop.setLow(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                high
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.drop.high}
                  onChange={(e) => s.drop.setHigh(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                <input type="checkbox" checked={s.drop.left} onChange={(e) => s.drop.setLeft(e.target.checked)} />
                left45_d
              </label>
              <label className="flex items-center gap-1">
                <input type="checkbox" checked={s.drop.right} onChange={(e) => s.drop.setRight(e.target.checked)} />
                right45_d
              </label>
            </>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="analyze_motion_state_transitions.py 相当: 指定 motion_state の開始/終了行と、その時の45°読みの壁面点">
            <input
              type="checkbox"
              checked={s.transition.enabled}
              onChange={(e) => s.transition.setEnabled(e.target.checked)}
            />
            状態遷移
          </label>
          {s.transition.enabled && (
            <label className="flex items-center gap-1">
              states
              <input
                type="text"
                className="w-24 rounded border border-border bg-background px-1"
                value={s.transition.states}
                onChange={(e) => s.transition.setStates(e.target.value)}
              />
            </label>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="analyze_sensor_trough.py 相当: 指定区間のセンサー距離の谷(最接近)と復帰">
            <input type="checkbox" checked={s.trough.enabled} onChange={(e) => s.trough.setEnabled(e.target.checked)} />
            トラフ
          </label>
          {s.trough.enabled && (
            <>
              <label className="flex items-center gap-1">
                motion_state終了
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.trough.motionState}
                  onChange={(e) => s.trough.setMotionState(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                探索区間states
                <input
                  type="text"
                  className="w-16 rounded border border-border bg-background px-1"
                  value={s.trough.states}
                  onChange={(e) => s.trough.setStates(e.target.value)}
                />
              </label>
              <label className="flex items-center gap-1">
                eps
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.trough.eps}
                  onChange={(e) => s.trough.setEps(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                median窓
                <input
                  type="number"
                  min={1}
                  step={2}
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.trough.medianWindow}
                  onChange={(e) => s.trough.setMedianWindow(parseInt(e.target.value, 10))}
                />
              </label>
              {TROUGH_COLUMNS.map((col) => (
                <label key={col} className="flex items-center gap-1">
                  <input
                    type="checkbox"
                    checked={s.trough.columns[col]}
                    onChange={(e) => s.trough.toggleColumn(col, e.target.checked)}
                  />
                  {col}
                </label>
              ))}
            </>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="wall_off_edge_check.py 相当: 壁切れ検出のレベル判定/気づいた点/逆算エッジの3段を表示">
            <input type="checkbox" checked={s.wallOff.enabled} onChange={(e) => s.wallOff.setEnabled(e.target.checked)} />
            壁切れエッジ
          </label>
          {s.wallOff.enabled && (
            <>
              <label className="flex items-center gap-1">
                motion_states
                <input
                  type="text"
                  className="w-16 rounded border border-border bg-background px-1"
                  value={s.wallOff.motionStates}
                  onChange={(e) => s.wallOff.setMotionStates(e.target.value)}
                />
              </label>
              <label className="flex items-center gap-1">
                baseline N
                <input
                  type="number"
                  min={1}
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.wallOff.baselineN}
                  onChange={(e) => s.wallOff.setBaselineN(parseInt(e.target.value, 10))}
                />
              </label>
              <label className="flex items-center gap-1">
                arm delta
                <input
                  type="number"
                  step={0.1}
                  className="w-16 rounded border border-border bg-background px-1"
                  value={s.wallOff.armDelta}
                  onChange={(e) => s.wallOff.setArmDelta(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                fit lo
                <input
                  type="number"
                  step={0.1}
                  className="w-16 rounded border border-border bg-background px-1"
                  value={s.wallOff.fitLo}
                  onChange={(e) => s.wallOff.setFitLo(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                fit hi
                <input
                  type="number"
                  step={0.1}
                  className="w-16 rounded border border-border bg-background px-1"
                  value={s.wallOff.fitHi}
                  onChange={(e) => s.wallOff.setFitHi(parseFloat(e.target.value))}
                />
              </label>
            </>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label
            className="flex items-center gap-1"
            title="turn_exit_check.py 相当: 旋回(SLALOM)ごとの追従状態と出口残差。duty_sen は片壁×2とヨーの見かけ横ずれで誇張されるので、壁読み値から出した wide(外側正)と yaw0 で判定する"
          >
            <input type="checkbox" checked={s.turnExit.enabled} onChange={(e) => s.turnExit.setEnabled(e.target.checked)} />
            旋回出口
          </label>
          {s.turnExit.enabled && showTurnExitSummary && (
            <>
              <label
                className="flex items-center gap-1"
                title="直近 N 本のログをサーバー側で集計(latest.csv は除外)。新しいログが保存されると自動で更新"
              >
                集計 直近
                <input
                  type="number"
                  min={1}
                  max={40}
                  className="w-14 rounded border border-border bg-background px-1"
                  value={s.turnExit.limit}
                  onChange={(e) => s.turnExit.setLimit(Math.min(40, Math.max(1, parseInt(e.target.value, 10) || 1)))}
                />
                本
              </label>
              <Button size="sm" variant="ghost" disabled={summaryBusy} onClick={onRefreshSummary}>
                再集計
              </Button>
            </>
          )}
        </div>
    </>
  );
}
