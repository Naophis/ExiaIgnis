"use client";

import type { CSSProperties, ReactNode } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import type { ConnectionStatus, PortInfo } from "@/lib/serial-manager";

const STATUS_LABEL: Record<ConnectionStatus, string> = {
  connected: "Connected",
  connecting: "Connecting...",
  disconnected: "Searching for device...",
};

const STATUS_VARIANT: Record<
  ConnectionStatus,
  "default" | "secondary" | "outline" | "destructive"
> = {
  connected: "default",
  connecting: "secondary",
  disconnected: "outline",
};

interface Props {
  ports: PortInfo[];
  connectedPath: string | null;
  status: ConnectionStatus;
  autoConnect: boolean;
  flashing: boolean;
  onDisconnect: () => void;
  onEnableAutoConnect: () => void;
  onFlash: () => void;
  // 右ペインの「コンソール / プロット」切り替え。右ペイン内に置くと1行分の高さを
  // 食うので、ヘッダーバーの空いている中央に出す(page.tsx が既定ビューのときだけ渡す)。
  tabs?: ReactNode;
  // 機体の切り替え(components/machine-bar.tsx)
  machines?: ReactNode;
  // 表示中の機体の色(左の縁に出す)
  accentColor?: string;
  // 右端のボタンの手前に出すもの(テーマカラーの切り替え)
  actions?: ReactNode;
}

export function PortPanel({
  ports,
  connectedPath,
  status,
  autoConnect,
  flashing,
  onDisconnect,
  onEnableAutoConnect,
  onFlash,
  tabs,
  machines,
  accentColor,
  actions,
}: Props) {
  const label = !autoConnect
    ? "Disconnected (auto-connect paused)"
    : STATUS_LABEL[status];
  const variant = !autoConnect ? "destructive" : STATUS_VARIANT[status];

  return (
    // 見た目は globals.css の .cb-header(左の帯 = 表示中の機体の色、六角形の網、下の縁の光)。
    // テーマの窓がこの下へはみ出して出るので、overflow は切らない。
    <div
      className="cb-header flex shrink-0 flex-wrap items-center gap-x-2 gap-y-1 py-1 pr-2.5 pl-3 text-sm"
      style={accentColor ? ({ "--cb-machine": accentColor } as CSSProperties) : undefined}
    >
      {/* 銘。2 行にして、ヘッダーの高さ(ボタン 1 行分)の中に収める */}
      <div className="flex shrink-0 flex-col gap-[3px] pr-1 leading-none text-accent-gold">
        <span className="font-display text-[7.5px] tracking-[0.34em] opacity-70">CELESTIAL BEING</span>
        <span className="font-display text-[11.5px] tracking-[0.1em]">EXIA PARAM CONSOLE</span>
      </div>
      <Badge variant={variant}>{label}</Badge>
      {/* 未検出のときは左のバッジ(Searching for device...)が同じことを言っているので出さない */}
      {(connectedPath ?? ports[0]?.path) && (
        <span className="text-muted-foreground">{connectedPath ?? `検出済み: ${ports[0].path}`}</span>
      )}
      {machines}
      {tabs && <div className="ml-2 flex gap-1">{tabs}</div>}
      <div className="flex-1" />
      {actions}
      <Button size="sm" variant="outline" disabled={flashing} onClick={onFlash}>
        {flashing ? "Flashing..." : "Flash"}
      </Button>
      {status === "disconnected" && !autoConnect ? (
        <Button size="sm" onClick={onEnableAutoConnect}>
          自動接続を再開
        </Button>
      ) : (
        <Button size="sm" variant="destructive" onClick={onDisconnect}>
          Disconnect
        </Button>
      )}
    </div>
  );
}
