"use client";

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
  onDisconnect: () => void;
  onEnableAutoConnect: () => void;
}

export function PortPanel({
  ports,
  connectedPath,
  status,
  autoConnect,
  onDisconnect,
  onEnableAutoConnect,
}: Props) {
  const label = !autoConnect
    ? "Disconnected (auto-connect paused)"
    : STATUS_LABEL[status];
  const variant = !autoConnect ? "destructive" : STATUS_VARIANT[status];

  return (
    <div className="flex shrink-0 items-center gap-2 rounded-xl border-l-2 border-l-accent-gold bg-card px-3 py-1.5 text-sm ring-1 ring-primary/20">
      <span className="font-semibold tracking-wide text-accent-gold">Exia PARAM CONSOLE</span>
      <span className="text-muted-foreground">Pico (ttyACM*)</span>
      <Badge variant={variant}>{label}</Badge>
      <span className="text-muted-foreground">
        {connectedPath ?? (ports[0]?.path ? `検出済み: ${ports[0].path}` : "デバイス未検出")}
      </span>
      <div className="flex-1" />
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
