"use client";

import { useEffect, useRef } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";

interface Props {
  lines: string[];
  paused: boolean;
  onClear: () => void;
  onTogglePause: () => void;
}

export function ConsoleLog({ lines, paused, onClear, onTogglePause }: Props) {
  const bottomRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (paused) return;
    bottomRef.current?.scrollIntoView({ block: "end" });
  }, [lines, paused]);

  return (
    <Card className="flex flex-1 flex-col overflow-hidden">
      <CardHeader className="flex-row items-center justify-between space-y-0">
        <div className="flex items-center gap-2">
          <CardTitle>コンソール</CardTitle>
          {paused && <Badge variant="destructive">一時停止中</Badge>}
        </div>
        <div className="flex gap-1">
          <Button size="sm" variant="outline" onClick={onTogglePause}>
            {paused ? "再開" : "Pause"}
          </Button>
          <Button size="sm" variant="outline" onClick={onClear}>
            Clear
          </Button>
        </div>
      </CardHeader>
      <CardContent className="flex-1 overflow-hidden p-0">
        <ScrollArea className="h-full min-h-0 px-4 pb-4">
          <pre className="font-mono text-xs whitespace-pre-wrap break-all">{lines.join("\n")}</pre>
          <div ref={bottomRef} />
        </ScrollArea>
      </CardContent>
    </Card>
  );
}
