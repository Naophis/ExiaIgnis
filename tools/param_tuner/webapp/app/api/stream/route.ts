import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

const PING_INTERVAL_MS = 15_000;

export async function GET(request: Request) {
  const encoder = new TextEncoder();

  const stream = new ReadableStream({
    start(controller) {
      const send = (event: string, data: unknown) => {
        controller.enqueue(
          encoder.encode(`event: ${event}\ndata: ${JSON.stringify(data)}\n\n`)
        );
      };

      // Push current status immediately so a freshly opened tab doesn't have
      // to wait for the next state change to know if we're connected.
      send("status", serialManager.getStatus());

      const onLog = (line: string) => send("log", { line });
      const onStatus = (status: unknown) => send("status", status);
      const onSaved = (info: unknown) => send("saved", info);
      const onClear = () => send("clear", {});

      serialManager.on("log", onLog);
      serialManager.on("status", onStatus);
      serialManager.on("saved", onSaved);
      serialManager.on("clear", onClear);

      const keepAlive = setInterval(() => {
        controller.enqueue(encoder.encode(`: ping\n\n`));
      }, PING_INTERVAL_MS);

      request.signal.addEventListener("abort", () => {
        clearInterval(keepAlive);
        serialManager.off("log", onLog);
        serialManager.off("status", onStatus);
        serialManager.off("saved", onSaved);
        serialManager.off("clear", onClear);
        try {
          controller.close();
        } catch {
          // already closed
        }
      });
    },
  });

  return new Response(stream, {
    headers: {
      "Content-Type": "text/event-stream",
      "Cache-Control": "no-cache, no-transform",
      Connection: "keep-alive",
    },
  });
}
