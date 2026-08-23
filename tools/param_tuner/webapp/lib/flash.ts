import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { serialManager } from "./serial-manager";

// webapp/ -> tools/param_tuner/ -> repo root, where flash.sh lives.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const REPO_ROOT = path.join(PARAM_TUNER_ROOT, "..", "..");
const FLASH_SCRIPT = path.join(REPO_ROOT, "flash.sh");

export interface FlashResult {
  ok: boolean;
  output: string;
}

function emitLines(chunk: Buffer) {
  for (const line of chunk.toString().split("\n")) {
    if (line.length > 0) serialManager.emit("log", `[flash] ${line}`);
  }
}

// After a successful flash the device reboots and re-enumerates over USB,
// which takes a bit longer than the 200ms search-loop tick alone suggests -
// enough that a test run started immediately after the "Flash" button
// reports done can start sending its dump_csv() header before the Param
// Console has actually finished reconnecting, losing it outright (confirmed
// 2026-08-23: a dump run right after reflashing came back with an entirely
// empty header - see project_param_console_dump_robustness memory). Block
// the flash result on an actual "connected" status instead of firing
// enableAutoConnect() and hoping, so the UI only reports done once it's
// truly safe to start testing again.
function waitForReconnect(timeoutMs: number): Promise<boolean> {
  return new Promise((resolve) => {
    const deadline = Date.now() + timeoutMs;
    const check = () => {
      if (serialManager.getStatus().status === "connected") {
        resolve(true);
        return;
      }
      if (Date.now() >= deadline) {
        resolve(false);
        return;
      }
      setTimeout(check, 200);
    };
    check();
  });
}

export function flashDevice(): Promise<FlashResult> {
  if (!fs.existsSync(FLASH_SCRIPT)) {
    return Promise.reject(new Error("flash.shが見つかりません"));
  }

  // picotool needs exclusive access to the USB device, and a successful
  // flash reboots it out of BOOTSEL back into normal firmware (the CDC
  // device disappears and re-enumerates). Release our own serial connection
  // first so it doesn't fight picotool for the port, then let the existing
  // auto-connect search loop (SerialManager.trySearch, 200ms poll) pick the
  // device back up once it re-enumerates - no explicit reconnect needed.
  serialManager.disconnect();

  return new Promise((resolve) => {
    const child = spawn(FLASH_SCRIPT, [], { cwd: REPO_ROOT });
    let output = "";
    child.stdout.on("data", (chunk: Buffer) => {
      output += chunk.toString();
      emitLines(chunk);
    });
    child.stderr.on("data", (chunk: Buffer) => {
      output += chunk.toString();
      emitLines(chunk);
    });
    child.on("close", async (code) => {
      serialManager.enableAutoConnect();
      if (code === 0) {
        const reconnected = await waitForReconnect(10_000);
        if (!reconnected) {
          const msg = "書き込みは完了しましたが、10秒待っても再接続を確認できませんでした。デバイスの状態を確認してください";
          serialManager.emit("log", `[flash] warning: ${msg}`);
          output += `\n${msg}`;
        } else {
          // A little extra margin beyond "port opened": the tty node can be
          // openable slightly before the device's own USB CDC stack has
          // fully settled, which is exactly the window that lost the dump
          // header in the incident above.
          await new Promise((r) => setTimeout(r, 500));
          serialManager.emit("log", "[flash] reconnected, ready");
        }
      }
      resolve({ ok: code === 0, output });
    });
    child.on("error", (err) => {
      output += `\n${err.message}`;
      serialManager.enableAutoConnect();
      resolve({ ok: false, output });
    });
  });
}
