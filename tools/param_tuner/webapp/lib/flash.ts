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
    child.on("close", (code) => {
      serialManager.enableAutoConnect();
      resolve({ ok: code === 0, output });
    });
    child.on("error", (err) => {
      output += `\n${err.message}`;
      serialManager.enableAutoConnect();
      resolve({ ok: false, output });
    });
  });
}
