import path from "node:path";
import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // serialport uses native (node-gyp) bindings; keep it out of the server
  // bundle so it's loaded via native require instead.
  serverExternalPackages: ["serialport", "@serialport/bindings-cpp"],
  // tools/param_tuner/ has its own lockfiles for the legacy CLI scripts;
  // pin the workspace root so Turbopack doesn't guess wrong.
  turbopack: {
    root: path.join(__dirname),
  },
};

export default nextConfig;
