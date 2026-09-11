import { spawn } from "node:child_process";
import { createServer } from "node:net";
import { fileURLToPath } from "node:url";
import { supervise } from "./supervise.mjs";

const file = fileURLToPath(import.meta.url);
const mode = process.argv[2];
if (mode === "worker") {
  // Exercise the forced cleanup path as well as ordinary signal forwarding.
  process.on("SIGTERM", () => {});
  process.on("SIGINT", () => {});
  createServer().listen(0, "127.0.0.1", function () {
    console.log(`PORT ${this.address().port}`);
    process.send?.("ready");
  });
} else if (mode === "leader") {
  const worker = spawn(process.execPath, [file, "worker"], {
    stdio: ["ignore", "inherit", "inherit", "ipc"],
  });
  worker.on("message", () => {
    if (process.argv[3] === "exit") process.exit(0);
    if (process.argv[3] === "fail") process.exit(7);
  });
} else {
  process.exitCode = await supervise(
    [
      { command: process.execPath, args: [file, "leader", mode] },
      { command: process.execPath, args: [file, "worker"] },
    ],
    { graceMs: 200 },
  );
}
