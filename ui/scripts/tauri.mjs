// Supply development-time shared libraries when Tauri launches its binary.
import { spawnSync } from "node:child_process";
import { supervise } from "./supervise.mjs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const ui = fileURLToPath(new URL("..", import.meta.url));
const root = path.resolve(ui, "..");
const env = { ...process.env };
if (process.platform === "linux") {
  const sysroot = spawnSync("rustc", ["--print", "target-libdir"], {
    encoding: "utf8",
  });
  env.LD_LIBRARY_PATH = [
    path.join(ui, "src-tauri/target/debug/deps"),
    path.join(ui, "src-tauri/target/release/deps"),
    path.join(root, ".mujoco/mujoco-3.9.0/lib"),
    sysroot.status === 0 ? sysroot.stdout.trim() : "",
    env.LD_LIBRARY_PATH,
  ]
    .filter(Boolean)
    .join(":");
}
const cli = path.join(ui, "node_modules/@tauri-apps/cli/tauri.js");
const args = process.argv.slice(2);
const dev =
  args[0] === "dev" && !args.includes("--help") && !args.includes("-h");
// Own Vite directly instead of leaving it behind a CLI-managed shell.
// The CLI still waits for devUrl before launching the native application.
const commands = dev
  ? [
      {
        command: process.execPath,
        args: [path.join(ui, "node_modules/vite/bin/vite.js")],
        cwd: ui,
        env,
      },
    ]
  : [];
if (dev)
  args.splice(
    1,
    0,
    "--config",
    JSON.stringify({ build: { beforeDevCommand: "" } }),
  );
commands.push({
  command: process.execPath,
  args: [cli, ...args],
  cwd: ui,
  env,
});
process.exitCode = await supervise(commands);
