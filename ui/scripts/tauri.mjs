// Supply development-time shared libraries when Tauri launches its binary.
import { spawn, spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import path from "node:path";

const ui = fileURLToPath(new URL("..", import.meta.url));
const root = path.resolve(ui, "..");
const env = { ...process.env };
if (process.platform === "linux") {
  const sysroot = spawnSync("rustc", ["--print", "target-libdir"], { encoding: "utf8" });
  env.LD_LIBRARY_PATH = [
    path.join(ui, "src-tauri/target/debug/deps"),
    path.join(ui, "src-tauri/target/release/deps"),
    path.join(root, ".mujoco/mujoco-3.9.0/lib"),
    sysroot.status === 0 ? sysroot.stdout.trim() : "",
    env.LD_LIBRARY_PATH,
  ].filter(Boolean).join(":");
}
const cli = path.join(ui, "node_modules/@tauri-apps/cli/tauri.js");
const child = spawn(process.execPath, [cli, ...process.argv.slice(2)], { cwd: ui, env, stdio: "inherit" });
child.on("error", (error) => { console.error(error.message); process.exitCode = 1; });
child.on("exit", (code) => { process.exitCode = code ?? 1; });
for (const signal of ["SIGINT", "SIGTERM"]) process.on(signal, () => child.kill(signal));
