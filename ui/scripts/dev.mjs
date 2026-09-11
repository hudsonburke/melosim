import { fileURLToPath } from "node:url";
import path from "node:path";
import { supervise } from "./supervise.mjs";

const ui = fileURLToPath(new URL("..", import.meta.url));
const root = path.resolve(ui, "..");
process.exitCode = await supervise([
  {
    command: "cargo",
    args: ["run", "--features", "web-editor", "--bin", "web-editor"],
    cwd: root,
  },
  {
    command: process.execPath,
    args: [path.join(ui, "node_modules/vite/bin/vite.js")],
    cwd: ui,
  },
]);
