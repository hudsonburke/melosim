import { spawn } from "node:child_process";

/** Own each child's process group, including shells and grandchildren. */
export function supervise(commands, { graceMs = 2000 } = {}) {
  return new Promise((resolve) => {
    const children = [];
    let stopping = false;
    let timer;
    const signals = { SIGINT: 130, SIGTERM: 143, SIGHUP: 129 };
    const handlers = new Map();
    function signal(child, value) {
      if (!child.pid) return;
      try {
        if (process.platform === "win32") {
          spawn("taskkill", ["/pid", String(child.pid), "/T", "/F"], {
            stdio: "ignore",
          });
        } else {
          process.kill(-child.pid, value);
        }
      } catch (error) {
        if (error.code !== "ESRCH") console.error(error.message);
      }
    }
    function stop(code, value = "SIGTERM") {
      if (stopping) return;
      stopping = true;
      for (const child of children) signal(child, value);
      // A launcher may exit before its descendants. Always clean the owned
      // groups after the grace period, even if all direct children exited.
      timer = setTimeout(() => {
        for (const child of children) signal(child, "SIGKILL");
        for (const [name, handler] of handlers) process.off(name, handler);
        resolve(code);
      }, graceMs);
    }
    for (const [name, code] of Object.entries(signals)) {
      const handler = () => stop(code, name);
      handlers.set(name, handler);
      process.on(name, handler);
    }
    for (const { command, args = [], ...options } of commands) {
      const child = spawn(command, args, {
        stdio: "inherit",
        ...options,
        detached: process.platform !== "win32",
      });
      children.push(child);
      child.once("error", (error) => {
        console.error(`Unable to start ${command}: ${error.message}`);
        stop(1);
      });
      child.once("exit", (code, sig) => stop(code ?? signals[sig] ?? 1));
    }
    if (!commands.length) {
      clearTimeout(timer);
      stop(0);
    }
  });
}
