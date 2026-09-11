import { test } from "node:test";
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { createServer } from "node:net";
import { fileURLToPath } from "node:url";

for (const mode of ["SIGINT", "SIGTERM", "SIGHUP", "exit", "fail"]) {
  test(
    `releases child and grandchild ports after ${mode}`,
    {
      skip: process.platform === "win32",
      timeout: 10000,
    },
    async () => {
      const child = spawn(
        process.execPath,
        [
          fileURLToPath(new URL("./supervise.fixture.mjs", import.meta.url)),
          mode,
        ],
        {
          stdio: ["ignore", "pipe", "inherit"],
        },
      );
      let output = "";
      const exited = new Promise((resolve) => child.on("exit", resolve));
      const ready = new Promise((resolve) =>
        child.stdout.on("data", (chunk) => {
          output += chunk;
          if ((output.match(/PORT \d+/g) || []).length === 2) resolve();
        }),
      );
      await ready;
      if (mode.startsWith("SIG")) child.kill(mode);
      const code = await exited;
      assert.equal(
        code,
        { SIGINT: 130, SIGTERM: 143, SIGHUP: 129, exit: 0, fail: 7 }[mode],
      );
      const ports = [...output.matchAll(/PORT (\d+)/g)].map((match) =>
        Number(match[1]),
      );
      assert.equal(ports.length, 2);
      for (const port of ports) {
        await new Promise((resolve, reject) => {
          const server = createServer();
          server.on("error", reject);
          server.listen(port, "127.0.0.1", () => server.close(resolve));
        });
      }
    },
  );
}
