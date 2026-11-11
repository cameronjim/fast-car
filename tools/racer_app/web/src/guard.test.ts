// Invariant 1 (CLAUDE.md): nothing in this app publishes to the command path. The UI only
// subscribes. These tests read the source and fail if anything reaches for a publishing call
// of the foxglove client, or names a command topic anywhere but in a comment or a test.

import { readdirSync, readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";

const SRC = __dirname;
const files = readdirSync(SRC).filter((f) => /\.(ts|tsx)$/.test(f) && !f.endsWith(".test.ts"));

function code(file: string): string {
  // strip block and line comments so the rule is about code, not prose
  return readFileSync(join(SRC, file), "utf8")
    .replace(/\/\*[\s\S]*?\*\//g, "")
    .replace(/(^|[^:])\/\/.*$/gm, "$1");
}

describe("the UI never publishes", () => {
  it("has source files to check", () => {
    expect(files).toContain("ros.ts");
    expect(files.length).toBeGreaterThan(5);
  });

  it.each(files)("%s makes no publishing calls", (file) => {
    const src = code(file);
    for (const call of [
      ".advertise(",
      ".unadvertise(",
      ".sendMessage(",
      ".sendServiceCallRequest(",
      ".sendCallServiceRequest(",
      ".setParameters(",
      ".fetchAsset(",
    ]) {
      expect(src, `${file} uses ${call}`).not.toContain(call);
    }
  });

  it.each(files)("%s does not subscribe to or name the command topics", (file) => {
    const src = code(file);
    expect(src).not.toMatch(/["'`]\/drive_raw["'`]/);
    expect(src).not.toMatch(/["'`]\/teleop\/cmd_vel["'`]/);
  });

  it("only /drive is read, as the steering readout", () => {
    const uses = files.filter((f) => /["'`]\/drive["'`]/.test(code(f)));
    expect(uses).toEqual(["live.ts"]);
    expect(code("live.ts")).toContain('ros.subscribe<DriveLike>("/drive"');
  });

  it("the API client has no route for starting an arbitrary command", () => {
    const api = code("api.ts");
    const routes = [...api.matchAll(/"\/api\/[^"`]*/g)].map((m) => m[0]);
    for (const r of routes) expect(r).not.toMatch(/exec|shell|cmd|run/);
  });
});
