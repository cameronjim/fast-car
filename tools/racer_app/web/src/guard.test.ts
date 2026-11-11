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

// The desktop build (tools/racer_app/desktop) bundles this same UI, so the same rules cover it:
// the shared files above include platform.ts and Shell.tsx, and the checks below cover the
// native half and the permissions it is granted.
const DESKTOP = join(SRC, "..", "..", "desktop", "src-tauri");

function rust(file: string): string {
  return readFileSync(join(DESKTOP, "src", file), "utf8")
    .replace(/\/\*[\s\S]*?\*\//g, "")
    .replace(/(^|[^:])\/\/.*$/gm, "$1");
}

const rustFiles = readdirSync(join(DESKTOP, "src")).filter((f) => f.endsWith(".rs"));

describe("the desktop build never publishes either", () => {
  it("includes the platform layer in the checks above", () => {
    expect(files).toContain("platform.ts");
    expect(files).toContain("Shell.tsx");
    expect(rustFiles).toEqual(expect.arrayContaining(["discovery.rs", "net.rs", "lib.rs"]));
  });

  it.each(rustFiles)("%s names no command topic and makes no write request", (file) => {
    const src = rust(file);
    expect(src).not.toMatch(/drive_raw|teleop|cmd_vel|\/drive\b/);
    expect(src).not.toMatch(
      /\.(post|put|patch|delete)\(|send_json|send_form|Method::(POST|PUT|PATCH|DELETE)/,
    );
    expect(src).not.toContain("/api/");
  });

  it("the native side only ever requests /health", () => {
    const urls = [...rust("net.rs").matchAll(/:\{port\}\/([a-z]+)/g)].map((m) => m[1]);
    expect(urls).toEqual(["health"]);
  });

  it("only platform.ts reaches the Tauri bridge, and only for known commands", () => {
    const users = files.filter((f) => code(f).includes("__TAURI__"));
    expect(users).toEqual(["platform.ts"]);
    const commands = [...code("platform.ts").matchAll(/invoke(?:<[^>]*>)?\(\s*"([a-z_]+)"/g)].map(
      (m) => m[1],
    );
    expect(new Set(commands)).toEqual(
      new Set([
        "token_get",
        "token_set",
        "token_clear",
        "discovery_state",
        "select_car",
        "add_manual",
        "rescan",
      ]),
    );
  });

  it("the window is granted no file, shell, process or opener access", () => {
    const cap = JSON.parse(
      readFileSync(join(DESKTOP, "capabilities", "default.json"), "utf8"),
    ) as { permissions: Array<string | { identifier: string }> };
    const ids = cap.permissions.map((p) => (typeof p === "string" ? p : p.identifier));
    expect(ids.sort()).toEqual(["core:default", "http:default"]);
  });

  it("keeps a content security policy", () => {
    const conf = JSON.parse(readFileSync(join(DESKTOP, "tauri.conf.json"), "utf8")) as {
      app: { security: { csp: string } };
    };
    expect(conf.app.security.csp).toContain("default-src 'self'");
    expect(conf.app.security.csp).not.toContain("unsafe-eval");
  });
});
