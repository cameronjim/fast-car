// Screenshots of every tab in demo mode, light and dark, desktop and phone width.
//   npm run shots -- <output dir>
// Starts its own vite dev server, drives the in-browser demo car (no car needed) and uses the
// system Chrome (set CHROME_PATH to override), so nothing is downloaded.

import { existsSync, mkdirSync } from "node:fs";
import { join, resolve } from "node:path";
import { chromium } from "playwright-core";
import { createServer } from "vite";

const out = resolve(process.argv[2] ?? "shots");
mkdirSync(out, { recursive: true });

const CHROME =
  process.env.CHROME_PATH ??
  ["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", "/usr/bin/google-chrome", "/usr/bin/chromium"].find(existsSync);
if (!CHROME) throw new Error("No Chrome found. Set CHROME_PATH.");

const server = await createServer({ server: { port: 5199, strictPort: true, proxy: {} }, logLevel: "error" });
await server.listen();
const base = "http://localhost:5199/";
const browser = await chromium.launch({ executablePath: CHROME, headless: true });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const written = [];

async function open(scheme, size, tab, state, extra = "") {
  const ctx = await browser.newContext({ viewport: size, colorScheme: scheme, deviceScaleFactor: 1 });
  await ctx.addInitScript((t) => {
    try {
      localStorage.setItem("racer-tab", t);
    } catch {}
  }, tab);
  const page = await ctx.newPage();
  await page.goto(`${base}?demo=1&state=${state}${extra}`);
  return { ctx, page };
}

async function shot(page, name) {
  const file = join(out, `${name}.png`);
  await page.screenshot({ path: file });
  written.push(file);
}

const DESKTOP = { width: 1280, height: 860 };
const PHONE = { width: 390, height: 844 };

for (const scheme of ["light", "dark"]) {
  // the five tabs on a driving car
  for (const tab of ["drive", "live", "checks", "logs", "system"]) {
    const { ctx, page } = await open(scheme, DESKTOP, tab, "driving");
    await sleep(3200);
    if (tab === "checks") await page.getByRole("button", { name: "Battery OK" }).click();
    await sleep(300);
    await shot(page, `${tab}-${scheme}`);
    await ctx.close();
  }

  // first load (skeletons): shot taken before the first status reply
  {
    const { ctx, page } = await open(scheme, DESKTOP, "drive", "running");
    await sleep(60);
    await shot(page, `loading-skeleton-${scheme}`);
    await ctx.close();
  }

  // stack start in progress: optimistic state and the timeline
  {
    const { ctx, page } = await open(scheme, DESKTOP, "system", "idle");
    await sleep(900);
    await page.getByRole("button", { name: "Start stack", exact: true }).click();
    await sleep(2600);
    await shot(page, `stack-starting-${scheme}`);
    await ctx.close();
  }

  // a failed start, with the reason
  {
    const { ctx, page } = await open(scheme, DESKTOP, "system", "idle", "&panel=1");
    await sleep(900);
    await page.getByRole("button", { name: "Fail next start" }).click();
    await page.getByRole("button", { name: "Start stack", exact: true }).click();
    await sleep(2600);
    await shot(page, `stack-start-failed-${scheme}`);
    await ctx.close();
  }

  // a slow stop: "closing the bag"
  {
    const { ctx, page } = await open(scheme, DESKTOP, "system", "running");
    await sleep(1500);
    await page.getByRole("button", { name: "Stop stack", exact: true }).click();
    await page.getByRole("button", { name: "Stop stack", exact: true }).click();
    await sleep(1800);
    await shot(page, `stack-stopping-${scheme}`);
    await ctx.close();
  }

  // the agent link drops: reconnecting banner, values greyed with their age
  {
    const { ctx, page } = await open(scheme, DESKTOP, "live", "driving", "&panel=1");
    await sleep(2500);
    await page.getByRole("button", { name: "Drop link 10 s" }).click();
    await sleep(7000);
    await shot(page, `reconnecting-${scheme}`);
    await ctx.close();
  }

  // the override flow: low battery refuses a drive, the panel offers an override
  {
    const { ctx, page } = await open(scheme, DESKTOP, "drive", "running", "&panel=1");
    await sleep(1500);
    await page.getByRole("button", { name: /Battery: ok/ }).click();
    await page.getByRole("button", { name: "Start Wall follow" }).click();
    await sleep(1800);
    await shot(page, `drive-override-${scheme}`);
    await ctx.close();
  }

  // phone width
  for (const tab of ["drive", "live", "checks"]) {
    const { ctx, page } = await open(scheme, PHONE, tab, "driving");
    await sleep(3200);
    await shot(page, `phone-${tab}-${scheme}`);
    await ctx.close();
  }
}

await browser.close();
await server.close();
console.log(written.join("\n"));
