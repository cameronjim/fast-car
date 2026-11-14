// Screenshots of every tab in demo mode, desktop and phone width.
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

async function open(size, tab, state, extra = "") {
  const ctx = await browser.newContext({ viewport: size, colorScheme: "dark", deviceScaleFactor: 1 });
  await ctx.addInitScript((t) => {
    try {
      localStorage.setItem("racer-tab", t);
    } catch {}
  }, tab);
  const page = await ctx.newPage();
  await page.goto(`${base}?demo=1&state=${state}${extra}`);
  return { ctx, page };
}

async function shot(page, name, fullPage = false) {
  const file = join(out, `${name}.png`);
  await page.screenshot({ path: file, fullPage });
  written.push(file);
}

const DESKTOP = { width: 1280, height: 860 };
const PHONE = { width: 390, height: 844 };

// every tab on a driving car
for (const tab of ["drive", "tune", "live", "data", "logs", "system"]) {
  const { ctx, page } = await open(DESKTOP, tab, "driving");
  await sleep(3200);
  await shot(page, `${tab}`, tab === "system" || tab === "logs");
  await ctx.close();
}

// an idle car: Start car, the lights off
{
  const { ctx, page } = await open(DESKTOP, "drive", "idle");
  await sleep(1200);
  await shot(page, "drive-idle");
  await page.getByRole("button", { name: "Start car", exact: true }).click();
  await sleep(2200);
  await shot(page, "drive-starting");
  await sleep(4000);
  await shot(page, "drive-ready");
  await page.getByRole("button", { name: /^Gap follow/ }).click();
  await sleep(300);
  await shot(page, "drive-gap-starting");
  await sleep(2500);
  await shot(page, "drive-gap-running");
  await ctx.close();
}

// a failed start: the red badge, then the Logs tab it opens
{
  const { ctx, page } = await open(DESKTOP, "drive", "idle", "&panel=1");
  await sleep(900);
  await page.getByRole("button", { name: "Fail next start" }).click();
  await page.getByRole("button", { name: "Start car", exact: true }).click();
  await sleep(2600);
  await shot(page, "drive-start-failed");
  await page.getByRole("button", { name: /Open Logs/ }).first().click();
  await sleep(600);
  await shot(page, "logs-after-failure");
  await ctx.close();
}

// the override flow: low battery refuses a drive
{
  const { ctx, page } = await open(DESKTOP, "drive", "running", "&panel=1");
  await sleep(1500);
  await page.getByRole("button", { name: /Battery: ok/ }).click();
  await page.getByRole("button", { name: /^Gap follow/ }).click();
  await sleep(1800);
  await shot(page, "drive-override");
  await ctx.close();
}

// tune: an out-of-bounds value is marked and not saved
{
  const { ctx, page } = await open(DESKTOP, "tune", "running");
  await sleep(1500);
  await page.getByRole("spinbutton", { name: "Max speed in m/s" }).fill("3");
  await sleep(900);
  await shot(page, "tune-invalid");
  await page.getByRole("spinbutton", { name: "Max speed in m/s" }).fill("0.5");
  await sleep(1600);
  await shot(page, "tune-saved");
  await ctx.close();
}

// data: delete asks twice
{
  const { ctx, page } = await open(DESKTOP, "data", "running");
  await sleep(1500);
  await page.getByRole("button", { name: /^Delete 2026-11-11/ }).click();
  await sleep(200);
  await shot(page, "data-delete-confirm");
  await ctx.close();
}

// the agent link drops
{
  const { ctx, page } = await open(DESKTOP, "drive", "driving", "&panel=1");
  await sleep(2500);
  await page.getByRole("button", { name: "Drop link 10 s" }).click();
  await sleep(7000);
  await shot(page, "reconnecting");
  await ctx.close();
}

// phone width
for (const tab of ["drive", "tune", "live", "data"]) {
  const { ctx, page } = await open(PHONE, tab, "driving");
  await sleep(3200);
  await shot(page, `phone-${tab}`, true);
  await ctx.close();
}

await browser.close();
await server.close();
console.log(written.join("\n"));
