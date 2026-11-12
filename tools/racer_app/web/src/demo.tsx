// Demo mode: `?demo=1` runs the whole UI against an in-browser fake agent and fake ROS link, so it
// can be reviewed (and screenshotted) without the car. `?demo=1&state=driving` starts in a state
// (idle, running, driving, starting, offline, failed). The panel at the side scripts trouble.

import { useMemo, useState } from "preact/hooks";
import type { TokenStore } from "./api";
import { App } from "./App";
import { DEMO_PRESETS, DemoCar, parsePreset, type DemoPreset } from "./demoAgent";
import { DemoRos } from "./demoRos";

export function isDemo(search: string): boolean {
  return new URLSearchParams(search).get("demo") === "1";
}

const memoryTokens = (): TokenStore => {
  let t: string | null = "demo";
  return { get: () => t, set: (v) => void (t = v), clear: () => void (t = null) };
};

export function DemoRoot({ search }: { search: string }) {
  const initial = parsePreset(new URLSearchParams(search).get("state"));
  const [run, setRun] = useState({ n: 0, preset: initial });
  const car = useMemo(() => {
    const c = new DemoCar();
    c.preset(run.preset);
    return c;
  }, [run]);
  const tokens = useMemo(memoryTokens, []);
  const rosFactory = useMemo(() => () => new DemoRos(car), [car]);
  return (
    <>
      <App
        key={run.n}
        target={{ apiBase: "", rosUrl: "demo" }}
        tokens={tokens}
        fetchImpl={car.fetch}
        rosFactory={rosFactory}
        carName="racer-car (demo)"
      />
      <DemoPanel car={car} search={search} onReset={(preset) => setRun((r) => ({ n: r.n + 1, preset }))} />
    </>
  );
}

function DemoPanel({ car, search, onReset }: { car: DemoCar; search: string; onReset: (p: DemoPreset) => void }) {
  const [open, setOpen] = useState(new URLSearchParams(search).get("panel") === "1");
  const [, bump] = useState(0);
  if (!open) {
    return (
      <div class="demo min">
        <button class="btn sm" onClick={() => setOpen(true)}>Demo</button>
      </div>
    );
  }
  return (
    <aside class="demo" aria-label="Demo controls">
      <div class="btnrow" style={{ justifyContent: "space-between" }}>
        <b>Demo car</b>
        <button class="btn ghost sm" onClick={() => setOpen(false)} aria-label="Hide demo controls">Hide</button>
      </div>
      <div class="btnrow">
        <button class="btn" onClick={() => void car.dropConnection(10_000)}>Drop link 10 s</button>
        <button
          class="btn"
          onClick={() => {
            car.failNext = true;
            bump((n) => n + 1);
          }}
        >
          {car.failNext ? "Next start fails (armed)" : "Fail next start"}
        </button>
        <button
          class="btn"
          onClick={() => {
            car.lowBattery = !car.lowBattery;
            bump((n) => n + 1);
          }}
        >
          {car.lowBattery ? "Battery: low" : "Battery: ok"}
        </button>
      </div>
      <div class="btnrow">
        {DEMO_PRESETS.map((p) => (
          <button key={p} class="btn" onClick={() => onReset(p)}>{p}</button>
        ))}
      </div>
    </aside>
  );
}
