import { useEffect, useMemo, useRef, useState } from "preact/hooks";
import { Api, ApiError, browserTokenStore, type BagInfo, type LogEntry, type Options, type Status } from "./api";
import { Controls, run, type Notify } from "./Controls";
import { Cameras, Checks, EventsFeed, Gauges, LidarView, MapView } from "./Dashboard";
import { emptyLive, startLive } from "./live";
import type { VehicleInfo } from "./model";
import { RosLink, type LinkStatus } from "./ros";

const FOXGLOVE_PORT = 8765;

export function App() {
  const api = useMemo(() => new Api(browserTokenStore()), []);
  const live = useRef(emptyLive()).current;
  const [, setTick] = useState(0);
  const [needLogin, setNeedLogin] = useState(!api.hasToken());
  const [status, setStatus] = useState<Status | null>(null);
  const [options, setOptions] = useState<Options | null>(null);
  const [vehicle, setVehicle] = useState<VehicleInfo | null>(null);
  const [bags, setBags] = useState<BagInfo[]>([]);
  const [log, setLog] = useState<LogEntry[]>([]);
  const [agentUp, setAgentUp] = useState(true);
  const [link, setLink] = useState<LinkStatus>({ connected: false, topics: [] });
  const [toast, setToast] = useState<{ text: string; kind: "ok" | "error" } | null>(null);
  const toastTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  const notify: Notify = (text, kind = "ok") => {
    setToast({ text, kind });
    if (toastTimer.current) clearTimeout(toastTimer.current);
    toastTimer.current = setTimeout(() => setToast(null), kind === "error" ? 8000 : 3500);
  };

  api.onUnauthorized = () => setNeedLogin(true);

  const refreshStatus = async () => {
    try {
      setStatus(await api.status());
      setAgentUp(true);
    } catch (e) {
      if (e instanceof ApiError && e.status === 0) setAgentUp(false);
    }
  };
  const refreshSlow = async () => {
    try {
      const [o, b, l] = await Promise.all([api.options(), api.bags(), api.log(40)]);
      setOptions(o);
      setBags(b.bags);
      setLog(l.entries);
    } catch {
      /* the status poll reports the connection problem */
    }
  };
  const refreshAll = () => {
    void refreshStatus();
    void refreshSlow();
  };

  useEffect(() => {
    if (needLogin) return;
    void api.vehicle().then(setVehicle).catch(() => undefined);
    refreshAll();
    const fast = setInterval(() => void refreshStatus(), 2000);
    const slow = setInterval(() => void refreshSlow(), 6000);
    return () => {
      clearInterval(fast);
      clearInterval(slow);
    };
  }, [needLogin]);

  useEffect(() => {
    const ros = new RosLink(`ws://${location.hostname}:${FOXGLOVE_PORT}`);
    const off = ros.onStatus(setLink);
    const stop = startLive(ros, live, () => !document.hidden);
    ros.connect();
    const redraw = setInterval(() => setTick((t) => t + 1), 200);
    return () => {
      clearInterval(redraw);
      stop();
      off();
      ros.close();
    };
  }, []);

  const stopAll = () => void run(() => api.stopAll(), notify, refreshAll);

  if (needLogin) return <Login api={api} onDone={() => setNeedLogin(false)} onStop={stopAll} />;

  return (
    <div class="app">
      <header class="top">
        <h1>Racer</h1>
        <span class={`pill ${agentUp ? "lvl-green" : "lvl-red"}`}>agent {agentUp ? "ok" : "unreachable"}</span>
        <span class={`pill ${link.connected ? "lvl-green" : "lvl-red"}`}>ROS link {link.connected ? "ok" : "down"}</span>
        <span class={`pill lvl-${status?.checks.overall ?? "red"}`}>{status?.checks.ready ? "ready to drive" : "not ready"}</span>
      </header>

      <p class="banner">
        This app never drives the car. It starts and stops software, and its STOP is a software stop: Wi-Fi can drop. Arming still needs the RC kill knob, which is the stop that always works.
      </p>

      <main class="grid">
        <Gauges live={live} status={status} vehicle={vehicle} />
        <Checks checks={status?.checks.checks ?? []} status={status} />
        <LidarView live={live} vehicle={vehicle} />
        <MapView live={live} />
        <Cameras live={live} />
        <EventsFeed live={live} />
        <Controls api={api} status={status} options={options} bags={bags} log={log} notify={notify} refresh={refreshAll} />
      </main>

      <button class="stop" onClick={stopAll} aria-label="Stop all driving">STOP ALL</button>
      {toast && <div class={`toast ${toast.kind}`} role="status">{toast.text}</div>}
    </div>
  );
}

function Login({ api, onDone, onStop }: { api: Api; onDone: () => void; onStop: () => void }) {
  const [token, setToken] = useState("");
  const [error, setError] = useState("");
  const submit = async (e: Event) => {
    e.preventDefault();
    api.setToken(token);
    try {
      await api.status();
      onDone();
    } catch (err) {
      setError(err instanceof ApiError && err.status === 401 ? "That token was not accepted." : String(err));
    }
  };
  return (
    <div class="app login">
      <h1>Racer</h1>
      <form onSubmit={submit}>
        <label>
          Access token (shown once when the app was installed)
          <input type="password" value={token} autocomplete="off" onInput={(e) => setToken(e.currentTarget.value)} />
        </label>
        <button type="submit" disabled={!token}>Connect</button>
        {error && <div class="warn">{error}</div>}
      </form>
      <button class="stop" onClick={onStop} aria-label="Stop all driving">STOP ALL</button>
    </div>
  );
}
