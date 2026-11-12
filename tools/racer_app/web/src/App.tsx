import { useEffect, useMemo, useRef, useState } from "preact/hooks";
import {
  Api,
  ApiError,
  type ActionReply,
  type BagInfo,
  type FetchLike,
  loginError,
  type LogEntry,
  type Options,
  type Status,
  type TokenStore,
} from "./api";
import { ChecksTab } from "./ChecksTab";
import { OpsTray, ReconnectBanner, StopBar, Tabs, TopBar } from "./chrome";
import { AppCtx, TABS, useStore, type Ctx, type TabId } from "./ctx";
import { DriveTab } from "./DriveTab";
import { ConnectionTracker, errorText, ToastStore } from "./feedback";
import { emptyLive, startLive } from "./live";
import { LiveTab } from "./LiveTab";
import { LogsTab } from "./LogsTab";
import type { VehicleInfo } from "./model";
import { doneText, OpsStore } from "./ops";
import { describeTokenStorage, type Target, type TokenStorageInfo } from "./platform";
import { RosLink, type LinkStatus, type RosSource } from "./ros";
import { SystemTab } from "./SystemTab";
import { Toasts } from "./ui";

export interface AppProps {
  target: Target;
  tokens: TokenStore;
  fetchImpl?: FetchLike;
  /** Desktop only: where the token is kept, so the UI can say when it is not the keychain. */
  tokenStorage?: TokenStorageInfo | null;
  /** Demo mode swaps in a fake ROS link. */
  rosFactory?: () => RosSource;
  carName?: string;
  /** Status poll period in ms (the poll doubles in speed while an operation is in flight). */
  pollMs?: number;
}

const TAB_KEY = "racer-tab";

function loadTab(): TabId {
  try {
    const t = localStorage.getItem(TAB_KEY);
    if (TABS.some((x) => x.id === t)) return t as TabId;
  } catch {
    /* no storage: start on Drive */
  }
  return "drive";
}

function saveTab(t: TabId): void {
  try {
    localStorage.setItem(TAB_KEY, t);
  } catch {
    /* private mode */
  }
}

function tokenWhere(info: TokenStorageInfo | null | undefined): string {
  if (!info) return "This browser";
  if (info.storage === "keychain") return "System keychain";
  if (info.storage === "file") return "App data file (only you can read it)";
  return "Memory only (lost on quit)";
}

export function App({ target, tokens, fetchImpl, tokenStorage, rosFactory, carName, pollMs = 2000 }: AppProps) {
  const api = useMemo(() => new Api(tokens, fetchImpl, target.apiBase), [target.apiBase]);
  const live = useRef(emptyLive()).current;
  const ops = useMemo(() => new OpsStore(), []);
  const toasts = useMemo(() => new ToastStore(), []);
  const agentConn = useRef(new ConnectionTracker(2)).current;
  const linkConn = useRef(new ConnectionTracker(1)).current;
  const seq = useRef(0);
  const [, setTick] = useState(0);
  const [needLogin, setNeedLogin] = useState(!api.hasToken());
  const [rejected, setRejected] = useState(false);
  const [status, setStatus] = useState<Status | null>(null);
  const [options, setOptions] = useState<Options | null>(null);
  const [vehicle, setVehicle] = useState<VehicleInfo | null>(null);
  const [bags, setBags] = useState<BagInfo[]>([]);
  const [log, setLog] = useState<LogEntry[]>([]);
  const [agentUp, setAgentUp] = useState(true);
  const [agentLastSeen, setAgentLastSeen] = useState<number | null>(null);
  const [link, setLink] = useState<LinkStatus>({ connected: false, topics: [] });
  const [linkLastSeen, setLinkLastSeen] = useState<number | null>(null);
  const [tab, setTabState] = useState<TabId>(loadTab);
  const latest = useRef({ status, link, bags });
  latest.current = { status, link, bags };

  const setTab = (t: TabId) => {
    setTabState(t);
    saveTab(t);
  };

  api.onUnauthorized = () => {
    // The saved token is stale (the owner rotated it, or it came from another car): drop it so
    // the next launch does not loop on it, and say so on the login screen.
    setRejected(true);
    tokens.clear();
    setNeedLogin(true);
  };

  const tickOps = (over: Partial<{ status: Status | null; link: LinkStatus; bags: BagInfo[] }> = {}) => {
    const cur = { ...latest.current, ...over };
    ops.tick({ status: cur.status, linkConnected: cur.link.connected, bags: cur.bags, seq: seq.current });
  };

  const refreshSlow = async () => {
    try {
      const [o, b, l] = await Promise.all([api.options(), api.bags(), api.log(40)]);
      setOptions(o);
      setBags(b.bags);
      setLog(l.entries);
      latest.current = { ...latest.current, bags: b.bags };
    } catch {
      /* the status poll reports the connection problem */
    }
  };

  const refreshStatus = async () => {
    try {
      const s = await api.status();
      seq.current += 1;
      const now = Date.now();
      const back = agentConn.ok(now);
      latest.current = { ...latest.current, status: s };
      setStatus(s);
      setAgentUp(true);
      setAgentLastSeen(now);
      if (back) {
        toasts.push("Reconnected to the car", "ok");
        void refreshSlow();
      }
      tickOps({ status: s });
    } catch (e) {
      if (e instanceof ApiError && (e.status === 0 || e.status >= 500)) {
        const lost = agentConn.fail();
        setAgentUp(agentConn.state.up);
        if (lost) toasts.push("Lost contact with the car. Retrying.", "error");
      }
    }
  };

  const refreshAll = () => {
    void refreshStatus();
    void refreshSlow();
  };

  useEffect(() => {
    if (needLogin) return;
    let alive = true;
    let timer: ReturnType<typeof setTimeout> | null = null;
    void api.vehicle().then(setVehicle).catch(() => undefined);
    void refreshSlow();
    // One poll at a time. While an operation is in flight the poll runs twice as often, so its
    // timeline follows the car closely.
    const loop = async () => {
      await refreshStatus();
      if (alive) timer = setTimeout(() => void loop(), ops.active().length ? Math.max(250, pollMs / 2) : pollMs);
    };
    void loop();
    const slow = setInterval(() => void refreshSlow(), 4000);
    // Timeouts must fire even when no poll answers (the car is gone mid-operation).
    const watchdog = setInterval(() => tickOps(), 1000);
    return () => {
      alive = false;
      if (timer) clearTimeout(timer);
      clearInterval(slow);
      clearInterval(watchdog);
    };
  }, [needLogin, api]);

  useEffect(() => {
    const ros: RosSource = rosFactory ? rosFactory() : new RosLink(target.rosUrl);
    const off = ros.onStatus((s) => {
      const now = Date.now();
      const change = linkConn.set(s.connected, now);
      setLink(s);
      if (s.connected) setLinkLastSeen(now);
      else if (change === "lost" && latest.current.status?.stack.running) toasts.push("Live data link lost", "error");
      latest.current = { ...latest.current, link: s };
      tickOps({ link: s });
    });
    const stop = startLive(ros, live, () => !document.hidden);
    ros.connect();
    const redraw = setInterval(() => setTick((t) => t + 1), 200);
    // While connected the "last seen" time follows the clock.
    const seen = setInterval(() => {
      if (linkConn.state.up) setLinkLastSeen(Date.now());
    }, 1000);
    return () => {
      clearInterval(redraw);
      clearInterval(seen);
      stop();
      off();
      ros.close();
    };
  }, [target.rosUrl, rosFactory]);

  // Say how each operation ended (the timeline shows the progress, the toast the outcome).
  const announced = useRef(new Set<number>());
  useStore((cb) => ops.subscribe(cb));
  useEffect(() => {
    for (const op of ops.all()) {
      if (announced.current.has(op.id)) continue;
      if (op.phase === "confirmed") toasts.push(doneText(op.kind), "ok");
      else if (op.phase === "timeout") toasts.push(`${op.title} timed out. ${op.error ?? ""}`.trim(), "error");
      else continue;
      announced.current.add(op.id);
    }
  });

  const act: Ctx["act"] = async (fn) => {
    try {
      const r = await fn();
      toasts.push(r.message ?? "Done", "ok");
      refreshAll();
      return r;
    } catch (e) {
      toasts.push(errorText(e), "error");
      refreshAll();
      throw e;
    }
  };

  const runOp: Ctx["runOp"] = (spec, fn) => {
    const box: { p: Promise<ActionReply> | null } = { p: null };
    const op = ops.begin(spec, () => (box.p = act(fn)));
    const p = box.p;
    if (!op || !p) return Promise.reject(new ApiError(`${spec.title ?? spec.kind} is already in progress`, 409, null));
    tickOps();
    return p;
  };

  const stopAll: Ctx["stopAll"] = () => {
    // STOP is never blocked by an operation in flight: it is always sent, and the timeline
    // (when none is running yet) shows it until the status says nothing is driving.
    const p = act(() => api.stopAll());
    ops.begin({ kind: "stop-all" }, () => p);
    tickOps();
    return p;
  };

  if (needLogin) {
    return (
      <Login
        api={api}
        rejected={rejected}
        tokenStorage={tokenStorage ?? null}
        onDone={() => {
          setRejected(false);
          setNeedLogin(false);
        }}
      />
    );
  }

  const ctx: Ctx = {
    api,
    status,
    options,
    bags,
    log,
    agentUp,
    agentLastSeen,
    link,
    linkLastSeen,
    ops,
    toasts,
    now: Date.now(),
    refresh: refreshAll,
    act,
    runOp,
    stopAll,
    setTab,
    carName: carName ?? "Racer",
    tokenWhere: tokenWhere(tokenStorage),
    tokenNote: describeTokenStorage(tokenStorage ?? null),
  };

  return (
    <AppCtx.Provider value={ctx}>
      <div class="app">
        <TopBar live={live} onTab={setTab} />
        <Tabs tab={tab} onTab={setTab} />
        <ReconnectBanner />
        <main class={`page ${agentUp ? "" : "offline"}`} aria-label={tab}>
          <OpsTray />
          {tab === "drive" && <DriveTab />}
          {tab === "live" && <LiveTab live={live} vehicle={vehicle} />}
          {tab === "checks" && <ChecksTab />}
          {tab === "logs" && <LogsTab />}
          {tab === "system" && <SystemTab />}
        </main>
        <StopBar onStop={stopAll} />
        <Toasts store={toasts} />
      </div>
    </AppCtx.Provider>
  );
}

function Login({
  api,
  rejected,
  tokenStorage,
  onDone,
}: {
  api: Api;
  rejected: boolean;
  tokenStorage: TokenStorageInfo | null;
  onDone: () => void;
}) {
  const [token, setToken] = useState("");
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const submit = async (e: Event) => {
    e.preventDefault();
    setBusy(true);
    setError("");
    try {
      // Check it first and save it only if the car accepts it: a typo must not replace a good
      // saved token, and a good one is stored the moment it works.
      await api.verifyToken(token);
      api.setToken(token);
      onDone();
    } catch (err) {
      setError(loginError(err));
    } finally {
      setBusy(false);
    }
  };
  const note = describeTokenStorage(tokenStorage);
  return (
    <div class="app">
      <div class="login">
        <h1>Racer</h1>
        {rejected && (
          <div class="warn" role="alert">
            The car rejected the saved token. Paste a new one. If it was rotated or lost, make a new one on the Jetson with <code>racer-agent init --rotate-token</code>.
          </div>
        )}
        <form onSubmit={submit}>
          <label class="field">
            <span>Access token</span>
            <input type="password" value={token} autocomplete="off" onInput={(e) => setToken(e.currentTarget.value)} />
          </label>
          <button class="btn primary lg" type="submit" disabled={!token || busy} aria-busy={busy}>
            {busy ? "Checking..." : "Connect"}
          </button>
          {error && <div class="warn" role="alert">{error}</div>}
        </form>
        <p class="sub">Shown once when the app was installed on the car.</p>
        {note && <p class="sub">{note}</p>}
      </div>
      <StopBar onStop={() => api.stopAll()} />
    </div>
  );
}
