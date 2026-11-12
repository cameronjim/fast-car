import { useEffect, useState } from "preact/hooks";
import { App } from "./App";
import { DemoRoot, isDemo } from "./demo";
import {
  carLabel,
  carTarget,
  loadPlatform,
  runSearch,
  searchText,
  type Car,
  type Desktop,
  type DiscoveryState,
  type Platform,
  type SearchView,
  type TokenStorageInfo,
} from "./platform";

/** Picks the right front: the plain dashboard in a browser, the car finder in the desktop app. */
export function Root() {
  if (isDemo(globalThis.location?.search ?? "")) return <DemoRoot search={globalThis.location.search} />;
  return <RealRoot />;
}

function RealRoot() {
  const [platform, setPlatform] = useState<Platform | null>(null);
  useEffect(() => {
    void loadPlatform().then(setPlatform);
  }, []);
  if (!platform) return <Starting />;
  if (platform.kind === "web") return <App target={platform.target!} tokens={platform.tokens} />;
  return <DesktopShell desktop={platform.desktop!} />;
}

function Starting() {
  return (
    <div class="login" aria-busy="true">
      <h1>Racer</h1>
      <div class="skeleton" style={{ height: 14, width: "70%" }} />
      <div class="skeleton" style={{ height: 14, width: "50%" }} />
      <div class="sub">Starting...</div>
    </div>
  );
}

function DesktopShell({ desktop }: { desktop: Desktop }) {
  const [state, setState] = useState<DiscoveryState | null>(null);
  const [choosing, setChoosing] = useState(false);
  const [storage, setStorage] = useState<TokenStorageInfo | null>(desktop.tokenStorage.current());

  useEffect(() => desktop.tokenStorage.subscribe(setStorage), [desktop]);

  useEffect(() => {
    let off: (() => void) | null = null;
    let alive = true;
    void desktop.state().then((s) => alive && setState(s));
    void desktop.onState((s) => alive && setState(s)).then((fn) => {
      if (alive) off = fn;
      else fn();
    });
    return () => {
      alive = false;
      off?.();
    };
  }, [desktop]);

  if (!state) return <Starting />;

  if (!state.car || choosing) {
    return (
      <Picker
        desktop={desktop}
        state={state}
        onPicked={() => setChoosing(false)}
        onCancel={state.car ? () => setChoosing(false) : undefined}
      />
    );
  }

  const car = state.car;
  return (
    <>
      <div class="carbar">
        <span class={`pill ${state.phase === "connected" ? "lvl-green" : "lvl-amber"}`}>
          {state.phase === "connected" ? "Car found" : "Looking for the car"}
        </span>
        <span class="muted grow">{carLabel(car)}</span>
        <button class="link" onClick={() => setChoosing(true)}>
          Change car
        </button>
      </div>
      {state.phase === "lost" && (
        <p class="banner" role="status">
          Lost contact with the car at {car.host}. Searching again. A rebooting Jetson comes back by itself.
        </p>
      )}
      <App
        key={`${car.host}:${car.port}`}
        target={carTarget(car)}
        tokens={desktop.tokens}
        fetchImpl={desktop.fetch}
        tokenStorage={storage}
        carName={carLabel(car)}
      />
    </>
  );
}

function Picker({
  desktop,
  state,
  onPicked,
  onCancel,
}: {
  desktop: Desktop;
  state: DiscoveryState;
  onPicked: () => void;
  onCancel?: () => void;
}) {
  const [address, setAddress] = useState("");
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [search, setSearch] = useState<SearchView>({ kind: "idle" });
  const searching = search.kind === "searching";

  const pick = async (car: Car) => {
    setError("");
    try {
      await desktop.selectCar(car);
      onPicked();
    } catch (e) {
      setError(String(e));
    }
  };
  const manual = async (e: Event) => {
    e.preventDefault();
    setBusy(true);
    setError("");
    try {
      await desktop.addManual(address);
      onPicked();
    } catch (err) {
      setError(String(err));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div class="login">
      <h1>Racer</h1>
      <p class="sub">
        {state.cars.length === 0 ? "Looking for the car on this network..." : "Cars found on this network:"}
      </p>
      <ul class="cars">
        {state.cars.map((c) => (
          <li key={`${c.host}:${c.port}`}>
            <button class="btn" onClick={() => void pick(c)}>{carLabel(c)}</button>
            <span class="muted"> via {c.source}</span>
          </li>
        ))}
      </ul>
      <button class="btn" disabled={searching} onClick={() => void runSearch(desktop, setSearch)}>
        {searching ? (<><span class="spinner" />Searching...</>) : "Search again"}
      </button>
      {search.kind !== "idle" && (
        <div class={search.kind === "none" || search.kind === "error" ? "warn" : "sub"} role="status">
          {searchText(search)}
        </div>
      )}
      <form onSubmit={manual}>
        <label class="field">
          <span>Or enter the address (192.168.1.50 or racer-car.local:8080)</span>
          <input type="text" value={address} autocomplete="off" onInput={(e) => setAddress(e.currentTarget.value)} />
        </label>
        <button class="btn primary" type="submit" disabled={!address || busy} aria-busy={busy}>
          Connect
        </button>
        {error && <div class="warn">{error}</div>}
      </form>
      {onCancel && (
        <button class="link" onClick={onCancel}>
          Back
        </button>
      )}
    </div>
  );
}
