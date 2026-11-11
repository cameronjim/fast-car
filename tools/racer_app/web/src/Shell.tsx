import { useEffect, useState } from "preact/hooks";
import { App } from "./App";
import {
  carLabel,
  carTarget,
  loadPlatform,
  type Car,
  type Desktop,
  type DiscoveryState,
  type Platform,
} from "./platform";

/** Picks the right front: the plain dashboard in a browser, the car finder in the desktop app. */
export function Root() {
  const [platform, setPlatform] = useState<Platform | null>(null);
  useEffect(() => {
    void loadPlatform().then(setPlatform);
  }, []);
  if (!platform) return <div class="app muted">Starting</div>;
  if (platform.kind === "web") return <App target={platform.target!} tokens={platform.tokens} />;
  return <DesktopShell desktop={platform.desktop!} />;
}

function DesktopShell({ desktop }: { desktop: Desktop }) {
  const [state, setState] = useState<DiscoveryState | null>(null);
  const [choosing, setChoosing] = useState(false);

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

  if (!state) return <div class="app muted">Starting</div>;

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
          {state.phase === "connected" ? "car found" : "looking for the car"}
        </span>
        <span class="muted">{carLabel(car)}</span>
        <button class="link" onClick={() => setChoosing(true)}>
          Change car
        </button>
      </div>
      {state.phase === "lost" && (
        <p class="banner">
          Lost contact with the car at {car.host}. Searching the network again. If the Jetson is rebooting this
          comes back by itself.
        </p>
      )}
      <App key={`${car.host}:${car.port}`} target={carTarget(car)} tokens={desktop.tokens} fetchImpl={desktop.fetch} />
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
    <div class="app login">
      <h1>Racer</h1>
      <p class="sub">
        {state.cars.length === 0
          ? "Looking for the car on this network (mDNS, then a scan of the local subnet)."
          : "Cars found on this network:"}
      </p>
      <ul class="cars">
        {state.cars.map((c) => (
          <li key={`${c.host}:${c.port}`}>
            <button onClick={() => void pick(c)}>{carLabel(c)}</button>
            <span class="muted"> via {c.source}</span>
          </li>
        ))}
      </ul>
      <button class="link" onClick={() => void desktop.rescan()}>
        Search again
      </button>
      <form onSubmit={manual}>
        <label>
          Or enter the address (for example 192.168.1.50 or racer-car.local:8080)
          <input type="text" value={address} autocomplete="off" onInput={(e) => setAddress(e.currentTarget.value)} />
        </label>
        <button type="submit" disabled={!address || busy}>
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
