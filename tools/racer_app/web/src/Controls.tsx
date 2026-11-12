import { useState } from "preact/hooks";
import { ApiError, type Api, type ActionReply, type BagInfo, type LogEntry, type Options, type Status } from "./api";

export type Notify = (text: string, kind?: "ok" | "error") => void;

interface Props {
  api: Api;
  status: Status | null;
  options: Options | null;
  bags: BagInfo[];
  log: LogEntry[];
  notify: Notify;
  refresh: () => void;
}

/** Run an action, toast the outcome, refresh. Returns the error so callers can react. */
export async function run(
  fn: () => Promise<ActionReply>,
  notify: Notify,
  refresh: () => void,
): Promise<ApiError | null> {
  try {
    const r = await fn();
    notify(r.message ?? "done", "ok");
    refresh();
    return null;
  } catch (e) {
    const err = e instanceof ApiError ? e : new ApiError(String(e), 0, null);
    notify(err.message, "error");
    refresh();
    return err;
  }
}

/**
 * A button that asks for a second click instead of window.confirm, which the desktop app's
 * webview does not show (the click silently did nothing). The first click arms it for a few
 * seconds and shows the question; the second click runs the action.
 */
export function ConfirmButton(p: {
  label: string;
  question: string;
  onConfirm: () => void;
  disabled?: boolean;
  class?: string;
}) {
  const [armed, setArmed] = useState(false);
  const click = () => {
    if (armed) {
      setArmed(false);
      p.onConfirm();
      return;
    }
    setArmed(true);
    setTimeout(() => setArmed(false), 4000);
  };
  return (
    <button class={p.class} disabled={p.disabled} title={p.question} onClick={click}>
      {armed ? `${p.question} Click again` : p.label}
    </button>
  );
}

export function Controls(p: Props) {
  const { api, status, options, notify, refresh } = p;
  const stackUp = status?.stack.running ?? false;
  const driving = status?.drive.mode ?? null;

  // stack flags
  const [lidar, setLidar] = useState(true);
  const [vesc, setVesc] = useState(true);
  const [mode, setMode] = useState<"plain" | "mapping" | "localization">("plain");
  const [map, setMap] = useState("");

  // driving
  const [gapProfile, setGapProfile] = useState("lane-2026-10-25");
  const [parkProfile, setParkProfile] = useState("floor-2026-10-07");
  const [raceline, setRaceline] = useState("");
  const [policyName, setPolicyName] = useState("");
  const [pnpBackend, setPnpBackend] = useState("geometric");
  const [pnpDynamics, setPnpDynamics] = useState("");
  const [pnpPerception, setPnpPerception] = useState("");
  const [pnpSpeed, setPnpSpeed] = useState(0.5);
  const [override, setOverride] = useState(false);
  const [reason, setReason] = useState("");
  const [needsOverride, setNeedsOverride] = useState<string | null>(null);
  const [mapName, setMapName] = useState("");

  // perception and the spiderbot
  const [patrol, setPatrol] = useState("");
  const [rule, setRule] = useState("arms_raised");
  const [patrolSpeed, setPatrolSpeed] = useState(0.5);

  const act = (fn: () => Promise<ActionReply>) => run(fn, notify, refresh);

  const drive = async (body: {
    mode: string;
    profile?: string;
    raceline?: string;
    policy?: string;
    patrol?: string;
    rule?: string;
    speed_mps?: number;
    pnp_perception?: string;
    pnp_dynamics?: string;
    pnp_backend?: string;
    pnp_v_max_mps?: number;
  }) => {
    const err = await act(() =>
      api.startDrive({ ...body, override, override_reason: override ? reason : "" }),
    );
    setNeedsOverride(err?.body?.overridable ? err.message : null);
  };

  const parkCmd = async (name: string) => {
    const err = await act(() => api.park(name, override, override ? reason : ""));
    setNeedsOverride(err?.body?.overridable ? err.message : null);
  };

  const policy = options?.policy ?? status?.policy ?? null;
  const pnp = options?.pnp ?? null;
  const pnpRange = pnp?.speed_range ?? [0.2, 1.5];
  const pnpNeedsPerception = pnpBackend === "net";
  const pnpReady =
    !!pnp?.available &&
    stackUp &&
    !driving &&
    (pnp.dynamics.length === 1 || !!pnpDynamics) &&
    (!pnpNeedsPerception || pnp.perception.length === 1 || !!pnpPerception) &&
    pnpSpeed >= pnpRange[0] &&
    pnpSpeed <= pnpRange[1];
  const perception = status?.perception ?? options?.perception ?? null;
  const spiderbot = status?.spiderbot ?? options?.spiderbot ?? null;
  const speedRange = options?.spiderbot_speed_range ?? [0.1, 1.0];

  return (
    <>
      <section class="card" aria-label="Stack">
        <h3>Stack <span class={`pill ${stackUp ? "lvl-green" : "lvl-red"}`}>{stackUp ? "running" : "stopped"}</span></h3>
        <div class="row">
          <label><input type="checkbox" checked={lidar} onChange={(e) => setLidar(e.currentTarget.checked)} /> LiDAR</label>
          <label><input type="checkbox" checked={vesc} onChange={(e) => setVesc(e.currentTarget.checked)} /> VESC</label>
        </div>
        <div class="row">
          <label><input type="radio" name="sm" checked={mode === "plain"} onChange={() => setMode("plain")} /> teleop only</label>
          <label><input type="radio" name="sm" checked={mode === "mapping"} onChange={() => setMode("mapping")} /> mapping</label>
          <label><input type="radio" name="sm" checked={mode === "localization"} onChange={() => setMode("localization")} /> localisation</label>
        </div>
        {mode === "localization" && (
          <div class="row">
            <select value={map} onChange={(e) => setMap(e.currentTarget.value)} aria-label="Map">
              <option value="">choose a map</option>
              {options?.maps.map((m) => <option key={m} value={m}>{m}</option>)}
            </select>
          </div>
        )}
        <div class="row">
          <button disabled={stackUp} onClick={() => act(() => api.startStack({ lidar, vesc, mapping: mode === "mapping", localization: mode === "localization", map: mode === "localization" ? map : "" }))}>Start stack</button>
          <button disabled={!stackUp} onClick={() => act(() => api.restartStack())}>Restart</button>
          <ConfirmButton class="danger" disabled={!stackUp} label="Stop stack" question="Stop the stack? Recording stops." onConfirm={() => act(() => api.stopStack())} />
        </div>
        <div class="sub">Starts car-stack with the recorder. Running now: {stackUp ? Object.entries(status?.stack.flags ?? {}).map(([k, v]) => `${k}=${v}`).join(" ") || "default" : "nothing"}.</div>
        <div class="row">
          <input placeholder="map name" value={mapName} onInput={(e) => setMapName(e.currentTarget.value)} aria-label="Map name" />
          <button disabled={!stackUp || status?.stack.flags.mapping !== "true" || !mapName} onClick={() => act(() => api.saveMap(mapName))}>Save map</button>
        </div>
      </section>

      <section class="card" aria-label="Cameras">
        <h3>Cameras <span class={`pill ${status?.cameras.running ? "lvl-green" : "lvl-amber"}`}>{status?.cameras.running ? "on" : "off"}</span></h3>
        <div class="row">
          <button disabled={status?.cameras.running} onClick={() => act(() => api.startCameras())}>Start cameras</button>
          <button disabled={!status?.cameras.running} onClick={() => act(() => api.stopCameras())}>Stop cameras</button>
        </div>
        <div class="sub">About 6 MB/s per camera into the bag. Needs {status?.thresholds.cameras_min_free_gb ?? 10} GiB free; the disk guard stops them when space runs low.</div>
      </section>

      <section class="card" aria-label="Perception">
        <h3>Perception <span class={`pill ${perception?.running ? "lvl-green" : "lvl-amber"}`}>{perception?.running ? "on" : "off"}</span></h3>
        <div class="row">
          <button
            disabled={!perception?.available || perception?.running || !status?.cameras.running}
            title={perception?.reason}
            aria-describedby="perception-reason"
            onClick={() => act(() => api.startPerception({ mode: "pose", device: "cuda" }))}
          >Start perception (pose, GPU)</button>
          <button disabled={!perception?.running} onClick={() => act(() => api.stopPerception())}>Stop perception</button>
        </div>
        <div id="perception-reason" class="sub">
          {perception?.available
            ? status?.cameras.running ? "Runs the person and pose detector on the camera. Drives nothing." : "Start the cameras first."
            : `disabled: ${perception?.reason ?? "unknown"}`}
        </div>
      </section>

      <section class="card" aria-label="Driving">
        <h3>Driving <span class={`pill ${driving ? "lvl-amber" : "lvl-green"}`}>{driving ?? "idle"}</span></h3>
        <div class="warn">
          This is a software stop only. Wi-Fi can drop. Arming still needs the RC kill knob, and the knob is the stop that always works. Keep a hand on it.
        </div>
        <div class="row">
          <select value={gapProfile} onChange={(e) => setGapProfile(e.currentTarget.value)} aria-label="Gap follow profile">
            {options?.gap_follow_profiles.map((x) => <option key={x}>{x}</option>)}
          </select>
          <button disabled={!stackUp || !!driving} onClick={() => drive({ mode: "gap_follow", profile: gapProfile })}>Gap follow</button>
        </div>
        <div class="row">
          <button disabled={!stackUp || !!driving} onClick={() => drive({ mode: "wall_follow" })}>Wall follow</button>
        </div>
        <div class="row">
          <select value={raceline} onChange={(e) => setRaceline(e.currentTarget.value)} aria-label="Raceline">
            <option value="">raceline</option>
            {options?.racelines.map((x) => <option key={x}>{x}</option>)}
          </select>
          <button disabled={!stackUp || !!driving || !raceline} onClick={() => drive({ mode: "tracker", raceline })}>Tracker</button>
        </div>
        <div class="sub">Tracker needs the stack with localisation and a live pose, and a raceline written in that map's frame.</div>
        <div class="row">
          <select value={parkProfile} onChange={(e) => setParkProfile(e.currentTarget.value)} aria-label="Park profile">
            {options?.park_profiles.map((x) => <option key={x}>{x}</option>)}
          </select>
          <button disabled={!stackUp || !!driving} onClick={() => drive({ mode: "park", profile: parkProfile })}>Park launch</button>
        </div>
        <div class="row">
          <button disabled={!driving?.startsWith("park")} onClick={() => parkCmd("start")}>Park</button>
          <button disabled={!driving?.startsWith("park")} onClick={() => parkCmd("three_point_turn")}>Three-point turn</button>
          <button disabled={!driving?.startsWith("park")} onClick={() => act(() => api.park("abort"))}>Abort</button>
        </div>
        <div class="row">
          <select value={patrol} onChange={(e) => setPatrol(e.currentTarget.value)} aria-label="Patrol">
            <option value="">patrol</option>
            {(options?.patrols ?? []).map((x) => <option key={x}>{x}</option>)}
          </select>
          <select value={rule} onChange={(e) => setRule(e.currentTarget.value)} aria-label="Pose rule">
            {(options?.spiderbot_rules ?? ["arms_raised"]).map((x) => <option key={x}>{x}</option>)}
          </select>
          <input
            type="number"
            aria-label="Patrol speed in metres per second"
            min={speedRange[0]}
            max={speedRange[1]}
            step={0.1}
            value={patrolSpeed}
            onInput={(e) => setPatrolSpeed(Number(e.currentTarget.value))}
          />
          <button
            disabled={!spiderbot?.available || !stackUp || !!driving || !patrol}
            title={spiderbot?.reason}
            aria-describedby="spiderbot-reason"
            onClick={() => drive({ mode: "spiderbot", patrol, rule, speed_mps: patrolSpeed })}
          >Spiderbot patrol</button>
        </div>
        <div id="spiderbot-reason" class="sub">
          {spiderbot?.available
            ? "Patrols the loop on the map. Stops, turns to face a person who holds the pose, raises an alert, then resumes. Needs localisation and perception running. Goes through the safety node like every other drive."
            : `disabled: ${spiderbot?.reason ?? "unknown"}`}
        </div>
        <div class="row">
          <select value={policyName} onChange={(e) => setPolicyName(e.currentTarget.value)} aria-label="ML policy" disabled={!policy?.available}>
            <option value="">{policy?.available ? "policy" : "no usable policy"}</option>
            {policy?.valid.map((x) => <option key={x}>{x}</option>)}
          </select>
          <button
            disabled={!policy?.available || !stackUp || !!driving || !raceline || ((policy?.valid.length ?? 0) > 1 && !policyName)}
            title={policy?.available ? "Tracker plus the learned residual, on the raceline chosen above" : policy?.reason}
            aria-describedby="ml-reason"
            onClick={() => drive({ mode: "ml_policy", raceline, policy: policyName })}
          >ML policy</button>
        </div>
        <div id="ml-reason" class="sub">
          {policy?.available
            ? "Tracker plus a bounded learned residual, on the raceline chosen above. Same localisation needs as the tracker. The policy node refuses to start on any mismatch with this car's parameters. First trials: wheels off the ground, hand on the knob."
            : `disabled: ${policy?.reason ?? "unknown"}`}
        </div>
        <div class="row">
          <select value={pnpBackend} onChange={(e) => setPnpBackend(e.currentTarget.value)} aria-label="Plug-and-Play perception backend" disabled={!pnp?.available}>
            {(pnp?.backends ?? ["geometric", "net"]).map((x) => <option key={x}>{x}</option>)}
          </select>
          <select value={pnpDynamics} onChange={(e) => setPnpDynamics(e.currentTarget.value)} aria-label="Plug-and-Play dynamics bundle" disabled={!pnp?.available}>
            <option value="">{pnp?.dynamics.length === 1 ? pnp.dynamics[0] : "dynamics bundle"}</option>
            {pnp?.dynamics.map((x) => <option key={x}>{x}</option>)}
          </select>
          {pnpNeedsPerception && (
            <select value={pnpPerception} onChange={(e) => setPnpPerception(e.currentTarget.value)} aria-label="Plug-and-Play perception bundle" disabled={!pnp?.available}>
              <option value="">{pnp?.perception.length === 1 ? pnp.perception[0] : "perception bundle"}</option>
              {pnp?.perception.map((x) => <option key={x}>{x}</option>)}
            </select>
          )}
        </div>
        <div class="row">
          <label>
            Speed cap {pnpSpeed.toFixed(2)} m/s
            <input
              type="range"
              aria-label="Plug-and-Play speed cap"
              min={pnpRange[0]}
              max={pnpRange[1]}
              step={0.05}
              value={pnpSpeed}
              onInput={(e) => setPnpSpeed(Number(e.currentTarget.value))}
              disabled={!pnp?.available}
            />
          </label>
          <button
            disabled={!pnpReady}
            title={pnp?.available ? "Mapless driver: track perception plus the MPPI planner" : pnp?.reason}
            aria-describedby="pnp-reason"
            onClick={() =>
              drive({
                mode: "pnp",
                pnp_backend: pnpBackend,
                pnp_dynamics: pnpDynamics,
                pnp_perception: pnpNeedsPerception ? pnpPerception : "",
                pnp_v_max_mps: pnpSpeed,
              })
            }
          >Plug-and-Play</button>
        </div>
        <div id="pnp-reason" class="sub">
          {pnp?.available
            ? "Drives a track it has never seen: no map, no pose. Needs only the stack and its recorder. The nodes refuse any bundle that does not match this car. Start at a low cap, wheels off the ground first, hand on the knob."
            : `disabled: ${pnp?.reason ?? "unknown"}`}
        </div>
        {needsOverride && (
          <div class="override">
            <div>{needsOverride}</div>
            <label><input type="checkbox" checked={override} onChange={(e) => setOverride(e.currentTarget.checked)} /> Override the checks and drive anyway (logged)</label>
            {override && <input placeholder="reason (required)" value={reason} onInput={(e) => setReason(e.currentTarget.value)} aria-label="Override reason" />}
          </div>
        )}
        <div class="sub">A driving run needs the stack and its recorder running. That one cannot be overridden.</div>
      </section>

      <section class="card" aria-label="System">
        <h3>System</h3>
        <label class="row"><input type="checkbox" checked={status?.autostart ?? false} onChange={(e) => act(() => api.setAutostart(e.currentTarget.checked))} /> Start the stack at boot (LiDAR and VESC, no cameras)</label>
        <div class="row">
          <ConfirmButton class="danger" label="Reboot" question="Reboot the Jetson?" onConfirm={() => act(() => api.power("reboot"))} />
          <ConfirmButton class="danger" label="Shut down" question="Shut the Jetson down?" onConfirm={() => act(() => api.power("shutdown"))} />
        </div>
      </section>

      <section class="card" aria-label="Bags">
        <h3>Bags <span class="muted">{p.bags.length} folders</span></h3>
        <ul class="bags">
          {p.bags.map((b) => (
            <li key={b.name}>
              <span>{b.name}</span>
              <span class="muted">{b.size_mb >= 1000 ? `${(b.size_mb / 1000).toFixed(1)} GB` : `${b.size_mb.toFixed(0)} MB`}{b.active ? " recording" : ""}</span>
              <ConfirmButton disabled={b.active} label="Delete" question="Delete for good?" onConfirm={() => act(() => api.deleteBag(b.name))} />
            </li>
          ))}
          {p.bags.length === 0 && <li class="muted">No bags yet.</li>}
        </ul>
      </section>

      <section class="card" aria-label="Action log">
        <h3>Action log</h3>
        <ul class="actionlog">
          {p.log.map((e, i) => (
            <li key={`${e.ts}-${i}`} class={e.ok ? "" : "bad"}>
              <span class="muted">{new Date(e.ts * 1000).toLocaleTimeString()}</span> {e.action}{e.override ? " [OVERRIDE]" : ""} {e.detail}
            </li>
          ))}
        </ul>
      </section>
    </>
  );
}
