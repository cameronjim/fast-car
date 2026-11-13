// The System tab (the gear at the bottom of the sidebar): stack start, stop and restart with all
// their flags, cameras, perception, autostart, reboot and shut down, where this app keeps its
// token, and the less common drive modes (park, wall follow, ML policy, spiderbot).

import { useState } from "preact/hooks";
import { useApp } from "./ctx";
import { OtherDrives } from "./modes";
import { ActionButton, Card, SkeletonCard, StateBadge, Toggle } from "./ui";
import { elapsedLabel, serviceState, stackState, type ServiceState } from "./view";

function badge(st: ServiceState | "restarting", on = "Running", off = "Off") {
  if (st === "on") return <StateBadge kind="running">{on}</StateBadge>;
  if (st === "off") return <StateBadge kind="idle">{off}</StateBadge>;
  const text = st === "starting" ? "Starting..." : st === "restarting" ? "Restarting..." : "Stopping...";
  return <StateBadge kind={st === "stopping" ? "stopping" : "starting"}>{text}</StateBadge>;
}

export function SystemTab() {
  const c = useApp();
  const { status, options, api } = c;
  const [lidar, setLidar] = useState(true);
  const [vesc, setVesc] = useState(true);
  const [mode, setMode] = useState<"plain" | "mapping" | "localization">("plain");
  const [map, setMap] = useState("");
  const [mapName, setMapName] = useState("");

  if (!status) {
    return (
      <div class="grid">
        {Array.from({ length: 4 }, (_, i) => <SkeletonCard key={i} lines={2} />)}
      </div>
    );
  }

  const stack = stackState(status, c.ops);
  const cams = serviceState(status.cameras.running, c.ops, ["cameras-start"], ["cameras-stop"]);
  const perc = status.perception ?? options?.perception;
  const percState = serviceState(perc?.running ?? false, c.ops, ["perception-start"], ["perception-stop"]);
  const off = !c.agentUp;
  const flags = status.stack.flags;
  const stopOp = c.ops.activeOf("stack-stop", "stack-restart");
  const needMap = mode === "localization" && !map;

  return (
    <div class="grid">
      <Card
        title="Stack"
        info="Starts the car-stack container with the bag recorder. Stopping waits for the recorder to close the bag, which takes minutes for a big one: do not unplug anything."
        badge={badge(stack)}
      >
        <div class="inner-row">
          <label class="check"><input type="checkbox" checked={lidar} onChange={(e) => setLidar(e.currentTarget.checked)} /> LiDAR</label>
          <label class="check"><input type="checkbox" checked={vesc} onChange={(e) => setVesc(e.currentTarget.checked)} /> VESC</label>
        </div>
        <div class="seg" role="group" aria-label="Stack mode">
          <button aria-pressed={mode === "plain"} onClick={() => setMode("plain")}>Teleop</button>
          <button aria-pressed={mode === "mapping"} onClick={() => setMode("mapping")}>Mapping</button>
          <button aria-pressed={mode === "localization"} onClick={() => setMode("localization")}>Localisation</button>
        </div>
        {mode === "localization" && (
          <label class="field">
            <span>Map</span>
            <select value={map} onChange={(e) => setMap(e.currentTarget.value)} aria-label="Map">
              <option value="">choose a map</option>
              {options?.maps.map((m) => <option key={m} value={m}>{m}</option>)}
            </select>
          </label>
        )}
        <div class="btnrow">
          <ActionButton
            label="Start"
            pendingLabel="Starting..."
            successLabel="Started"
            variant="primary"
            disabled={stack !== "off" || off || needMap}
            title={needMap ? "Choose a map" : stack !== "off" ? "Already running" : undefined}
            ariaLabel="Start stack"
            onAction={() =>
              c.runOp({ kind: "stack-start" }, () =>
                api.startStack({ lidar, vesc, mapping: mode === "mapping", localization: mode === "localization", map: mode === "localization" ? map : "" }),
              )
            }
          />
          <ActionButton
            label="Restart"
            pendingLabel="Restarting..."
            successLabel="Restarted"
            disabled={stack !== "on" || off}
            ariaLabel="Restart stack"
            onAction={() => c.runOp({ kind: "stack-restart" }, () => api.restartStack())}
          />
          <ActionButton
            label="Stop"
            pendingLabel="Closing the bag..."
            successLabel="Stopped"
            variant="danger"
            confirm="Stop? Recording ends."
            disabled={stack !== "on" || off}
            ariaLabel="Stop stack"
            onAction={() => c.runOp({ kind: "stack-stop" }, () => api.stopStack())}
          />
        </div>
        {stopOp && (
          <div class="hint" role="status">
            Closing the bag, {elapsedLabel(c.now - stopOp.startedAt)} so far. A big bag takes a few minutes. Do not unplug anything.
          </div>
        )}
        {stack === "on" && (
          <div class="sub num">
            Running with {Object.entries(flags).map(([k, v]) => `${k}=${v}`).join(" ") || "defaults"}
          </div>
        )}
        {stack === "on" && flags.mapping === "true" && (
          <div class="inner-row">
            <input placeholder="Map name" value={mapName} onInput={(e) => setMapName(e.currentTarget.value)} aria-label="Map name" />
            <ActionButton
              label="Save map"
              pendingLabel="Saving..."
              successLabel="Saved"
              disabled={!mapName}
              onAction={() => c.act(() => api.saveMap(mapName))}
            />
          </div>
        )}
      </Card>

      <Card
        title="Cameras"
        info={`The bag takes a small 5 Hz stream per camera, only while a drive is running (about 0.25 MB/s per camera while driving, nothing while idle). The live view stays full rate. Needs ${status.thresholds.cameras_min_free_gb} GiB free; the disk guard stops them when space runs low.`}
        badge={badge(cams, "On", "Off")}
      >
        <div class="btnrow">
          <ActionButton
            label="Start"
            pendingLabel="Starting..."
            successLabel="Started"
            variant="primary"
            disabled={cams !== "off" || off}
            ariaLabel="Start cameras"
            onAction={() => c.runOp({ kind: "cameras-start" }, () => api.startCameras())}
          />
          <ActionButton
            label="Stop"
            pendingLabel="Stopping..."
            successLabel="Stopped"
            disabled={cams !== "on" || off}
            ariaLabel="Stop cameras"
            onAction={() => c.runOp({ kind: "cameras-stop" }, () => api.stopCameras())}
          />
        </div>
      </Card>

      <Card
        title="Perception"
        info="Runs the person and pose detector on the camera. Drives nothing. Needs the cameras running."
        badge={badge(percState, "On", "Off")}
      >
        <div class="btnrow">
          <ActionButton
            label="Start"
            pendingLabel="Starting..."
            successLabel="Started"
            variant="primary"
            disabled={!perc?.available || percState !== "off" || !status.cameras.running || off}
            title={perc?.available ? (status.cameras.running ? undefined : "Start the cameras first") : perc?.reason}
            ariaLabel="Start perception"
            onAction={() => c.runOp({ kind: "perception-start" }, () => api.startPerception({ mode: "pose", device: "cuda" }))}
          />
          <ActionButton
            label="Stop"
            pendingLabel="Stopping..."
            successLabel="Stopped"
            disabled={percState !== "on" || off}
            ariaLabel="Stop perception"
            onAction={() => c.runOp({ kind: "perception-stop" }, () => api.stopPerception())}
          />
        </div>
        {!perc?.available && <div class="why">Unavailable: {perc?.reason ?? "unknown"}</div>}
        {perc?.available && !status.cameras.running && percState === "off" && <div class="why">Start the cameras first</div>}
      </Card>

      <Card title="Startup" info="Starts the stack at boot with LiDAR and VESC, no cameras.">
        <div class="setting">
          <Toggle
            label="Start the stack at boot"
            checked={status.autostart}
            disabled={off}
            onChange={(next) => c.act(() => api.setAutostart(next))}
          />
        </div>
      </Card>

      <Card title="Power" info="Reboot and shut down act on the Jetson. A running stack closes its bag first.">
        <div class="btnrow">
          <ActionButton
            label="Reboot"
            pendingLabel="Rebooting..."
            variant="danger"
            confirm="Reboot the Jetson?"
            disabled={off}
            onAction={() => c.act(() => api.power("reboot"))}
          />
          <ActionButton
            label="Shut down"
            pendingLabel="Shutting down..."
            variant="danger"
            confirm="Shut it down?"
            disabled={off}
            onAction={() => c.act(() => api.power("shutdown"))}
          />
        </div>
      </Card>

      <Card
        title="Connection"
        info={c.tokenNote ?? "The access token is checked by the car on every request. The car stores only its hash, so a lost token is replaced with racer-agent init --rotate-token on the Jetson."}
      >
        <dl class="kv">
          <dt>Car</dt>
          <dd>{c.carName}</dd>
          <dt>Token kept in</dt>
          <dd>{c.tokenWhere}</dd>
        </dl>
        {c.tokenNote && <div class="warnline">{c.tokenNote}</div>}
      </Card>

      <h3 class="section span-all">More drives</h3>
      <OtherDrives />
    </div>
  );
}
