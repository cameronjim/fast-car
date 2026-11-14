// The Drive tab: status lights, Start car, the three drive buttons, Stop driving and the camera
// recording switch. Every button shows a spinner while its request is out and then follows what
// the car reports (status poll), including drives started from a terminal. The values each drive
// button sends come from the Tune tab's saved settings.

import { useApp, useRunner } from "./ctx";
import { useDriveStarter, type DriveBody } from "./modes";
import {
  gapBody,
  gapSummary,
  gapWhy,
  pnpBody,
  pnpSummary,
  pnpWhy,
  stackBody,
  trackerBody,
  trackerSummary,
  trackerWhy,
} from "./prefs";
import { ActionButton, Card, ErrorBadge, Icon, Light, SkeletonCard, Spinner, Toggle, type IconName } from "./ui";
import { driveBlocked, driveState, modeLabel, modeMatches, serviceState, stackState } from "./view";

export function DriveTab() {
  const c = useApp();
  const { status, options, api, prefs } = c;
  const { drive, panel } = useDriveStarter();

  if (!status || !options) {
    return (
      <div class="dash" aria-busy="true">
        <SkeletonCard lines={2} />
        <SkeletonCard lines={4} />
      </div>
    );
  }

  const blocked = plainWords(driveBlocked(status, c.ops, c.agentUp));
  const d = driveState(status, c.ops);
  const stack = stackState(status, c.ops);
  const cams = serviceState(status.cameras.running, c.ops, ["cameras-start"], ["cameras-stop"]);
  const toLogs = () => c.setTab("logs");

  const startCar = () => {
    const p = c.runOp({ kind: "stack-start", title: "Starting the car" }, () => api.startStack(stackBody(prefs)));
    // The cameras follow once the stack was accepted (a failed start leaves them off).
    void p.then(
      () => {
        if (!c.status?.cameras.running && !c.ops.activeOf("cameras-start")) {
          void c.runOp({ kind: "cameras-start" }, () => api.startCameras()).catch(() => undefined);
        }
      },
      () => undefined,
    );
    return p;
  };
  const stopCar = () => {
    if (status.cameras.running) void c.runOp({ kind: "cameras-stop" }, () => api.stopCameras()).catch(() => undefined);
    return c.runOp({ kind: "stack-stop", title: "Stopping the car" }, () => api.stopStack());
  };

  const camRecording = status.cameras.running && status.cameras.record != null ? status.cameras.record : prefs.record_camera;
  const external = status.drive.external ?? [];

  return (
    <div class="dash">
      <Card title="Status" icon="pulse" class="span-all">
        <div class="lights">
          <CarLight />
          <Light
            label="Stack"
            icon="layers"
            level={stack === "on" ? (status.stack.recorder ? "green" : "red") : stack === "off" ? "off" : "amber"}
            busy={stack !== "on" && stack !== "off"}
            text={stack === "on" ? (status.stack.recorder ? "Running" : "No recorder") : stack === "off" ? "Off" : stack === "stopping" ? "Stopping" : "Starting"}
          />
          <SafetyLight />
          <Light
            label="Camera"
            icon="camera"
            level={cams === "on" ? "green" : cams === "off" ? "off" : "amber"}
            busy={cams === "starting" || cams === "stopping"}
            text={cams === "on" ? (status.cameras.record === false ? "On" : "REC") : cams === "off" ? "Off" : cams === "starting" ? "Starting" : "Stopping"}
          />
        </div>
        {(d.state !== "idle" || external.length > 0) && (
          <div class={`driving ${d.state}`} role="status">
            {d.state === "running" || d.state === "idle" ? <span class="ldot pulse" /> : <Spinner />}
            <b>{d.state === "starting" ? "Starting" : d.state === "stopping" ? "Stopping" : "Driving"}</b>
            <span>{modeLabel(d.mode ?? external[0]?.mode ?? "")}</span>
            {external.length > 0 && <span class="tag">terminal</span>}
          </div>
        )}
      </Card>

      <Card
        title="Drive"
        icon="wheel"
        class="span-all"
        tools={
          <button class="btn ghost sm" onClick={() => c.setTab("tune")}>
            <Icon name="sliders" />
            Tune
          </button>
        }
      >
        <div class="car-row">
          {stack === "on" || stack === "stopping" ? (
            <ActionButton
              label="Stop car"
              icon="power"
              pendingLabel="Closing the bag..."
              variant="danger"
              size="lg"
              confirm="Stop car? Recording ends"
              disabled={stack !== "on" || !c.agentUp}
              ariaLabel="Stop car"
              onAction={stopCar}
              onErrorBadge={toLogs}
            />
          ) : (
            <ActionButton
              label="Start car"
              icon="power"
              pendingLabel="Starting..."
              variant="primary"
              size="lg"
              disabled={stack !== "off" || !c.agentUp || !c.prefsLoaded}
              title={c.agentUp ? undefined : "Car offline"}
              ariaLabel="Start car"
              onAction={startCar}
              onErrorBadge={toLogs}
            />
          )}
          <Toggle
            label="Record camera"
            checked={camRecording}
            disabled={status.cameras.running || !c.prefsLoaded || !c.agentUp}
            title={status.cameras.running ? "Set before Start car. LiDAR and telemetry always record." : "LiDAR and telemetry always record."}
            onChange={(next) => c.savePrefs({ ...prefs, record_camera: next })}
            onErrorBadge={toLogs}
          />
        </div>

        {panel}

        <div class="tiles">
          <Tile
            mode="gap_follow"
            label="Gap follow"
            icon="route"
            summary={gapSummary(prefs)}
            why={blocked || gapWhy(prefs, options)}
            onStart={() => drive("gap follow", gapBody(prefs) as DriveBody)}
          />
          <Tile
            mode="pnp"
            label="ML driver"
            icon="chip"
            summary={pnpSummary(prefs, options)}
            why={blocked || pnpWhy(prefs, options)}
            onStart={() => drive("ML driver", pnpBody(prefs, options) as DriveBody)}
          />
          <Tile
            mode="tracker"
            label="Racer"
            icon="flag"
            summary={trackerSummary(prefs)}
            why={blocked || trackerWhy(prefs, status)}
            onStart={() => drive("racer", trackerBody(prefs) as DriveBody)}
          />
        </div>

        <ActionButton
          label="Stop driving"
          icon="stop"
          pendingLabel="Stopping..."
          successLabel="Stopped"
          variant="danger"
          size="lg"
          block
          class="stop-driving"
          disabled={d.state === "idle" && external.length === 0}
          ariaLabel="Stop driving"
          onAction={() => c.stopAll()}
          onErrorBadge={toLogs}
        />
        <div class="sub center">Software stop. The RC kill knob always works.</div>
      </Card>
    </div>
  );
}

/** The agent's reasons, in the words of this tab ("Start car", "Gap follow"). */
function plainWords(why: string): string {
  if (why === "Start the stack first") return "Start car first";
  if (why === "Stack starting") return "Car starting";
  if (why === "Stack stopping") return "Car stopping";
  const m = /^(\S+) is running( from a terminal)?$/.exec(why);
  return m ? `${modeLabel(m[1])} is running${m[2] ?? ""}` : why;
}

function CarLight() {
  const c = useApp();
  const stackUp = c.status?.stack.running ?? false;
  if (!c.agentUp) return <Light label="Car" icon="car" level="red" text="Offline" busy />;
  if (stackUp && !c.link.connected) return <Light label="Car" icon="car" level="amber" text="No live data" />;
  return <Light label="Car" icon="car" level="green" text="Online" />;
}

function SafetyLight() {
  const c = useApp();
  const s = c.status;
  if (!s?.stack.running) return <Light label="Safety" icon="shield" level="off" text="Off" />;
  const check = s.checks.checks.find((x) => x.id === "safety");
  const level = check?.level ?? "amber";
  return <Light label="Safety" icon="shield" level={level} text={level === "green" ? "OK" : level === "red" ? "Down" : "Check"} />;
}

/** One big drive button: name, icon, what it will run with, and its real state. */
function Tile(p: { mode: string; label: string; icon: IconName; summary: string; why: string; onStart: () => Promise<unknown> }) {
  const c = useApp();
  const runner = useRunner({ errorMs: 20_000 });
  const d = driveState(c.status, c.ops);
  const mine = modeMatches(d.mode, p.mode);
  const running = mine && d.state === "running";
  const starting = mine && d.state === "starting";
  const stopping = mine && d.state === "stopping";
  const pending = runner.state.phase === "pending" || starting;
  const disabled = running || pending || stopping || (!!p.why && !mine);
  const state = running ? "running" : pending ? "starting" : stopping ? "stopping" : p.why ? "blocked" : "idle";
  const sub = running ? "Running" : pending ? "Starting..." : stopping ? "Stopping..." : p.why || p.summary;
  return (
    <div class="tile-wrap">
      <button
        class={`tile ${state}`}
        disabled={disabled}
        aria-busy={pending}
        aria-label={`${p.label}: ${sub}`}
        title={p.why || p.summary}
        onClick={() => void runner.run(async () => (await p.onStart()) as { message?: string } | void)}
      >
        <span class="tile-ic">{pending || stopping ? <Spinner /> : <Icon name={p.icon} size={26} />}</span>
        <span class="tile-name">{p.label}</span>
        <span class="tile-sub">{sub}</span>
      </button>
      {runner.state.phase === "error" && <ErrorBadge title={runner.state.error} onClick={() => c.setTab("logs")} />}
    </div>
  );
}
