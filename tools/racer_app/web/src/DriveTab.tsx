// The Drive tab: one card per drive mode, each with its few controls, a big Start button and its
// current state. Safety gating, the override flow and every API call are the ones the agent
// already enforces; this file only decides what the buttons look like and say.

import type { ComponentChildren } from "preact";
import { useState } from "preact/hooks";
import { ApiError, type Options } from "./api";
import { useApp } from "./ctx";
import { externalDriveText, gapMinSpeed, gapSpeed, pnpSelectionText, selectedBundle, selectedParamsFile } from "./drivePick";
import { toApiError } from "./feedback";
import { ActionButton, Card, Icon, Pill, SkeletonCard, StateBadge } from "./ui";
import { driveBlocked, driveState, modeMatches, stackState } from "./view";

type DriveBody = Parameters<ReturnType<typeof useApp>["api"]["startDrive"]>[0];

export function DriveTab() {
  const c = useApp();
  const { status, options, api } = c;
  const [override, setOverride] = useState(false);
  const [reason, setReason] = useState("");
  const [needsOverride, setNeedsOverride] = useState<string | null>(null);

  const [gapProfile, setGapProfile] = useState("lane-2026-10-25");
  const [gapText, setGapText] = useState("");
  const [gapMinText, setGapMinText] = useState("");
  const [parkProfile, setParkProfile] = useState("floor-2026-10-07");
  const [raceline, setRaceline] = useState("");
  const [mlRaceline, setMlRaceline] = useState("");
  const [policyName, setPolicyName] = useState("");
  const [pnpBackend, setPnpBackend] = useState("geometric");
  const [pnpDynamics, setPnpDynamics] = useState("");
  const [pnpPerception, setPnpPerception] = useState("");
  const [pnpParams, setPnpParams] = useState("");
  const [pnpSpeed, setPnpSpeed] = useState(0.5);
  const [patrol, setPatrol] = useState("");
  const [rule, setRule] = useState("arms_raised");
  const [patrolSpeed, setPatrolSpeed] = useState(0.5);

  if (!status || !options) {
    return (
      <div class="grid" aria-busy="true">
        {Array.from({ length: 6 }, (_, i) => <SkeletonCard key={i} lines={2} />)}
      </div>
    );
  }

  const blocked = driveBlocked(status, c.ops, c.agentUp);
  const external = externalDriveText(status.drive.external);
  const stack = stackState(status, c.ops);

  /** Start a drive: the card shows "starting" at once, the override panel opens on a refusal. */
  const drive = async (label: string, body: DriveBody) => {
    if (override && !reason.trim()) throw new ApiError("Give a reason to override", 400, null);
    try {
      const r = await c.runOp({ kind: "drive-start", title: `Starting ${label}`, target: body.mode }, () =>
        api.startDrive({ ...body, override, override_reason: override ? reason : "" }),
      );
      setNeedsOverride(null);
      return r;
    } catch (e) {
      const err = toApiError(e);
      setNeedsOverride(err.body?.overridable ? err.message : null);
      throw e;
    }
  };

  const policy = options.policy ?? status.policy;
  const pnp = options.pnp ?? null;
  const pnpRange = pnp?.speed_range ?? [0.2, 1.5];
  const pnpNeedsPerception = pnpBackend === "net";
  const dynChoice = selectedBundle(pnpDynamics, pnp?.dynamics);
  const percChoice = pnpNeedsPerception ? selectedBundle(pnpPerception, pnp?.perception) : "";
  const paramsChoice = selectedParamsFile(pnpParams, pnp?.params_files);
  const gapBounds = options.gap_follow_speed?.[gapProfile];
  const gapCap = gapSpeed(gapText, gapBounds);
  const gapFloor = gapMinSpeed(gapMinText, gapBounds, gapCap.value);
  const gapMinCeiling = gapCap.value ?? gapBounds?.max;
  const spiderbot = status.spiderbot ?? options.spiderbot ?? null;
  const speedRange = options.spiderbot_speed_range ?? [0.1, 1.0];

  const pnpWhy = !pnp?.available
    ? `Unavailable: ${pnp?.reason ?? "unknown"}`
    : !dynChoice
      ? "Choose a dynamics bundle"
      : pnpNeedsPerception && !percChoice
        ? "Choose a perception bundle"
        : pnpSpeed < pnpRange[0] || pnpSpeed > pnpRange[1]
          ? `Speed cap must be ${pnpRange[0]} to ${pnpRange[1]} m/s`
          : "";

  const needsRaceline = (r: string) => (r ? "" : "Choose a raceline");

  return (
    <div class="stack-col">
      <StackStrip />
      {external && (
        <div class="warnline" role="alert">
          <Icon name="warn" size={16} />
          <span>{external}</span>
        </div>
      )}
      {needsOverride && (
        <div class="override" role="alert">
          <div><b>Checks failed:</b> {needsOverride}</div>
          <label class="check">
            <input type="checkbox" checked={override} onChange={(e) => setOverride(e.currentTarget.checked)} />
            Override the checks and drive anyway (logged)
          </label>
          {override && (
            <input placeholder="Reason (required)" value={reason} onInput={(e) => setReason(e.currentTarget.value)} aria-label="Override reason" />
          )}
          <div class="sub">The stack, recorder and disk checks can never be overridden.</div>
        </div>
      )}

      <div class="grid">
        <ModeCard
          mode="gap_follow"
          title="Gap follow"
          info="Follows the widest gap the LiDAR sees. Max speed empty means the profile's own speed; a number must sit inside the profile's bounds and the global speed cap. Min speed empty means the profile's own floor (0.36); a number must be 0 or more and cannot exceed the max speed in effect."
          blocked={blocked}
          extra={gapCap.error || gapFloor.error}
          onStart={() =>
            drive("gap follow", {
              mode: "gap_follow",
              profile: gapProfile,
              max_speed_mps: gapCap.value,
              min_speed_mps: gapFloor.value,
            })
          }
        >
          <div class="controls">
            <Field label="Profile">
              <select value={gapProfile} onChange={(e) => setGapProfile(e.currentTarget.value)} aria-label="Gap follow profile">
                {options.gap_follow_profiles.map((x) => <option key={x}>{x}</option>)}
              </select>
            </Field>
            <Field label="Max speed (m/s)">
              <input
                type="number"
                aria-label="Gap follow max speed in metres per second"
                placeholder={gapBounds ? `profile, up to ${gapBounds.max}` : "profile's own"}
                min={gapBounds?.min}
                max={gapBounds?.max}
                step={0.05}
                value={gapText}
                onInput={(e) => setGapText(e.currentTarget.value)}
              />
            </Field>
            <Field label="Min speed (m/s)">
              <input
                type="number"
                aria-label="Gap follow min speed in metres per second"
                placeholder={gapMinCeiling !== undefined ? `profile 0.36, up to ${gapMinCeiling}` : "profile's own"}
                min={0}
                max={gapMinCeiling}
                step={0.05}
                value={gapMinText}
                onInput={(e) => setGapMinText(e.currentTarget.value)}
              />
            </Field>
          </div>
        </ModeCard>

        <ModeCard
          mode="pnp"
          title="Plug-and-Play"
          info={`Drives a track it has never seen: no map, no pose. Needs only the stack and its recorder. The nodes refuse any bundle that does not match this car. Start at a low cap, wheels off the ground first, hand on the knob. ${pnp?.available ? pnpSelectionText({ backend: pnpBackend, dynamics: dynChoice, perception: percChoice, paramsFile: paramsChoice, speed: pnpSpeed }) : ""}`}
          blocked={blocked}
          extra={pnpWhy}
          onStart={() =>
            drive("Plug-and-Play", {
              mode: "pnp",
              pnp_backend: pnpBackend,
              pnp_dynamics: dynChoice,
              pnp_perception: percChoice,
              pnp_params: paramsChoice,
              pnp_v_max_mps: pnpSpeed,
            })
          }
        >
          <div class="controls">
            <Field label="Perception">
              <select value={pnpBackend} onChange={(e) => setPnpBackend(e.currentTarget.value)} aria-label="Plug-and-Play perception backend" disabled={!pnp?.available}>
                {(pnp?.backends ?? ["geometric", "net"]).map((x) => <option key={x}>{x}</option>)}
              </select>
            </Field>
            <Field label="Dynamics">
              <select value={dynChoice} onChange={(e) => setPnpDynamics(e.currentTarget.value)} aria-label="Plug-and-Play dynamics bundle" disabled={!pnp?.available}>
                {!dynChoice && <option value="">choose</option>}
                {pnp?.dynamics.map((x) => <option key={x} value={x}>{x}</option>)}
              </select>
            </Field>
            {pnpNeedsPerception && (
              <Field label="Net bundle">
                <select value={percChoice} onChange={(e) => setPnpPerception(e.currentTarget.value)} aria-label="Plug-and-Play perception bundle" disabled={!pnp?.available}>
                  {!percChoice && <option value="">choose</option>}
                  {pnp?.perception.map((x) => <option key={x} value={x}>{x}</option>)}
                </select>
              </Field>
            )}
            <Field label="Planner file">
              <select value={paramsChoice} onChange={(e) => setPnpParams(e.currentTarget.value)} aria-label="Plug-and-Play planner params file" disabled={!pnp?.available}>
                <option value="">default</option>
                {(pnp?.params_files ?? []).map((x) => <option key={x} value={x}>{x}</option>)}
              </select>
            </Field>
          </div>
          <Field label="Speed cap" value={`${pnpSpeed.toFixed(2)} m/s`}>
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
          </Field>
        </ModeCard>

        <ModeCard
          mode="tracker"
          title="Tracker"
          info="Follows a raceline. Needs the stack with localisation, a live pose, and a raceline written in that map's frame."
          blocked={blocked}
          extra={needsRaceline(raceline)}
          onStart={() => drive("tracker", { mode: "tracker", raceline })}
        >
          <Field label="Raceline">
            <select value={raceline} onChange={(e) => setRaceline(e.currentTarget.value)} aria-label="Raceline">
              <option value="">choose</option>
              {options.racelines.map((x) => <option key={x}>{x}</option>)}
            </select>
          </Field>
        </ModeCard>

        <ParkCard
          blocked={blocked}
          profile={parkProfile}
          setProfile={setParkProfile}
          options={options}
          onStart={() => drive("park", { mode: "park", profile: parkProfile })}
          command={async (name) => {
            try {
              const r = await c.act(() => api.park(name, override, override ? reason : ""));
              setNeedsOverride(null);
              return r;
            } catch (e) {
              const err = toApiError(e);
              setNeedsOverride(err.body?.overridable ? err.message : null);
              throw e;
            }
          }}
          abort={() => c.act(() => api.park("abort"))}
        />

        <ModeCard
          mode="wall_follow"
          title="Wall follow"
          info="Keeps a fixed distance to the wall on one side."
          blocked={blocked}
          onStart={() => drive("wall follow", { mode: "wall_follow" })}
        />

        <ModeCard
          mode="ml_policy"
          title="ML policy"
          info="Tracker plus a bounded learned residual. Same localisation needs as the tracker. The policy node refuses to start on any mismatch with this car's parameters. First trials: wheels off the ground, hand on the knob."
          blocked={blocked}
          unavailable={policy?.available ? "" : (policy?.reason ?? "No usable policy")}
          extra={needsRaceline(mlRaceline) || ((policy?.valid.length ?? 0) > 1 && !policyName ? "Choose a policy" : "")}
          onStart={() => drive("ML policy", { mode: "ml_policy", raceline: mlRaceline, policy: policyName })}
        >
          <div class="controls">
            <Field label="Policy">
              <select value={policyName} onChange={(e) => setPolicyName(e.currentTarget.value)} aria-label="ML policy" disabled={!policy?.available}>
                <option value="">{policy?.available ? (policy.valid.length > 1 ? "choose" : "default") : "none usable"}</option>
                {policy?.valid.map((x) => <option key={x}>{x}</option>)}
              </select>
            </Field>
            <Field label="Raceline">
              <select value={mlRaceline} onChange={(e) => setMlRaceline(e.currentTarget.value)} aria-label="ML policy raceline" disabled={!policy?.available}>
                <option value="">choose</option>
                {options.racelines.map((x) => <option key={x}>{x}</option>)}
              </select>
            </Field>
          </div>
        </ModeCard>

        <ModeCard
          mode="spiderbot"
          title="Spiderbot"
          info="Patrols the loop on the map. Stops, turns to face a person who holds the pose, raises an alert, then resumes. Needs localisation and perception running. Goes through the safety node like every other drive."
          blocked={blocked}
          unavailable={spiderbot?.available ? "" : (spiderbot?.reason ?? "Unavailable")}
          extra={patrol ? "" : "Choose a patrol"}
          onStart={() => drive("spiderbot", { mode: "spiderbot", patrol, rule, speed_mps: patrolSpeed })}
        >
          <div class="controls">
            <Field label="Patrol">
              <select value={patrol} onChange={(e) => setPatrol(e.currentTarget.value)} aria-label="Patrol" disabled={!spiderbot?.available}>
                <option value="">choose</option>
                {(options.patrols ?? []).map((x) => <option key={x}>{x}</option>)}
              </select>
            </Field>
            <Field label="Pose">
              <select value={rule} onChange={(e) => setRule(e.currentTarget.value)} aria-label="Pose rule" disabled={!spiderbot?.available}>
                {(options.spiderbot_rules ?? ["arms_raised"]).map((x) => <option key={x}>{x}</option>)}
              </select>
            </Field>
            <Field label="Speed (m/s)">
              <input
                type="number"
                aria-label="Patrol speed in metres per second"
                min={speedRange[0]}
                max={speedRange[1]}
                step={0.1}
                value={patrolSpeed}
                disabled={!spiderbot?.available}
                onInput={(e) => setPatrolSpeed(Number(e.currentTarget.value))}
              />
            </Field>
          </div>
        </ModeCard>
      </div>
      {stack === "stopping" && <div class="sub">The stack is closing its bag. Drives are unavailable until it is done.</div>}
    </div>
  );
}

function Field({ label, value, children }: { label: string; value?: string; children: ComponentChildren }) {
  return (
    <label class="field">
      <span>
        {label}
        {value && <span class="val num">{value}</span>}
      </span>
      {children}
    </label>
  );
}

/** One row above the cards: is the stack up, and a one-click start when it is not. */
function StackStrip() {
  const c = useApp();
  const st = stackState(c.status, c.ops);
  const flags = c.status?.stack.flags ?? {};
  const mode = flags.mapping === "true" ? "mapping" : flags.localization === "true" ? "localisation" : "teleop";
  const parts = [flags.lidar !== "false" ? "LiDAR" : "", flags.vesc !== "false" ? "VESC" : "", mode].filter(Boolean);
  const busy = st === "starting" || st === "stopping" || st === "restarting";
  return (
    <div class="strip">
      <div class="grow">
        <b>Stack</b>
        {st === "on" && <Pill view={{ level: "green", text: "Running" }} />}
        {st === "off" && <Pill view={{ level: "red", text: "Off" }} />}
        {busy && <Pill view={{ level: "busy", text: st === "starting" ? "Starting" : st === "restarting" ? "Restarting" : "Stopping" }} />}
        {st === "on" && <span class="muted">{parts.join(", ")}</span>}
        {st === "off" && <span class="muted">Drives need the stack and its recorder.</span>}
      </div>
      {st === "off" && (
        <ActionButton
          label="Start stack"
          pendingLabel="Starting..."
          variant="primary"
          size="sm"
          disabled={!c.agentUp}
          title={c.agentUp ? "LiDAR and VESC, teleop only (more options in System)" : "Car offline"}
          onAction={() => c.runOp({ kind: "stack-start" }, () => c.api.startStack({ lidar: true, vesc: true, mapping: false, localization: false, map: "" }))}
        />
      )}
      <button class="btn ghost sm" onClick={() => c.setTab("system")}>System</button>
    </div>
  );
}

function ModeCard(p: {
  mode: string;
  title: string;
  info: string;
  blocked: string;
  /** A reason from this card's own inputs (nothing chosen yet). */
  extra?: string;
  /** The feature is not available on this car at all. */
  unavailable?: string;
  onStart: () => Promise<unknown>;
  children?: ComponentChildren;
  /** Extra buttons shown while this mode runs. */
  runningControls?: ComponentChildren;
}) {
  const c = useApp();
  const d = driveState(c.status, c.ops);
  const mine = modeMatches(d.mode, p.mode);
  const running = mine && d.state === "running";
  const starting = mine && d.state === "starting";
  const stopping = mine && d.state === "stopping";
  const why = p.unavailable || p.blocked || p.extra || "";
  const state = running ? "running" : starting ? "starting" : stopping ? "stopping" : p.unavailable || p.blocked ? "blocked" : "idle";
  const badge =
    state === "running" ? "Running" : state === "starting" ? "Starting..." : state === "stopping" ? "Stopping..." : p.unavailable ? "Unavailable" : "Idle";
  return (
    <Card
      title={p.title}
      info={p.info}
      class={`state-${state}`}
      badge={<StateBadge kind={state}>{badge}</StateBadge>}
      label={p.title}
    >
      {p.children}
      {running && p.runningControls}
      {running || stopping ? (
        <ActionButton
          label="Stop"
          pendingLabel="Stopping..."
          variant="danger"
          size="lg"
          block
          disabled={stopping}
          onAction={() => c.stopAll()}
          ariaLabel={`Stop ${p.title}`}
        />
      ) : (
        <ActionButton
          label="Start"
          pendingLabel="Starting..."
          successLabel="Started"
          variant="primary"
          size="lg"
          block
          disabled={starting || !!why}
          title={why || undefined}
          onAction={p.onStart}
          ariaLabel={`Start ${p.title}`}
        />
      )}
      <div class="why" aria-live="polite">
        {!running && !starting && !stopping && why}
        {starting && "Waiting for the car to report it running"}
      </div>
    </Card>
  );
}

function ParkCard(p: {
  blocked: string;
  profile: string;
  setProfile: (v: string) => void;
  options: Options;
  onStart: () => Promise<unknown>;
  command: (name: string) => Promise<unknown>;
  abort: () => Promise<unknown>;
}) {
  return (
    <ModeCard
      mode="park"
      title="Park"
      info="Launches the parking behaviour, then Park runs the manoeuvre and Three-point turn turns around. Abort stops the manoeuvre."
      blocked={p.blocked}
      onStart={p.onStart}
      runningControls={
        <div class="inner-row">
          <ActionButton label="Park" pendingLabel="Sending..." size="sm" variant="primary" onAction={() => p.command("start")} />
          <ActionButton label="3-point turn" pendingLabel="Sending..." size="sm" onAction={() => p.command("three_point_turn")} />
          <ActionButton label="Abort" pendingLabel="Sending..." size="sm" variant="danger" onAction={p.abort} />
        </div>
      }
    >
      <Field label="Profile">
        <select value={p.profile} onChange={(e) => p.setProfile(e.currentTarget.value)} aria-label="Park profile">
          {p.options.park_profiles.map((x) => <option key={x}>{x}</option>)}
        </select>
      </Field>
    </ModeCard>
  );
}
