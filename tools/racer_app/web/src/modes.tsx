// Starting drives: the override flow shared by every place a drive can start, and the cards for
// the less common modes (park, wall follow, ML policy, spiderbot) that live on the System tab.
// Safety gating, the override flow and every API call are the ones the agent already enforces.

import type { ComponentChildren } from "preact";
import { useState } from "preact/hooks";
import { ApiError, type Options } from "./api";
import { useApp } from "./ctx";
import { toApiError } from "./feedback";
import { ActionButton, Card, SkeletonCard, StateBadge } from "./ui";
import { driveBlocked, driveState, modeMatches } from "./view";

export type DriveBody = Parameters<ReturnType<typeof useApp>["api"]["startDrive"]>[0];

/**
 * `drive(label, body)` starts a drive as an operation (shown as starting at once, confirmed by the
 * status). A refusal the agent marks overridable opens `panel`: a checkbox and a reason, logged.
 */
export function useDriveStarter() {
  const c = useApp();
  const [override, setOverride] = useState(false);
  const [reason, setReason] = useState("");
  const [needsOverride, setNeedsOverride] = useState<string | null>(null);

  const guard = async <T,>(fn: (o: { override: boolean; override_reason: string }) => Promise<T>): Promise<T> => {
    if (override && !reason.trim()) throw new ApiError("Give a reason to override", 400, null);
    try {
      const r = await fn({ override, override_reason: override ? reason : "" });
      setNeedsOverride(null);
      setOverride(false);
      return r;
    } catch (e) {
      const err = toApiError(e);
      setNeedsOverride(err.body?.overridable ? err.message : null);
      throw e;
    }
  };

  const drive = (label: string, body: DriveBody) =>
    guard((o) => c.runOp({ kind: "drive-start", title: `Starting ${label}`, target: body.mode }, () => c.api.startDrive({ ...body, ...o })));

  const panel = needsOverride ? (
    <div class="override" role="alert">
      <div class="override-head">
        <b>Checks failed</b>
        <span class="muted">{needsOverride}</span>
      </div>
      <label class="check">
        <input type="checkbox" checked={override} onChange={(e) => setOverride(e.currentTarget.checked)} />
        Override (logged)
      </label>
      {override && (
        <input placeholder="Reason" value={reason} onInput={(e) => setReason(e.currentTarget.value)} aria-label="Override reason" />
      )}
    </div>
  ) : null;

  return { drive, guard, panel };
}

export function Field({ label, value, children }: { label: string; value?: string; children: ComponentChildren }) {
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

export function ModeCard(p: {
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
    <Card title={p.title} info={p.info} class={`state-${state}`} badge={<StateBadge kind={state}>{badge}</StateBadge>} label={p.title}>
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
          onErrorBadge={() => c.setTab("logs")}
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
          onErrorBadge={() => c.setTab("logs")}
        />
      )}
      <div class="why" aria-live="polite">
        {!running && !starting && !stopping && why}
        {starting && "Waiting for the car"}
      </div>
    </Card>
  );
}

/** Park, wall follow, the ML policy and the spiderbot: the System tab's "More drives". */
export function OtherDrives() {
  const c = useApp();
  const { status, options, api } = c;
  const { drive, guard, panel } = useDriveStarter();
  const [parkProfile, setParkProfile] = useState("floor-2026-10-07");
  const [mlRaceline, setMlRaceline] = useState("");
  const [policyName, setPolicyName] = useState("");
  const [patrol, setPatrol] = useState("");
  const [rule, setRule] = useState("arms_raised");
  const [patrolSpeed, setPatrolSpeed] = useState(0.5);

  if (!status || !options) return <SkeletonCard lines={2} />;

  const blocked = driveBlocked(status, c.ops, c.agentUp);
  const policy = options.policy ?? status.policy;
  const spiderbot = status.spiderbot ?? options.spiderbot ?? null;
  const speedRange = options.spiderbot_speed_range ?? [0.1, 1.0];

  return (
    <>
      {panel && <div class="span-all">{panel}</div>}
      <ParkCard
        blocked={blocked}
        profile={parkProfile}
        setProfile={setParkProfile}
        options={options}
        onStart={() => drive("park", { mode: "park", profile: parkProfile })}
        command={(name) => guard((o) => c.act(() => api.park(name, o.override, o.override_reason)))}
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
        extra={mlRaceline ? ((policy?.valid.length ?? 0) > 1 && !policyName ? "Choose a policy" : "") : "Choose a raceline"}
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
    </>
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
