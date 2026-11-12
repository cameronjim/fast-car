// The persistent frame: top bar with the status pills, tabs, the reconnecting banner, the
// operations tray and the STOP ALL bar that is on screen on every tab.

import { useRunner, useApp, useStore, TABS, type TabId } from "./ctx";
import { lastSeenText } from "./feedback";
import { opSteps } from "./ops";
import { Icon, OpRow, Pill, Skeleton, Spinner } from "./ui";
import {
  batteryPill,
  connectionState,
  freshness,
  readyPill,
  recordingPill,
  type PillView,
} from "./view";
import type { Live } from "./live";

export function TopBar({ live, onTab }: { live: Live; onTab: (t: TabId) => void }) {
  const c = useApp();
  const conn = connectionState(c.agentUp, c.link.connected, c.status?.stack.running ?? false);
  const connPill: PillView =
    conn === "connected"
      ? { level: "green", text: "Connected" }
      : conn === "reconnecting"
        ? { level: "amber", text: "Reconnecting" }
        : { level: "amber", text: "Live data down", title: "The agent answers, the ROS link does not" };
  const loading = !c.status;
  const volts = freshness(live.voltage, c.now);
  return (
    <header class="topbar">
      <div class="brand">
        <span class="logo" aria-hidden="true"><Icon name="bolt" size={15} /></span>
        <span class="name">{c.carName}</span>
        <Pill view={connPill} />
      </div>
      <div class="pills" aria-label="Car status">
        {loading ? (
          <>
            <Skeleton w={84} h={24} />
            <Skeleton w={110} h={24} />
            <Skeleton w={72} h={24} />
          </>
        ) : (
          <>
            <Pill view={readyPill(c.status)} onClick={() => onTab("checks")} />
            <Pill class={c.bags.some((b) => b.active) ? "rec" : ""} view={recordingPill(c.bags, c.status)} onClick={() => onTab("logs")} />
            <Pill view={batteryPill(volts, c.status?.thresholds.battery_cells ?? 3)} onClick={() => onTab("live")} />
          </>
        )}
      </div>
    </header>
  );
}

export function Tabs({ tab, onTab }: { tab: TabId; onTab: (t: TabId) => void }) {
  const c = useApp();
  const failing = c.status?.checks.checks.filter((x) => x.required && x.level === "red").length ?? 0;
  return (
    <nav class="tabs" role="tablist" aria-label="Sections">
      {TABS.map((t) => (
        <button key={t.id} class="tab" role="tab" aria-selected={tab === t.id} onClick={() => onTab(t.id)}>
          {t.label}
          {t.id === "checks" && failing > 0 && <span class="badge" aria-label={`${failing} failing`}>{failing}</span>}
        </button>
      ))}
    </nav>
  );
}

/** Non-blocking: the page stays usable (and dimmed) while the agent or the ROS link is away. */
export function ReconnectBanner() {
  const c = useApp();
  if (c.agentUp && (c.link.connected || !c.status?.stack.running)) return null;
  const agentDown = !c.agentUp;
  return (
    <div class="recon" role="status" aria-live="polite">
      <Spinner />
      <span>{agentDown ? "Reconnecting to the car..." : "Live data link is down, retrying..."}</span>
      <span class="muted">
        {agentDown ? lastSeenText(c.agentLastSeen, c.now) : lastSeenText(c.linkLastSeen, c.now)}
        {agentDown ? ". Values below are the last ones received." : ""}
      </span>
    </div>
  );
}

export function OpsTray() {
  const c = useApp();
  useStore((cb) => c.ops.subscribe(cb));
  const list = c.ops.all();
  if (list.length === 0) return null;
  const snap = { status: c.status, linkConnected: c.link.connected, bags: c.bags };
  return (
    <div class="ops" aria-label="In progress">
      {list.map((op) => (
        <OpRow key={op.id} op={op} steps={opSteps(op, snap)} now={c.now} onDismiss={() => c.ops.dismiss(op.id)} />
      ))}
    </div>
  );
}

/**
 * STOP ALL on every screen, including the login screen (it needs no token on purpose). It is a
 * software stop and says so in one line; the RC kill knob is the stop that always works.
 */
export function StopBar({ onStop }: { onStop: () => Promise<unknown> }) {
  const runner = useRunner({ reentrant: true, successMs: 2500, errorMs: 0 });
  const s = runner.state;
  const press = () => void runner.run(async () => (await onStop()) as { message?: string } | void);
  return (
    <div class="stopbar" role="region" aria-label="Emergency stop">
      <div class="note">
        <div>
          <b>Software stop only.</b> Wi-Fi can drop. Keep a hand on the RC kill knob.
        </div>
        <div class={`stopstate ${s.phase === "error" ? "lvl-red" : "lvl-green"}`} role="status">
          {s.phase === "pending" && "Sending stop..."}
          {s.phase === "success" && "Stop sent. Check the car has stopped."}
          {s.phase === "error" && `Stop may not have arrived: ${s.error}. Use the knob.`}
        </div>
      </div>
      <button
        class={`stopbtn ${s.phase === "pending" ? "is-pending" : ""} ${s.phase === "success" ? "is-success" : ""}`}
        onClick={press}
        aria-label="Stop all driving"
      >
        {s.phase === "pending" ? (
          <>
            <Spinner />
            STOPPING
          </>
        ) : s.phase === "success" ? (
          <>
            <Icon name="check" size={26} />
            STOP SENT
          </>
        ) : (
          "STOP ALL"
        )}
      </button>
    </div>
  );
}
