// The persistent frame: the top bar (car, connection, recording, battery, the problems badge and
// the red STOP), the sidebar, the reconnecting banner and the operations tray. STOP is on screen
// on every tab, and on the login screen too (it needs no token on purpose).

import { useRunner, useApp, useStore, TABS, type TabId } from "./ctx";
import { lastSeenText } from "./feedback";
import { opSteps } from "./ops";
import { badgeCount, liveProblems } from "./problems";
import { ErrorBadge, Icon, OpRow, Pill, Skeleton, Spinner } from "./ui";
import { batteryPill, connectionState, freshness, recordingPill, type PillView } from "./view";
import type { Live } from "./live";

export function TopBar({ live, onTab, tab }: { live: Live; onTab: (t: TabId) => void; tab: TabId }) {
  const c = useApp();
  useStore((cb) => c.problems.subscribe(cb));
  const conn = connectionState(c.agentUp, c.link.connected, c.status?.stack.running ?? false);
  const connPill: PillView =
    conn === "connected"
      ? { level: "green", text: "Connected" }
      : conn === "reconnecting"
        ? { level: "amber", text: "Reconnecting" }
        : { level: "amber", text: "No live data", title: "The agent answers, the ROS link does not" };
  const loading = !c.status;
  const volts = freshness(live.voltage, c.now);
  const count = badgeCount(liveProblems(c.status, c.now), tab === "logs" ? [] : c.problems.unseen());
  return (
    <header class="topbar">
      <div class="brand">
        <span class="logo" aria-hidden="true"><Icon name="bolt" size={16} /></span>
        <span class="name">{c.carName}</span>
      </div>
      <div class="pills" aria-label="Car status">
        <Pill view={connPill} />
        {loading ? (
          <Skeleton w={110} h={24} />
        ) : (
          <>
            <Pill class={c.bags.some((b) => b.active) ? "rec" : ""} view={recordingPill(c.bags, c.status)} onClick={() => onTab("data")} />
            <Pill class="hide-sm" view={batteryPill(volts, c.status?.thresholds.battery_cells ?? 3)} onClick={() => onTab("live")} />
          </>
        )}
        {count > 0 && <ErrorBadge count={count} title={`${count} problem${count > 1 ? "s" : ""}`} onClick={() => onTab("logs")} />}
      </div>
      <StopButton onStop={c.stopAll} />
    </header>
  );
}

export function Sidebar({ tab, onTab }: { tab: TabId; onTab: (t: TabId) => void }) {
  return (
    <nav class="sidebar" aria-label="Sections">
      <ul role="tablist">
        {TABS.map((t) => (
          <li key={t.id} class={t.id === "system" ? "bottom" : ""}>
            <button class="navitem" role="tab" aria-selected={tab === t.id} onClick={() => onTab(t.id)}>
              <Icon name={t.icon} size={20} />
              <span>{t.label}</span>
            </button>
          </li>
        ))}
      </ul>
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
      <span>{agentDown ? "Reconnecting" : "Live data down"}</span>
      <span class="muted">{agentDown ? lastSeenText(c.agentLastSeen, c.now) : lastSeenText(c.linkLastSeen, c.now)}</span>
    </div>
  );
}

/** In-flight operations. Off the Logs tab a failure is a red badge; Logs shows the reason. */
export function OpsTray({ tab }: { tab: TabId }) {
  const c = useApp();
  useStore((cb) => c.ops.subscribe(cb));
  const list = c.ops.all();
  if (list.length === 0) return null;
  const snap = { status: c.status, linkConnected: c.link.connected, bags: c.bags };
  return (
    <div class="ops" aria-label="In progress">
      {list.map((op) => (
        <OpRow
          key={op.id}
          op={op}
          steps={opSteps(op, snap)}
          now={c.now}
          onDismiss={() => c.ops.dismiss(op.id)}
          onLogs={tab === "logs" ? undefined : () => c.setTab("logs")}
        />
      ))}
    </div>
  );
}

/**
 * The red STOP in the top bar. A software stop: it SIGINTs every driving launch (STOP ALL). Wi-Fi
 * can drop, so the RC kill knob is the stop that always works; the button's tooltip says so.
 */
export function StopButton({ onStop }: { onStop: () => Promise<unknown> }) {
  const runner = useRunner({ reentrant: true, successMs: 2500, errorMs: 8000 });
  const s = runner.state;
  const press = () => void runner.run(async () => (await onStop()) as { message?: string } | void);
  const title =
    s.phase === "error"
      ? `Stop may not have arrived: ${s.error}. Use the RC kill knob.`
      : "Stop all driving. Software stop: keep a hand on the RC kill knob.";
  return (
    <button
      class={`stopbtn ${s.phase === "pending" ? "is-pending" : ""} ${s.phase === "success" ? "is-success" : ""} ${s.phase === "error" ? "is-error" : ""}`}
      onClick={press}
      title={title}
      aria-label="Stop all driving"
    >
      {s.phase === "pending" ? <Spinner /> : s.phase === "success" ? <Icon name="check" size={18} /> : s.phase === "error" ? <Icon name="warn" size={18} /> : <Icon name="stop" size={18} />}
      <span>{s.phase === "success" ? "SENT" : s.phase === "error" ? "RETRY" : "STOP"}</span>
    </button>
  );
}

/** The login screen's stop bar: same STOP, with the one-line warning spelled out. */
export function StopBar({ onStop }: { onStop: () => Promise<unknown> }) {
  return (
    <div class="stopbar" role="region" aria-label="Emergency stop">
      <div class="note">
        <b>Software stop only.</b> Keep a hand on the RC kill knob.
      </div>
      <StopButton onStop={onStop} />
    </div>
  );
}
