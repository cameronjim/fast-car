// The Logs tab: where the detail lives. Problems (every red badge leads here), the pre-drive
// checks, the action log, the driving launch's own output and the last raw command output.

import { useEffect, useState } from "preact/hooks";
import { ChecksTab } from "./ChecksTab";
import { useApp, useStore } from "./ctx";
import { errorText } from "./feedback";
import { failedActions, liveProblems, type Problem } from "./problems";
import { ActionButton, Card, Skeleton } from "./ui";

const OPENED_S = Date.now() / 1000;

export function LogsTab() {
  const c = useApp();
  useStore((cb) => c.problems.subscribe(cb));
  const loading = !c.status;

  // Opening Logs counts as having seen the problems: the badge clears, the list stays.
  useEffect(() => {
    c.problems.markSeen();
  }, []);

  const live = liveProblems(c.status, c.now);
  const mine = c.problems.all();
  const fromAgent = failedActions(c.log, OPENED_S - 3600).filter(
    (a) => !mine.some((m) => Math.abs(m.at - a.at) < 3000 && m.detail === a.detail),
  );
  const list: Problem[] = [...live, ...mine, ...fromAgent];

  return (
    <div class="grid wide">
      <Card
        title="Problems"
        icon="warn"
        label="Problems"
        badge={<span class={list.length ? "lvl-red" : "muted"}>{list.length || "none"}</span>}
        tools={
          mine.length > 0 && (
            <button class="btn ghost sm" onClick={() => c.problems.clear()}>
              Clear
            </button>
          )
        }
      >
        {list.length === 0 && <div class="empty">Nothing wrong</div>}
        <ul class="timeline">
          {list.map((p) => (
            <li key={p.key} class="bad">
              <span class="when">{p.kind === "live" ? "now" : new Date(p.at).toLocaleTimeString()}</span>
              <span class="what">{p.title}</span>
              {p.detail && <span class="detail">{p.detail}</span>}
            </li>
          ))}
        </ul>
      </Card>

      <ChecksTab />

      {c.lastOutput && (
        <Card
          title={c.lastOutput.title}
          icon="list"
          tools={
            <button class="btn ghost sm" onClick={() => c.setLastOutput(null)}>
              Close
            </button>
          }
        >
          <pre class="raw">{c.lastOutput.text || "(no output)"}</pre>
        </Card>
      )}

      <DriveOutput />

      <Card title="Action log" icon="list" label="Action log" info="Everything the app asked the car to do, newest first. Red: failed. Amber: an override was used.">
        {loading && Array.from({ length: 4 }, (_, i) => <Skeleton key={i} h={34} />)}
        {!loading && c.log.length === 0 && <div class="empty">Nothing yet</div>}
        <ul class="timeline">
          {c.log.map((e, i) => (
            <li key={`${e.ts}-${i}`} class={e.ok ? (e.override ? "ovr" : "") : "bad"}>
              <span class="when">{new Date(e.ts * 1000).toLocaleTimeString()}</span>
              <span class="what">
                {e.action}
                {e.override && <span class="tag">OVERRIDE</span>}
                {!e.ok && <span class="tag bad">FAILED</span>}
              </span>
              {e.detail && <span class="detail">{e.detail}</span>}
              {e.override_reason && <span class="detail">Reason: {e.override_reason}</span>}
            </li>
          ))}
        </ul>
      </Card>
    </div>
  );
}

/** The last lines the driving launch printed (its log inside the stack container). */
function DriveOutput() {
  const c = useApp();
  const [text, setText] = useState<string | null>(null);
  const load = async () => {
    try {
      setText((await c.api.driveLog()).log);
    } catch (e) {
      setText(`(could not read it: ${errorText(e)})`);
    }
  };
  useEffect(() => {
    void load();
  }, [c.status?.drive.mode ?? ""]);
  return (
    <Card
      title="Drive output"
      icon="list"
      info="The last lines the driving launch printed, read from the stack container."
      tools={<ActionButton label="Refresh" pendingLabel="Reading..." successLabel="Updated" size="sm" variant="ghost" onAction={load} />}
    >
      {text === null ? <Skeleton h={60} /> : <pre class="raw">{text.trim() || "(empty)"}</pre>}
    </Card>
  );
}
