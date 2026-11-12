// The Logs tab: bags with sizes and the two-click delete, and the action log as a timeline.

import { useApp } from "./ctx";
import { fmtBagSize, fmtGrowth } from "./model";
import { ActionButton, Card, Skeleton } from "./ui";

export function LogsTab() {
  const c = useApp();
  const loading = !c.status;
  const maxMb = Math.max(1, ...c.bags.map((b) => b.size_mb));
  return (
    <div class="grid wide">
      <Card
        title="Bags"
        label="Bags"
        info="Every run is recorded. Delete asks twice and cannot be undone. The bag being recorded cannot be deleted."
        badge={<span class="muted">{c.bags.length} folders</span>}
      >
        {loading && Array.from({ length: 3 }, (_, i) => <Skeleton key={i} h={38} />)}
        {!loading && c.bags.length === 0 && <div class="empty">No bags yet.</div>}
        <div>
          {c.bags.map((b) => (
            <div class="bag" key={b.name}>
              <div>
                <div class="nm">
                  {b.name}
                  {b.active && <span class="pill lvl-red rec">REC</span>}
                </div>
                <div class="meta num">
                  {fmtBagSize(b.size_mb)}
                  {b.active ? `, ${fmtGrowth(b.growth_mb_s)}` : ""}
                </div>
              </div>
              <ActionButton
                label="Delete"
                pendingLabel="Deleting..."
                confirm="Delete for good?"
                variant="danger"
                size="sm"
                disabled={b.active || !c.agentUp}
                title={b.active ? "Being recorded" : undefined}
                onAction={() => c.act(() => c.api.deleteBag(b.name))}
              />
              <div class="bar"><div class="fill" style={{ width: `${Math.max(2, (b.size_mb / maxMb) * 100)}%` }} /></div>
            </div>
          ))}
        </div>
      </Card>

      <Card title="Action log" label="Action log" info="Everything the app asked the car to do, newest first. Red means it failed. Amber means an override was used.">
        {loading && Array.from({ length: 4 }, (_, i) => <Skeleton key={i} h={34} />)}
        {!loading && c.log.length === 0 && <div class="empty">Nothing yet.</div>}
        <ul class="timeline">
          {c.log.map((e, i) => (
            <li key={`${e.ts}-${i}`} class={e.ok ? (e.override ? "ovr" : "") : "bad"}>
              <span class="when">{new Date(e.ts * 1000).toLocaleTimeString()}</span>
              <span class="what">
                {e.action}
                {e.override && <span class="tag">OVERRIDE</span>}
                {!e.ok && <span class="tag" style={{ background: "var(--red)", color: "#fff" }}>FAILED</span>}
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
