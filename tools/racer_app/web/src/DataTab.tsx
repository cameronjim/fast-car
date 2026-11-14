// The Data tab: how full the disk is, every recorded run with its date and size, delete (asks
// twice, never the run being recorded), the command that copies a run to this computer, and a
// docker cleanup behind a confirm.

import { useState } from "preact/hooks";
import { useApp } from "./ctx";
import { copyCommand, diskBar, fmtGb, fmtRunDate } from "./data";
import { fmtBagSize, fmtGrowth } from "./model";
import { ActionButton, Card, Icon, Skeleton } from "./ui";

export function DataTab() {
  const c = useApp();
  const info = c.bagsInfo;
  const loading = info === null;
  const bar = diskBar(info, c.status?.thresholds.disk_red_gb ?? 5);
  const [copied, setCopied] = useState<string | null>(null);

  const copy = async (name: string, text: string) => {
    try {
      await navigator.clipboard.writeText(text);
      setCopied(name);
      setTimeout(() => setCopied((n) => (n === name ? null : n)), 2500);
    } catch {
      // No clipboard (plain http on a phone): show the command so it can be copied by hand.
      c.setLastOutput({ title: `Copy ${name}`, text });
      c.setTab("logs");
    }
  };

  return (
    <div class="stack-col">
      <Card
        title="Disk"
        icon="disk"
        tools={
          <ActionButton
            label="Clean up"
            icon="trash"
            pendingLabel="Cleaning..."
            successLabel="Cleaned"
            size="sm"
            confirm="Prune docker?"
            title="docker image prune -f and docker builder prune -f. Runs are not touched."
            disabled={!c.agentUp}
            onErrorBadge={() => c.setTab("logs")}
            onAction={async () => {
              try {
                const r = await c.act(() => c.api.cleanup(), "Clean up");
                c.setLastOutput({ title: "Clean up", text: String(r.output ?? r.message ?? "") });
                return r;
              } catch (e) {
                const out = (e as { body?: { output?: string } }).body?.output;
                if (out) c.setLastOutput({ title: "Clean up (failed)", text: out });
                throw e;
              }
            }}
          />
        }
      >
        {loading ? (
          <Skeleton h={28} />
        ) : (
          <>
            <div class="diskline">
              <span class={`big num lvl-${bar.level}`}>
                {fmtGb(bar.freeGb)}
                <small>free</small>
              </span>
              <span class="muted num">of {fmtGb(bar.totalGb)}</span>
            </div>
            <div class="diskbar" role="img" aria-label={`Disk: ${fmtGb(bar.bagsGb)} of runs, ${fmtGb(bar.freeGb)} free`}>
              <span class="seg-bags" style={{ width: `${bar.bagsPct}%` }} />
              <span class="seg-other" style={{ width: `${bar.otherPct}%` }} />
            </div>
            <div class="legend">
              <span><i class="sw bags" />Runs {fmtGb(bar.bagsGb)}</span>
              <span><i class="sw other" />Other</span>
              <span><i class="sw free" />Free</span>
            </div>
          </>
        )}
      </Card>

      <Card title="Runs" icon="list" label="Runs" badge={<span class="muted">{info ? info.bags.length : ""}</span>}>
        {loading && Array.from({ length: 3 }, (_, i) => <Skeleton key={i} h={44} />)}
        {!loading && info.bags.length === 0 && <div class="empty">No runs yet</div>}
        <ul class="runs">
          {(info?.bags ?? []).map((b) => {
            const cmd = copyCommand(info?.user, c.carHost, info?.bag_dir, b.name);
            return (
              <li key={b.name} class={`run ${b.active ? "active" : ""}`}>
                <div class="run-main">
                  <div class="run-date">
                    {fmtRunDate(b.mtime)}
                    {b.active && <span class="pill lvl-red rec">REC</span>}
                  </div>
                  <div class="run-name">{b.name}</div>
                </div>
                <div class="run-size num">
                  {fmtBagSize(b.size_mb)}
                  {b.active && <small>{fmtGrowth(b.growth_mb_s)}</small>}
                </div>
                <div class="run-act">
                  <button
                    class="iconbtn"
                    disabled={!cmd}
                    title={cmd ? `Copy to your computer: ${cmd}` : "No copy command for this name"}
                    aria-label={`Copy command for ${b.name}`}
                    onClick={() => cmd && void copy(b.name, cmd)}
                  >
                    <Icon name={copied === b.name ? "check" : "copy"} size={16} />
                  </button>
                  <ActionButton
                    label=""
                    icon="trash"
                    ariaLabel={`Delete ${b.name}`}
                    pendingLabel=""
                    successLabel=""
                    confirm="Delete?"
                    variant="danger"
                    size="sm"
                    class="iconish"
                    disabled={b.active || !c.agentUp}
                    title={b.active ? "Being recorded" : "Delete for good"}
                    onErrorBadge={() => c.setTab("logs")}
                    onAction={() => c.act(() => c.api.deleteBag(b.name), `Delete ${b.name}`)}
                  />
                </div>
              </li>
            );
          })}
        </ul>
        {copied && <div class="sub" role="status">Copied. Paste it in Terminal on your computer.</div>}
      </Card>
    </div>
  );
}
