// The Checks tab: the pre-drive checklist as compact rows (dot, short label, one word). The
// long detail from the agent is one click away.

import { useState } from "preact/hooks";
import type { Check } from "./api";
import { useApp } from "./ctx";
import { fmt, fmtDuration } from "./model";
import { Card, Icon, Pill, Skeleton } from "./ui";
import { readyPill } from "./view";

const WORD: Record<Check["level"], string> = { green: "OK", amber: "Warn", red: "Fail" };

export function ChecksTab() {
  const c = useApp();
  const status = c.status;
  const [open, setOpen] = useState<string | null>(null);
  if (!status) {
    return (
      <Card title="Pre-drive checks" label="Pre-drive checks">
        {Array.from({ length: 7 }, (_, i) => <Skeleton key={i} h={34} />)}
      </Card>
    );
  }
  const checks = status.checks.checks;
  const ok = checks.filter((x) => x.level === "green").length;
  return (
    <Card
      title="Pre-drive checks"
      label="Pre-drive checks"
      info="Every required check must be green before a drive starts. Optional checks only warn. The stack, recorder and disk checks cannot be overridden."
      badge={<Pill view={readyPill(status)} />}
    >
      <div class="summary">
        <span class="big num">{ok}<small>of {checks.length} OK</small></span>
      </div>
      <ul class="rows">
        {checks.map((x) => {
          const isOpen = open === x.id;
          return (
            <li key={x.id} class="row-item">
              <button class="row-btn" aria-expanded={isOpen} onClick={() => setOpen(isOpen ? null : x.id)}>
                <span class={`dot lg lvl-${x.level}`} aria-hidden="true" />
                <span class="label">
                  {x.label}
                  {!x.required && <small>optional</small>}
                </span>
                <span class={`word lvl-${x.level}`}>{WORD[x.level]}</span>
                <span class="chev"><Icon name="chevron" /></span>
              </button>
              {isOpen && <div class="row-detail">{x.detail}</div>}
            </li>
          );
        })}
      </ul>
      <div class="sub num">
        Disk {fmt(status.disk_free_gb, 1, " GiB")} free. Up {fmtDuration(status.uptime_s)}.
      </div>
    </Card>
  );
}
