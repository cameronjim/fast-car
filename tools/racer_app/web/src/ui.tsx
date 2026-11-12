// Shared building blocks: icons, pills, cards, skeletons, the action button with its four states,
// the optimistic toggle, the step timeline and the toast stack.

import type { ComponentChildren } from "preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { type Outcome, type ToastStore } from "./feedback";
import { useRunner, useStore } from "./ctx";
import type { Op, Step } from "./ops";
import { elapsedLabel, type PillView } from "./view";

// -- icons -------------------------------------------------------------------------------------

const PATHS = {
  check: "M4 12.5l5 5L20 6.5",
  x: "M6 6l12 12M18 6L6 18",
  info: "M12 11v6M12 7.5v.5",
  warn: "M12 3.5l9.5 16.5h-19zM12 10v4.5M12 17.4v.2",
  chevron: "M6 9l6 6 6-6",
  alert: "M12 8v5M12 16.5v.5",
  bolt: "M13 3L5 14h6l-1 7 8-11h-6z",
} as const;
export type IconName = keyof typeof PATHS;

export function Icon({ name, size }: { name: IconName; size?: number }) {
  return (
    <svg
      viewBox="0 0 24 24"
      width={size}
      height={size}
      fill="none"
      stroke="currentColor"
      stroke-width="2.4"
      stroke-linecap="round"
      stroke-linejoin="round"
      aria-hidden="true"
    >
      <path d={PATHS[name]} />
    </svg>
  );
}

export function Spinner({ label }: { label?: string }) {
  return <span class="spinner" role={label ? "status" : undefined} aria-label={label} />;
}

export function Skeleton({ w, h = 14, class: cls }: { w?: string | number; h?: number; class?: string }) {
  return <span class={`skeleton ${cls ?? ""}`} style={{ width: w ?? "100%", height: h }} aria-hidden="true" />;
}

/** A card-shaped placeholder shown on first load, before the first status reply. */
export function SkeletonCard({ lines = 3 }: { lines?: number }) {
  return (
    <div class="card" aria-busy="true" aria-label="Loading">
      <Skeleton w="45%" h={18} />
      {Array.from({ length: lines }, (_, i) => (
        <Skeleton key={i} w={`${90 - i * 14}%`} />
      ))}
      <Skeleton h={40} />
    </div>
  );
}

export function Pill({ view, onClick, class: cls }: { view: PillView; onClick?: () => void; class?: string }) {
  const lvl = view.level === "busy" ? "busy" : view.level === "neutral" ? "" : `lvl-${view.level}`;
  const c = `pill ${lvl} ${cls ?? ""}`;
  return onClick ? (
    <button class={c} title={view.title} onClick={onClick}>
      {view.level === "busy" && <Spinner />}
      {view.text}
    </button>
  ) : (
    <span class={c} title={view.title}>
      {view.level === "busy" && <Spinner />}
      {view.text}
    </span>
  );
}

// -- cards -------------------------------------------------------------------------------------

/** A card with an optional state badge and an info button that opens the long explanation. */
export function Card(p: {
  title: string;
  badge?: ComponentChildren;
  info?: string;
  tools?: ComponentChildren;
  class?: string;
  label?: string;
  children?: ComponentChildren;
}) {
  const [open, setOpen] = useState(false);
  return (
    <section class={`card ${p.class ?? ""}`} aria-label={p.label ?? p.title}>
      <header>
        <h2>{p.title}</h2>
        {p.badge}
        {(p.tools || p.info) && (
          <span class="tools">
            {p.tools}
            {p.info && (
              <button class="infobtn" aria-expanded={open} aria-label={`About ${p.title}`} onClick={() => setOpen(!open)}>
                <Icon name="info" />
              </button>
            )}
          </span>
        )}
      </header>
      {open && p.info && <div class="hint" role="note">{p.info}</div>}
      {p.children}
    </section>
  );
}

export function StateBadge({ kind, children }: { kind: "idle" | "running" | "starting" | "stopping" | "blocked"; children: ComponentChildren }) {
  return (
    <span class={`state-badge ${kind}`}>
      {(kind === "starting" || kind === "stopping") && <Spinner />}
      {children}
    </span>
  );
}

// -- the action button ---------------------------------------------------------------------------

export interface ActionButtonProps {
  label: string;
  /** Shown with the spinner while the request is out, for example "Starting...". */
  pendingLabel?: string;
  successLabel?: string;
  onAction: () => Promise<unknown>;
  disabled?: boolean;
  /** Tooltip, and the reason a disabled button is disabled. */
  title?: string;
  variant?: "primary" | "danger" | "default" | "ghost";
  size?: "sm" | "md" | "lg";
  block?: boolean;
  /** Two-click confirm: the first click arms the button and shows this text for 4 s. */
  confirm?: string;
  class?: string;
  ariaLabel?: string;
}

/**
 * idle -> pending (spinner, disabled, "Starting...") -> success (check, 1.5 s) or error (the reason
 * inline, under the button). A click is never silent, and a double click cannot send twice.
 */
export function ActionButton(p: ActionButtonProps) {
  const runner = useRunner();
  const [armed, setArmed] = useState(false);
  const armTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => () => void (armTimer.current && clearTimeout(armTimer.current)), []);
  const s = runner.state;
  const pending = s.phase === "pending";

  const fire = () => {
    void runner.run(async () => (await p.onAction()) as { message?: string } | void);
  };
  const click = () => {
    if (p.confirm && !armed) {
      setArmed(true);
      armTimer.current = setTimeout(() => setArmed(false), 4000);
      return;
    }
    if (armTimer.current) clearTimeout(armTimer.current);
    setArmed(false);
    fire();
  };

  const variant = p.variant && p.variant !== "default" ? p.variant : "";
  const cls = [
    "btn",
    variant,
    p.size ?? "",
    p.block ? "block" : "",
    pending ? "is-pending" : "",
    s.phase === "success" ? "is-success" : "",
    s.phase === "error" ? "is-error" : "",
    armed ? "armed" : "",
    p.class ?? "",
  ].join(" ");

  let content: ComponentChildren = p.label;
  if (pending) content = (<><Spinner />{p.pendingLabel ?? "Working..."}</>);
  else if (s.phase === "success") content = (<><Icon name="check" />{p.successLabel ?? "Done"}</>);
  else if (armed) content = p.confirm;

  return (
    <div class="action">
      <button
        class={cls}
        disabled={p.disabled || pending}
        aria-busy={pending}
        aria-label={p.ariaLabel}
        title={p.title}
        onClick={click}
      >
        {content}
      </button>
      {s.phase === "error" && (
        <div class="inline-error" role="alert">
          <Icon name="warn" />
          <span>{s.error}</span>
        </div>
      )}
    </div>
  );
}

/** A switch that flips at once, shows a pending state, and flips back with the reason on error. */
export function Toggle(p: {
  label: string;
  checked: boolean;
  onChange: (next: boolean) => Promise<unknown>;
  disabled?: boolean;
}) {
  const runner = useRunner({ errorMs: 6000 });
  const [optimistic, setOptimistic] = useState<boolean | null>(null);
  const s = runner.state;
  const shown = optimistic ?? p.checked;
  const change = (next: boolean) => {
    setOptimistic(next);
    void runner.run(async () => (await p.onChange(next)) as { message?: string } | void).then((o: Outcome | null) => {
      if (!o || !o.ok) setOptimistic(null); // revert: the real value stays what the status says
    });
  };
  // Once the poll agrees with the optimistic value, stop overriding it.
  useEffect(() => {
    if (optimistic !== null && p.checked === optimistic && s.phase !== "pending") setOptimistic(null);
  }, [p.checked, optimistic, s.phase]);
  return (
    <div class="action">
      <label class={`switch ${s.phase === "pending" ? "pending" : ""} ${p.disabled ? "disabled" : ""}`}>
        <input
          type="checkbox"
          checked={shown}
          disabled={p.disabled || s.phase === "pending"}
          onChange={(e) => change(e.currentTarget.checked)}
        />
        <span class="track" />
        <span>{p.label}</span>
        {s.phase === "pending" && <Spinner />}
        {s.phase === "success" && <span class="lvl-green"><Icon name="check" size={16} /></span>}
      </label>
      {s.phase === "error" && (
        <div class="inline-error" role="alert">
          <Icon name="warn" />
          <span>{s.error}</span>
        </div>
      )}
    </div>
  );
}

// -- timeline ------------------------------------------------------------------------------------

export function StepList({ steps }: { steps: Step[] }) {
  return (
    <ol class="steps" aria-label="Progress">
      {steps.map((s) => (
        <li key={s.id} class={s.state}>
          <span class="ic">{s.state === "done" ? <Icon name="check" /> : s.state === "failed" ? <Icon name="x" /> : null}</span>
          {s.label}
        </li>
      ))}
    </ol>
  );
}

export function OpRow({ op, steps, now, onDismiss }: { op: Op; steps: Step[]; now: number; onDismiss: () => void }) {
  const active = op.phase === "requesting" || op.phase === "waiting";
  const end = op.finishedAt ?? now;
  const failed = op.phase === "failed" || op.phase === "timeout";
  return (
    <div class={`op ${op.phase}`} role="status" aria-live="polite">
      <header>
        {active && <Spinner />}
        {op.phase === "confirmed" && <span class="lvl-green"><Icon name="check" size={16} /></span>}
        {failed && <span class="lvl-red"><Icon name="warn" size={16} /></span>}
        <b>
          {op.title}
          {op.phase === "confirmed" ? ": done" : op.phase === "timeout" ? ": timed out" : op.phase === "failed" ? ": failed" : "..."}
        </b>
        <span class="elapsed num">{elapsedLabel(end - op.startedAt)}</span>
        {!active && (
          <button class="btn ghost sm" onClick={onDismiss} aria-label="Dismiss">
            <Icon name="x" />
          </button>
        )}
      </header>
      <StepList steps={steps} />
      {op.error && <div class="reason">{op.error}</div>}
    </div>
  );
}

// -- toasts --------------------------------------------------------------------------------------

export function Toasts({ store }: { store: ToastStore }) {
  useStore((cb) => store.subscribe(cb));
  const list = store.all();
  if (list.length === 0) return null;
  return (
    <div class="toasts" aria-live="polite">
      {list.map((t) => (
        <div key={t.id} class={`toast ${t.kind}`} role={t.kind === "error" ? "alert" : "status"}>
          <span class="msg">{t.text}</span>
          <button aria-label="Dismiss" onClick={() => store.dismiss(t.id)}>x</button>
        </div>
      ))}
    </div>
  );
}
