// The Tune tab: the values the Drive buttons use, per mode, with the defaults and a reset. Saved
// on the car (GET/POST /api/prefs) a moment after each change, so every phone and laptop sees the
// same settings. Values that are out of bounds are marked and not saved.

import { useEffect, useRef, useState } from "preact/hooks";
import type { Prefs } from "./api";
import { useApp } from "./ctx";
import { errorText } from "./feedback";
import { Field } from "./modes";
import { A_LAT_RANGE, GAP_NODE_DEFAULTS, gapErrors, parseNum, patch, pnpErrors, prefsProblems, samePrefs } from "./prefs";
import { Card, ErrorBadge, Icon, Pill, SkeletonCard } from "./ui";

type SaveState = { kind: "saved" } | { kind: "saving" } | { kind: "invalid"; why: string } | { kind: "error"; why: string };

export function TuneTab() {
  const c = useApp();
  const { options } = c;
  const [draft, setDraft] = useState<Prefs>(c.prefs);
  const [save, setSave] = useState<SaveState>({ kind: "saved" });
  const dirty = useRef(false);

  // Follow the car's copy until the owner edits here (another client may have saved).
  useEffect(() => {
    if (!dirty.current) setDraft(c.prefs);
  }, [c.prefs]);

  useEffect(() => {
    if (!dirty.current || samePrefs(draft, c.prefs)) {
      if (dirty.current) dirty.current = false;
      setSave({ kind: "saved" });
      return;
    }
    const problems = prefsProblems(draft, options);
    if (problems.length) {
      setSave({ kind: "invalid", why: problems.join("; ") });
      return;
    }
    setSave({ kind: "saving" });
    const timer = setTimeout(() => {
      c.savePrefs(draft).then(
        () => {
          dirty.current = false;
          setSave({ kind: "saved" });
        },
        (e: unknown) => setSave({ kind: "error", why: errorText(e) }),
      );
    }, 700);
    return () => clearTimeout(timer);
  }, [draft]);

  if (!c.status || !options || !c.prefsLoaded) {
    return (
      <div class="grid">
        {Array.from({ length: 4 }, (_, i) => <SkeletonCard key={i} lines={3} />)}
      </div>
    );
  }

  const edit = (next: Prefs) => {
    dirty.current = true;
    setDraft(next);
  };
  const d = c.prefsDefaults;
  const g = draft.gap_follow;
  const bounds = options.gap_follow_speed?.[g.profile];
  const gErr = gapErrors(g, bounds);
  const maxInEffect = g.max_speed_mps !== null && !gErr.max_speed_mps ? g.max_speed_mps : (bounds?.max ?? 1.5);
  const n = draft.pnp;
  const pnp = options.pnp;
  const pRange = pnp?.speed_range ?? [0.2, 1.5];
  const pErr = pnpErrors(n, options);
  const setGap = (v: Partial<Prefs["gap_follow"]>) => edit(patch(draft, "gap_follow", v));
  const setPnp = (v: Partial<Prefs["pnp"]>) => edit(patch(draft, "pnp", v));

  const reset = (key: "gap_follow" | "pnp" | "tracker" | "stack") => (
    <button class="btn ghost sm" onClick={() => edit(patch(draft, key, d[key]))} disabled={samePrefs({ ...draft, [key]: d[key] }, draft)}>
      <Icon name="reset" />
      Reset
    </button>
  );

  return (
    <div class="stack-col">
      <div class="pagehead">
        <SaveBadge state={save} onLogs={() => c.setTab("logs")} />
      </div>
      <div class="grid">
        <Card title="Gap follow" icon="route" tools={reset("gap_follow")}>
          <Field label="Profile">
            <select value={g.profile} onChange={(e) => setGap({ profile: e.currentTarget.value })} aria-label="Gap follow profile">
              {options.gap_follow_profiles.map((x) => <option key={x}>{x}</option>)}
            </select>
          </Field>
          <NumRow
            label="Max speed"
            unit="m/s"
            value={g.max_speed_mps}
            min={bounds?.min ?? 0}
            max={bounds?.max ?? 1.5}
            step={0.05}
            fallback={bounds?.max ?? 1}
            error={gErr.max_speed_mps}
            onChange={(v) => setGap({ max_speed_mps: v })}
          />
          <NumRow
            label="Min speed"
            unit="m/s"
            value={g.min_speed_mps}
            min={0}
            max={maxInEffect}
            step={0.05}
            fallback={GAP_NODE_DEFAULTS.min_speed_mps}
            error={gErr.min_speed_mps}
            onChange={(v) => setGap({ min_speed_mps: v })}
          />
          <NumRow
            label="Corner grip (a_lat_max)"
            unit="m/s²"
            value={g.a_lat_max_mps2}
            min={A_LAT_RANGE[0]}
            max={6}
            inputMax={A_LAT_RANGE[1]}
            step={0.1}
            fallback={GAP_NODE_DEFAULTS.a_lat_max_mps2}
            fallbackUnknown
            error={gErr.a_lat_max_mps2}
            onChange={(v) => setGap({ a_lat_max_mps2: v })}
          />
          <NumRow
            label="Wall speed (side_cap_min)"
            unit="m/s"
            value={g.side_cap_min_speed_mps}
            min={0}
            max={maxInEffect}
            step={0.05}
            fallback={GAP_NODE_DEFAULTS.side_cap_min_speed_mps}
            fallbackUnknown
            error={gErr.side_cap_min_speed_mps}
            onChange={(v) => setGap({ side_cap_min_speed_mps: v })}
          />
        </Card>

        <Card title="ML driver" icon="chip" tools={reset("pnp")}>
          {!pnp?.available && <div class="why">Unavailable: {pnp?.reason ?? "loading"}</div>}
          <NumRow
            label="Speed cap"
            unit="m/s"
            value={n.v_max_mps}
            min={pRange[0]}
            max={pRange[1]}
            step={0.05}
            fallback={d.pnp.v_max_mps}
            error={pErr.v_max_mps}
            nullable={false}
            onChange={(v) => setPnp({ v_max_mps: v ?? d.pnp.v_max_mps })}
          />
          <div class="field">
            <span>Perception</span>
            <div class="seg" role="group" aria-label="ML driver perception backend">
              {(pnp?.backends ?? ["geometric", "net"]).map((b) => (
                <button key={b} aria-pressed={n.backend === b} onClick={() => setPnp({ backend: b })}>
                  {b}
                </button>
              ))}
            </div>
          </div>
          <Field label="Dynamics">
            <select value={n.dynamics} onChange={(e) => setPnp({ dynamics: e.currentTarget.value })} aria-label="ML driver dynamics bundle">
              <option value="">{pnp?.dynamics.length === 1 ? `auto (${pnp.dynamics[0]})` : "choose"}</option>
              {pnp?.dynamics.map((x) => <option key={x} value={x}>{x}</option>)}
            </select>
          </Field>
          {n.backend === "net" && (
            <Field label="Net bundle">
              <select value={n.perception} onChange={(e) => setPnp({ perception: e.currentTarget.value })} aria-label="ML driver perception bundle">
                <option value="">{pnp?.perception.length === 1 ? `auto (${pnp.perception[0]})` : "choose"}</option>
                {pnp?.perception.map((x) => <option key={x} value={x}>{x}</option>)}
              </select>
            </Field>
          )}
          <Field label="Planner file">
            <select value={n.params_file} onChange={(e) => setPnp({ params_file: e.currentTarget.value })} aria-label="ML driver planner file">
              <option value="">default</option>
              {(pnp?.params_files ?? []).map((x) => <option key={x} value={x}>{x}</option>)}
            </select>
          </Field>
        </Card>

        <Card title="Racer" icon="flag" tools={reset("tracker")}>
          <Field label="Raceline">
            <select value={draft.tracker.raceline} onChange={(e) => edit(patch(draft, "tracker", { raceline: e.currentTarget.value }))} aria-label="Raceline">
              <option value="">choose</option>
              {options.racelines.map((x) => <option key={x}>{x}</option>)}
            </select>
          </Field>
          <div class="sub">Needs Start mode: Localisation.</div>
        </Card>

        <Card title="Start car" icon="power" tools={reset("stack")}>
          <div class="field">
            <span>Start mode</span>
            <div class="seg" role="group" aria-label="Start mode">
              {(
                [
                  ["teleop", "Plain"],
                  ["mapping", "Mapping"],
                  ["localization", "Localisation"],
                ] as const
              ).map(([m, label]) => (
                <button key={m} aria-pressed={draft.stack.mode === m} onClick={() => edit(patch(draft, "stack", { mode: m, map: m === "localization" ? draft.stack.map : "" }))}>
                  {label}
                </button>
              ))}
            </div>
          </div>
          {draft.stack.mode === "localization" && (
            <Field label="Map">
              <select value={draft.stack.map} onChange={(e) => edit(patch(draft, "stack", { map: e.currentTarget.value }))} aria-label="Map">
                <option value="">choose</option>
                {options.maps.map((m) => <option key={m} value={m}>{m}</option>)}
              </select>
            </Field>
          )}
          <div class="sub">LiDAR and VESC. Applies at the next Start car.</div>
        </Card>
      </div>
    </div>
  );
}

function SaveBadge({ state, onLogs }: { state: SaveState; onLogs: () => void }) {
  if (state.kind === "saved") return <Pill view={{ level: "green", text: "Saved on the car" }} />;
  if (state.kind === "saving") return <Pill view={{ level: "busy", text: "Saving" }} />;
  if (state.kind === "invalid") return <Pill view={{ level: "amber", text: "Not saved: fix the red values", title: state.why }} />;
  return <ErrorBadge title={state.why} onClick={onLogs} />;
}

/** A slider and a number box for one value. Empty box (or reset) means the profile's own value. */
function NumRow(p: {
  label: string;
  unit: string;
  value: number | null;
  min: number;
  max: number;
  /** The number box may go above the slider's range (rarely needed values). */
  inputMax?: number;
  step: number;
  /** Where the slider rests while the value is the profile's own. */
  fallback: number;
  error?: string;
  nullable?: boolean;
  /** The profile may set its own value, so the resting slider position is not the real value. */
  fallbackUnknown?: boolean;
  onChange: (v: number | null) => void;
}) {
  const nullable = p.nullable ?? true;
  const show = (v: number | null) => (v === null ? "" : String(Number(v.toFixed(3))));
  const [text, setText] = useState(show(p.value));
  useEffect(() => {
    const parsed = parseNum(text);
    if (parsed !== p.value && !(Number.isNaN(parsed) && p.value === null)) setText(show(p.value));
  }, [p.value]);
  const bad = !!p.error || Number.isNaN(parseNum(text));
  const isDefault = p.value === null;
  return (
    <div class={`numrow ${bad ? "bad" : ""}`}>
      <div class="numrow-head">
        <span>{p.label}</span>
        <span class={`numrow-val num ${isDefault ? "muted" : ""}`}>{isDefault ? (p.fallbackUnknown ? "profile" : `profile (${p.fallback})`) : `${show(p.value)} ${p.unit}`}</span>
      </div>
      <div class="numrow-ctl">
        <input
          type="range"
          class={isDefault ? "dim" : ""}
          min={p.min}
          max={p.max}
          step={p.step}
          value={Math.min(p.max, Math.max(p.min, p.value ?? p.fallback))}
          aria-label={`${p.label} slider`}
          onInput={(e) => {
            const v = Number(e.currentTarget.value);
            setText(show(v));
            p.onChange(v);
          }}
        />
        <input
          type="number"
          inputMode="decimal"
          class="numbox"
          min={p.min}
          max={p.inputMax ?? p.max}
          step={p.step}
          value={text}
          placeholder={nullable ? "profile" : String(p.fallback)}
          aria-label={`${p.label} in ${p.unit}`}
          aria-invalid={bad}
          onInput={(e) => {
            const t = e.currentTarget.value;
            setText(t);
            const v = parseNum(t);
            if (v === null) p.onChange(nullable ? null : p.fallback);
            else if (!Number.isNaN(v)) p.onChange(v);
          }}
        />
        <button
          class="iconbtn"
          title={nullable ? "Back to the profile's value" : "Back to the default"}
          aria-label={`Reset ${p.label}`}
          disabled={nullable ? isDefault : p.value === p.fallback}
          onClick={() => {
            setText(nullable ? "" : show(p.fallback));
            p.onChange(nullable ? null : p.fallback);
          }}
        >
          <Icon name="reset" size={15} />
        </button>
      </div>
      {bad && <span class="field-err">{p.error ?? "not a number"}</span>}
    </div>
  );
}
