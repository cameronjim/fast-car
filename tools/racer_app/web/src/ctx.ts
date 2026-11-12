import { createContext } from "preact";
import { useContext, useEffect, useRef, useState } from "preact/hooks";
import type { ActionReply, Api, BagInfo, LogEntry, Options, Status } from "./api";
import { ActionRunner, type RunnerOptions } from "./feedback";
import type { ToastStore } from "./feedback";
import type { BeginSpec, OpsStore } from "./ops";
import type { LinkStatus } from "./ros";

export type TabId = "drive" | "live" | "checks" | "logs" | "system";
export const TABS: { id: TabId; label: string }[] = [
  { id: "drive", label: "Drive" },
  { id: "live", label: "Live" },
  { id: "checks", label: "Checks" },
  { id: "logs", label: "Logs" },
  { id: "system", label: "System" },
];

export interface Ctx {
  api: Api;
  status: Status | null;
  options: Options | null;
  bags: BagInfo[];
  log: LogEntry[];
  agentUp: boolean;
  /** Date.now() of the last good status reply. */
  agentLastSeen: number | null;
  link: LinkStatus;
  linkLastSeen: number | null;
  ops: OpsStore;
  toasts: ToastStore;
  /** Date.now() at this render (the app redraws five times a second). */
  now: number;
  refresh: () => void;
  /** Run a request, toast its outcome, refresh, and rethrow so the button shows the error. */
  act: (fn: () => Promise<ActionReply>) => Promise<ActionReply>;
  /** Like act, but also shows the target state at once with a timeline (see ops.ts). */
  runOp: (spec: BeginSpec, fn: () => Promise<ActionReply>) => Promise<ActionReply>;
  stopAll: () => Promise<ActionReply>;
  setTab: (t: TabId) => void;
  carName: string;
  /** Where the token is kept, and a warning when it is not the keychain (desktop only). */
  tokenWhere: string;
  tokenNote: string | null;
}

export const AppCtx = createContext<Ctx>(null as unknown as Ctx);
export const useApp = (): Ctx => useContext(AppCtx);

/** Re-render when any of the stores' subscribe() fires. */
export function useStore(sub: (cb: () => void) => () => void): void {
  const [, set] = useState(0);
  useEffect(() => sub(() => set((n) => n + 1)), [sub]);
}

/** One ActionRunner per button, re-rendering the button on every phase change. */
export function useRunner(opts?: RunnerOptions): ActionRunner {
  const ref = useRef<ActionRunner | null>(null);
  if (!ref.current) ref.current = new ActionRunner(opts);
  const r = ref.current;
  const [, set] = useState(0);
  useEffect(() => {
    const off = r.subscribe(() => set((n) => n + 1));
    return () => {
      off();
      r.dispose();
    };
  }, [r]);
  return r;
}
