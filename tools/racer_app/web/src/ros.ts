// A READ-ONLY foxglove ws-protocol client for the car's foxglove_bridge (port 8765).
//
// This file only ever subscribes. It never advertises a client channel, never sends a message,
// never calls a service and never sets a parameter, so the app cannot publish /drive_raw or
// /drive over this socket (CLAUDE.md invariant 1). guard.test.ts fails the build if any other
// source file reaches for those calls.
//
// Cameras are duty-cycled: subscribe, take ONE frame, unsubscribe, wait, subscribe again. The
// bridge has no per-subscription rate limit, and a 720p JPEG stream at 60 fps is several MB/s
// over Wi-Fi, so a thumbnail that only wants one or two frames a second must not hold a
// subscription open.

import { FoxgloveClient } from "@foxglove/ws-protocol";
import type { Channel, IWebSocket, MessageData } from "@foxglove/ws-protocol";
import { parse } from "@foxglove/rosmsg";
import { MessageReader } from "@foxglove/rosmsg2-serialization";

export type SocketFactory = (url: string, protocols: string[]) => IWebSocket;
export type Unsubscribe = () => void;

export interface SubscribeOptions {
  /** Duty-cycle: take one message, drop the subscription, resubscribe after this many ms. */
  oneShotEveryMs?: number;
}

interface Want {
  topic: string;
  handler: (msg: unknown) => void;
  opts: SubscribeOptions;
  subId: number | null;
  resubscribeTimer: ReturnType<typeof setTimeout> | null;
}

export interface LinkStatus {
  connected: boolean;
  topics: string[];
}

export class RosLink {
  private client: FoxgloveClient | null = null;
  private channels = new Map<number, Channel>();
  private readers = new Map<number, MessageReader>();
  private wants = new Set<Want>();
  private subToWant = new Map<number, Want>();
  private statusListeners = new Set<(s: LinkStatus) => void>();
  private retryTimer: ReturnType<typeof setTimeout> | null = null;
  private closed = false;
  private connected = false;

  constructor(
    private url: string,
    private makeSocket: SocketFactory = (u, p) => new WebSocket(u, p) as unknown as IWebSocket,
    private retryMs = 2000,
  ) {}

  onStatus(cb: (s: LinkStatus) => void): Unsubscribe {
    this.statusListeners.add(cb);
    cb(this.status());
    return () => this.statusListeners.delete(cb);
  }

  status(): LinkStatus {
    return {
      connected: this.connected,
      topics: [...this.channels.values()].map((c) => c.topic).sort(),
    };
  }

  private emit(): void {
    const s = this.status();
    this.statusListeners.forEach((cb) => cb(s));
  }

  connect(): void {
    this.closed = false;
    this.open();
  }

  close(): void {
    this.closed = true;
    if (this.retryTimer) clearTimeout(this.retryTimer);
    this.wants.forEach((w) => w.resubscribeTimer && clearTimeout(w.resubscribeTimer));
    this.client?.close();
    this.client = null;
  }

  private open(): void {
    const ws = this.makeSocket(this.url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]);
    const client = new FoxgloveClient({ ws });
    this.client = client;
    client.on("open", () => {
      this.connected = true;
      this.emit();
    });
    client.on("advertise", (channels) => {
      for (const ch of channels) this.channels.set(ch.id, ch);
      this.emit();
      this.wants.forEach((w) => this.trySubscribe(w));
    });
    client.on("unadvertise", (ids) => {
      for (const id of ids) {
        this.channels.delete(id);
        this.readers.delete(id);
        for (const [sub, w] of this.subToWant) {
          if (w.subId === sub && !this.findChannel(w.topic)) {
            this.subToWant.delete(sub);
            w.subId = null;
          }
        }
      }
      this.emit();
    });
    client.on("message", (ev) => this.onMessage(ev));
    const down = () => {
      if (this.client !== client) return;
      this.connected = false;
      this.channels.clear();
      this.readers.clear();
      this.subToWant.clear();
      this.wants.forEach((w) => (w.subId = null));
      this.client = null;
      this.emit();
      if (!this.closed) this.retryTimer = setTimeout(() => this.open(), this.retryMs);
    };
    client.on("close", down);
    client.on("error", () => undefined); // a close always follows; handled there
  }

  subscribe<T = unknown>(
    topic: string,
    handler: (msg: T) => void,
    opts: SubscribeOptions = {},
  ): Unsubscribe {
    const want: Want = {
      topic,
      handler: handler as (msg: unknown) => void,
      opts,
      subId: null,
      resubscribeTimer: null,
    };
    this.wants.add(want);
    this.trySubscribe(want);
    return () => {
      this.wants.delete(want);
      if (want.resubscribeTimer) clearTimeout(want.resubscribeTimer);
      this.dropSubscription(want);
    };
  }

  private findChannel(topic: string): Channel | undefined {
    for (const ch of this.channels.values()) if (ch.topic === topic) return ch;
    return undefined;
  }

  private trySubscribe(want: Want): void {
    if (!this.client || want.subId !== null || !this.wants.has(want)) return;
    const channel = this.findChannel(want.topic);
    if (!channel) return;
    const subId = this.client.subscribe(channel.id);
    want.subId = subId;
    this.subToWant.set(subId, want);
  }

  private dropSubscription(want: Want): void {
    if (want.subId === null) return;
    this.subToWant.delete(want.subId);
    this.client?.unsubscribe(want.subId);
    want.subId = null;
  }

  private readerFor(channel: Channel): MessageReader | null {
    let reader = this.readers.get(channel.id);
    if (reader) return reader;
    if (channel.encoding !== "cdr") return null;
    try {
      reader = new MessageReader(parse(channel.schema, { ros2: true }));
    } catch {
      return null;
    }
    this.readers.set(channel.id, reader);
    return reader;
  }

  private onMessage(ev: MessageData): void {
    const want = this.subToWant.get(ev.subscriptionId);
    if (!want) return;
    const channel = this.findChannel(want.topic);
    if (!channel) return;
    const reader = this.readerFor(channel);
    if (!reader) return;
    let msg: unknown;
    try {
      msg = reader.readMessage(ev.data);
    } catch {
      return;
    }
    want.handler(msg);
    if (want.opts.oneShotEveryMs) {
      this.dropSubscription(want);
      want.resubscribeTimer = setTimeout(() => this.trySubscribe(want), want.opts.oneShotEveryMs);
    }
  }
}
