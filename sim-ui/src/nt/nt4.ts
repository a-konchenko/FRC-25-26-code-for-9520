// Minimal NetworkTables 4 client (browser + Node 22+, uses global WebSocket).
//
// Speaks the NT4 protocol the robot code's NT server uses (same as Elastic /
// AdvantageScope): JSON text frames for publish/subscribe/announce, MessagePack
// binary frames for values: [topicId, timestampMicros, typeIndex, value].
// Spec: https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc

import { decodeMulti, encode } from '@msgpack/msgpack';

export type NTType =
  | 'boolean' | 'double' | 'int' | 'float' | 'string' | 'json' | 'raw'
  | 'boolean[]' | 'double[]' | 'int[]' | 'float[]' | 'string[]';

const TYPE_INDEX: Record<string, number> = {
  boolean: 0, double: 1, int: 2, float: 3, string: 4, json: 4, raw: 5, rpc: 5, msgpack: 5, protobuf: 5,
  'boolean[]': 16, 'double[]': 17, 'int[]': 18, 'float[]': 19, 'string[]': 20,
};

export type NTValue = boolean | number | string | Uint8Array | boolean[] | number[] | string[];

interface Topic { id: number; name: string; type: string; }
interface Pub { pubuid: number; name: string; type: NTType; retain: boolean; last?: NTValue; }
interface Sub { subuid: number; topics: string[]; prefix: boolean; periodic: number; }

type Listener = () => void;

export class NT4Client {
  private ws: WebSocket | null = null;
  private topicsById = new Map<number, Topic>();
  private values = new Map<string, { value: NTValue; ts: number }>();
  private pubs = new Map<string, Pub>();
  private subs: Sub[] = [];
  private nextUid = 1;
  private serverOffsetUs: number | null = null;
  private pendingValues: { pub: Pub; value: NTValue }[] = [];
  private listeners = new Set<Listener>();
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private rttTimer: ReturnType<typeof setInterval> | null = null;
  private closed = false;
  private lastSentTs = 0;
  /** Incremented on every value change — cheap "something changed" signal. */
  public version = 0;
  public connected = false;

  private readonly host: string;
  private readonly clientName: string;

  constructor(host: string, clientName = 'sim-ui') {
    this.host = host;
    this.clientName = clientName;
    this.connect();
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /** Subscribe to topic names (or prefixes when prefix = true). */
  subscribe(topics: string[], prefix = true, periodic = 0.02): void {
    const sub: Sub = { subuid: this.nextUid++, topics, prefix, periodic };
    this.subs.push(sub);
    if (this.connected) this.sendSubscribe(sub);
  }

  get(name: string): NTValue | undefined {
    return this.values.get(name)?.value;
  }

  getTimestamp(name: string): number | undefined {
    return this.values.get(name)?.ts;
  }

  /** All topic names currently known (announced by the server). */
  topicNames(): string[] {
    return [...this.topicsById.values()].map((t) => t.name);
  }

  /**
   * Publish a value. retain = true (settings) → re-sent after a reconnect.
   * retain = false (one-shot actions) → never re-sent, so a restarted robot
   * doesn't replay an old "teleport" or "play".
   */
  set(name: string, type: NTType, value: NTValue, retain = true): void {
    let pub = this.pubs.get(name);
    if (!pub) {
      pub = { pubuid: this.nextUid++, name, type, retain };
      this.pubs.set(name, pub);
      if (this.connected) this.sendPublish(pub);
    }
    pub.last = value;
    this.values.set(name, { value, ts: this.serverTimeUs() });
    this.bump();
    this.sendValue(pub, value);
  }

  onChange(l: Listener): () => void {
    this.listeners.add(l);
    return () => this.listeners.delete(l);
  }

  close(): void {
    this.closed = true;
    this.ws?.close();
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    if (this.rttTimer) clearInterval(this.rttTimer);
  }

  /** Estimated server (robot FPGA) time in microseconds. */
  serverTimeUs(): number {
    return Math.round(nowUs() + (this.serverOffsetUs ?? 0));
  }

  // ── Connection ────────────────────────────────────────────────────────────

  private connect(): void {
    const url = `ws://${this.host}:5810/nt/${this.clientName}-${Math.floor(Math.random() * 1e6)}`;
    const ws = new WebSocket(url, ['v4.1.networktables.first.wpi.edu', 'networktables.first.wpi.edu']);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onopen = () => {
      this.connected = true;
      this.serverOffsetUs = null;
      this.lastSentTs = 0;
      this.topicsById.clear();
      this.sendRtt();
      this.rttTimer = setInterval(() => this.sendRtt(), 2000);
      for (const s of this.subs) this.sendSubscribe(s);
      for (const p of this.pubs.values()) {
        this.sendPublish(p);
        if (p.retain && p.last !== undefined) this.sendValue(p, p.last);
      }
      this.bump();
    };
    ws.onclose = () => {
      const was = this.connected;
      this.connected = false;
      if (this.rttTimer) clearInterval(this.rttTimer);
      this.pendingValues = [];
      if (was) this.bump();
      if (!this.closed) this.reconnectTimer = setTimeout(() => this.connect(), 1000);
    };
    ws.onerror = () => { /* onclose handles retry */ };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') this.onText(ev.data);
      else this.onBinary(new Uint8Array(ev.data as ArrayBuffer));
    };
  }

  private onText(text: string): void {
    let msgs: { method: string; params: any }[];
    try { msgs = JSON.parse(text); } catch { return; }
    for (const m of msgs) {
      if (m.method === 'announce') {
        this.topicsById.set(m.params.id, { id: m.params.id, name: m.params.name, type: m.params.type });
      } else if (m.method === 'unannounce') {
        this.topicsById.delete(m.params.id);
      }
    }
    this.bump();
  }

  private onBinary(data: Uint8Array): void {
    let changed = false;
    for (const msg of decodeMulti(data) as Iterable<unknown>) {
      if (!Array.isArray(msg) || msg.length < 4) continue;
      const [id, ts, , value] = msg as [number, number, number, NTValue];
      if (id === -1) {
        // RTT reply: value = our send time, ts = server time when it answered
        const sent = Number(value);
        const rtt = nowUs() - sent;
        const offset = Number(ts) + rtt / 2 - nowUs();
        const first = this.serverOffsetUs === null;
        this.serverOffsetUs = offset;
        if (first) this.flushPending();
        continue;
      }
      const topic = this.topicsById.get(id);
      if (!topic) continue;
      this.values.set(topic.name, { value, ts: Number(ts) });
      changed = true;
    }
    if (changed) this.bump();
  }

  // ── Sending ───────────────────────────────────────────────────────────────

  private sendJson(method: string, params: object): void {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify([{ method, params }]));
  }

  private sendSubscribe(s: Sub): void {
    this.sendJson('subscribe', {
      topics: s.topics, subuid: s.subuid,
      options: { prefix: s.prefix, periodic: s.periodic, all: false },
    });
  }

  private sendPublish(p: Pub): void {
    this.sendJson('publish', { name: p.name, pubuid: p.pubuid, type: p.type, properties: {} });
  }

  private sendRtt(): void {
    if (this.ws?.readyState !== WebSocket.OPEN) return;
    this.ws.send(encodeMessage(-1, 0, TYPE_INDEX.int, Math.round(nowUs()), 'int'));
  }

  private sendValue(pub: Pub, value: NTValue): void {
    // Never send before the clock is synced: a value stamped with the wrong
    // clock would make later (correctly stamped) values look "older".
    if (!this.connected || this.serverOffsetUs === null) {
      this.pendingValues = this.pendingValues.filter((p) => p.pub !== pub || !pub.retain);
      this.pendingValues.push({ pub, value });
      return;
    }
    if (this.ws?.readyState === WebSocket.OPEN) {
      // Strictly increasing timestamps: the server ignores values that look
      // older than the last one, and robot time can be paused (SimHooks) —
      // then the clock sync would otherwise make our stamps go backwards.
      const ts = Math.max(this.serverTimeUs(), this.lastSentTs + 1);
      this.lastSentTs = ts;
      this.ws.send(encodeMessage(pub.pubuid, ts, TYPE_INDEX[pub.type], value, pub.type));
    }
  }

  private flushPending(): void {
    const pending = this.pendingValues;
    this.pendingValues = [];
    for (const p of pending) this.sendValue(p.pub, p.value);
  }

  private bump(): void {
    this.version++;
    for (const l of this.listeners) l();
  }
}

function nowUs(): number {
  return performance.now() * 1000;
}

// MessagePack-encode [id, ts, type, value]. Doubles are always written as
// float64 (msgpack would otherwise turn 2.0 into an integer, which the NT
// server rejects for "double" topics).
function encodeMessage(id: number, ts: number, typeIdx: number, value: NTValue, type: string): Uint8Array {
  const parts: Uint8Array[] = [new Uint8Array([0x94]), encode(id), encode(ts), encode(typeIdx)];
  if (type === 'double') parts.push(f64(Number(value)));
  else if (type === 'double[]') {
    const arr = value as number[];
    parts.push(arrayHeader(arr.length));
    for (const v of arr) parts.push(f64(v));
  } else if (type === 'float') parts.push(f32(Number(value)));
  else if (type === 'float[]') {
    const arr = value as number[];
    parts.push(arrayHeader(arr.length));
    for (const v of arr) parts.push(f32(v));
  } else parts.push(encode(value));

  const total = parts.reduce((n, p) => n + p.length, 0);
  const out = new Uint8Array(total);
  let o = 0;
  for (const p of parts) { out.set(p, o); o += p.length; }
  return out;
}

function f64(v: number): Uint8Array {
  const b = new Uint8Array(9);
  b[0] = 0xcb;
  new DataView(b.buffer).setFloat64(1, v);
  return b;
}

function f32(v: number): Uint8Array {
  const b = new Uint8Array(5);
  b[0] = 0xca;
  new DataView(b.buffer).setFloat32(1, v);
  return b;
}

function arrayHeader(n: number): Uint8Array {
  if (n < 16) return new Uint8Array([0x90 | n]);
  if (n < 0x10000) return new Uint8Array([0xdc, n >> 8, n & 0xff]);
  return new Uint8Array([0xdd, (n >>> 24) & 0xff, (n >>> 16) & 0xff, (n >>> 8) & 0xff, n & 0xff]);
}
