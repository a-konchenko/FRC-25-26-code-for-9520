// Play check: run an auto/path through the robot code and report plan-vs-fact.
//   node scripts/play-test.ts [kind 0|1|2] [name] [red 0|1] [preset]
import { NT4Client } from '../src/nt/nt4.ts';
const [kind = '1', name = 'Test path', red = '0', preset = 'ideal'] = process.argv.slice(2);
const PRESETS: Record<string, Record<string, number>> = {
  ideal: { wheelScalePct: 0, slipCmPerSqrtM: 0, gyroBiasDegPerMin: 0, gyroScalePct: 0 },
  welltuned: { wheelScalePct: 1.5, slipCmPerSqrtM: 1, gyroBiasDegPerMin: 0.3, gyroScalePct: 0.2 },
};
const nt = new NT4Client('localhost', 'play-test');
nt.subscribe(['/Sim/'], true, 0.02);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
await sleep(1500);
for (const [k, v] of Object.entries(PRESETS[preset])) nt.set(`/Sim/Control/drift/${k}`, 'double', v);
nt.set('/Sim/Control/run/name', 'string', name);
await sleep(200);
nt.set('/Sim/Control/play', 'double[]', [Date.now(), Number(kind), Number(red), 0, 0, 0], false);
const g = (t: string) => nt.get(`/Sim/${t}`) as any;
let maxTrack = 0, maxLoc = 0, lastPrint = -1, sawRunning = false;
const t0 = Date.now();
while (Date.now() - t0 < 26000) {
  await sleep(20);
  const st = g('Run/state');
  if (st === 'running') sawRunning = true;
  const pair = g('TruthEstimate') as number[] | undefined;
  const tr = pair?.slice(0, 3), est = pair?.slice(3, 6), sp = pair?.slice(7, 10), spActive = !!pair && pair[6] > 0.5;
  if (spActive && est && sp) maxTrack = Math.max(maxTrack, Math.hypot(est[0] - sp[0], est[1] - sp[1]));
  if (est && tr) maxLoc = Math.max(maxLoc, Math.hypot(est[0] - tr[0], est[1] - tr[1]));
  const el = g('Run/elapsed') ?? 0;
  if (st === 'running' && Math.floor(el) !== lastPrint) {
    lastPrint = Math.floor(el);
    console.log(`t=${el.toFixed(1).padStart(4)}s mode=${g('Match/mode')} matchTime=${(g('Match/time') ?? 0).toFixed(1)} est=[${est.map((x: number) => x.toFixed(2))}] setpoint=${g('SetpointActive') ? `[${sp.map((x: number) => x.toFixed(2))}]` : '—'} path=${g('ActivePathName') || '—'} (${(g('ActivePath') ?? []).length / 3} pts)`);
  }
  if (sawRunning && st === 'finished') break;
}
console.log(`state=${g('Run/state')} elapsed=${(g('Run/elapsed') ?? 0).toFixed(2)} commandEnd=${(g('Run/commandEnd') ?? -1).toFixed(2)} error="${g('Run/error')}"`);
console.log(`max tracking error ${(maxTrack * 100).toFixed(1)} cm, max localization error ${(maxLoc * 100).toFixed(1)} cm, mode now ${g('Match/mode')}`);
nt.close(); process.exit(0);
