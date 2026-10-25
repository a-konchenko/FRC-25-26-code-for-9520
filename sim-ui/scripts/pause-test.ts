// Pause/step/resume check: does robot time freeze, step, and resume without a pose jump?
import { NT4Client } from '../src/nt/nt4.ts';
const nt = new NT4Client('localhost', 'pause-test');
nt.subscribe(['/Sim/'], true, 0.01);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const clock = (op: number, arg = 0) => nt.set('/Sim/Control/clock', 'double[]', [Date.now() + Math.random(), op, arg], false);
const P = () => (nt.get('/Sim/Truth') as number[]).map((x) => +x.toFixed(3));
const T = () => +(nt.get('/Sim/Time') as number).toFixed(3);
await sleep(1500);
for (const [k, v] of Object.entries({ wheelScalePct: 0, slipCmPerSqrtM: 0, gyroBiasDegPerMin: 0, gyroScalePct: 0 })) nt.set(`/Sim/Control/drift/${k}`, 'double', v);
nt.set('/Sim/Control/placeRobot', 'double[]', [Date.now(), 2, 2, 0], false);
await sleep(400);
nt.set('/Sim/Control/testDrive', 'double[]', [Date.now(), 1], false);
let samples: number[][] = [];
const rec = setInterval(() => { const p = nt.get('/Sim/Truth') as number[]; if (p) samples.push([T(), p[0], p[1]]); }, 10);
await sleep(1600);
clock(1); await sleep(300);
const t0 = T(), p0 = P();
await sleep(2000);
console.log('paused:', nt.get('/Sim/Clock/paused'), ' time frozen:', T() === t0, t0, '→', T(), ' pose frozen:', JSON.stringify(P()) === JSON.stringify(p0));
for (let i = 0; i < 10; i++) { clock(2, 0.02); await sleep(60); }
await sleep(300);
console.log('after 10 steps of 20 ms: time advanced', (T() - t0).toFixed(3), 's, pose', JSON.stringify(p0), '→', JSON.stringify(P()));
clock(0);
await sleep(7000);
clearInterval(rec);
// largest jump between consecutive sim-time samples, as speed
let maxV = 0;
for (let i = 1; i < samples.length; i++) {
  const dt = samples[i][0] - samples[i - 1][0];
  if (dt <= 0) continue;
  maxV = Math.max(maxV, Math.hypot(samples[i][1] - samples[i - 1][1], samples[i][2] - samples[i - 1][2]) / dt);
}
const d = nt.get('/Sim/Drift/state') as number[];
console.log('resumed: final truth', JSON.stringify(P()), ' max apparent speed', maxV.toFixed(2), 'm/s (commanded 1.5)', ' loc err', (d[2] * 100).toFixed(2), 'cm');
nt.close(); process.exit(0);
