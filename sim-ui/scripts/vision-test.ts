// Phase 3/4 check: cameras correct a kidnapped robot; LL4 aims; ShootAllAuto scores.
//   node scripts/vision-test.ts
import { NT4Client } from '../src/nt/nt4.ts';
const nt = new NT4Client('localhost', 'vision-test');
nt.subscribe(['/Sim/'], true, 0.02);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const g = (t: string) => nt.get(`/Sim/${t}`) as any;
const set = (k: string, v: number) => nt.set(`/Sim/Control/${k}`, 'double', v);
const act = (k: string, args: number[] = []) => nt.set(`/Sim/Control/${k}`, 'double[]', [Date.now() + Math.random(), ...args], false);
const loc = () => { const p = g('TruthEstimate') as number[]; return Math.hypot(p[0] - p[3], p[1] - p[4]) * 100; };
await sleep(1500);
for (const [k, v] of Object.entries({ wheelScalePct: 0, slipCmPerSqrtM: 0, gyroBiasDegPerMin: 0, gyroScalePct: 0 })) set(`drift/${k}`, v);
set('cam/ll3/enabled', 1); set('cam/ll4/enabled', 1); set('cam/pv/enabled', 0);

// 1. Kidnap with cameras ON — estimate should converge to truth
act('placeRobot', [2.6, 4.0, 0]); await sleep(800);
console.log('LL3 sees tags', JSON.stringify(g('Vision/ll3/visible')), ' LL4 sees', JSON.stringify(g('Vision/ll4/visible')), ' LL4 frame [n,tv,tx,ty,ta]', JSON.stringify((g('Vision/ll4/frame') ?? []).map((x: number) => +x.toFixed(2))));
act('kidnap', [2.2, 3.4, 0]); await sleep(100);
const f0 = (g('Vision/fused') ?? []).length / 5;
for (const t of [0.1, 0.5, 1, 2, 3]) { await sleep(t === 0.1 ? 0 : (t - (t === 0.5 ? 0.1 : t === 1 ? 0.5 : t === 2 ? 1 : 2)) * 1000); console.log(`  t+${t}s after kidnap 0.72 m: localization error ${loc().toFixed(1)} cm`); }
console.log('  fused measurements in ring:', (g('Vision/fused') ?? []).length / 5, '(was', f0, ')');

// 2. Same kidnap with cameras OFF — no correction
set('cam/ll3/enabled', 0); act('syncTruth'); await sleep(300);
act('kidnap', [2.6, 4.6, 0]); await sleep(2000);
console.log(`cameras OFF, 2 s after kidnap: localization error ${loc().toFixed(1)} cm (should stay ~60)`);
set('cam/ll3/enabled', 1); await sleep(2500);
console.log(`cameras back ON, 2.5 s later: ${loc().toFixed(1)} cm`);

// 3. ShootAllAuto from 25° off-target, ~3 m from the blue hub
act('resetFuel');
nt.set('/Sim/Control/run/name', 'string', 'Shoot All (Stationary)'); await sleep(100);
act('placeRobot', [1.8, 3.2, 0.44]); await sleep(500);
act('play', [2, 0, 0, 0, 0]);
for (let i = 0; i < 10; i++) {
  await sleep(1000);
  const m = g('Mech/state') ?? [], f = g('Fuel/state') ?? [], ll4 = g('Vision/ll4/frame') ?? [];
  console.log(`  t=${(g('Run/elapsed') ?? 0).toFixed(1)}s tx=${(ll4[2] ?? 0).toFixed(2)}° flywheel ${(m[0] ?? 0).toFixed(1)}/${(m[1] ?? 0).toFixed(0)} rps feeder ${(m[2] ?? 0).toFixed(1)}  fuel left ${f[0]} scored ${f[1]} missed ${f[2]}  state=${g('Run/state')}`);
}
const shots = g('Fuel/shots') ?? [];
const reasons = []; for (let k = 0; k + 5 < shots.length; k += 6) reasons.push(['HIT', 'aim', 'close', 'far', 'slow'][shots[k + 5]]);
console.log('shots:', reasons.join(' '), ' commandEnd', (g('Run/commandEnd') ?? -1).toFixed(2));
act('stop');
nt.close(); process.exit(0);
