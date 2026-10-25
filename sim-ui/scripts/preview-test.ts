// Exact-preview check: send Test path JSON, get PathPlannerLib's timed trajectory back.
import { readFileSync } from 'node:fs';
import { NT4Client } from '../src/nt/nt4.ts';
const path = JSON.parse(readFileSync('../Swerve9520 ctre tuner x project (og home base ver.)/src/main/deploy/pathplanner/paths/Test path.path', 'utf8'));
const nt = new NT4Client('localhost', 'preview-test');
nt.subscribe(['/Sim/Plan/'], true, 0.02);
await new Promise((r) => setTimeout(r, 1200));
const id = 42 + Math.floor(Math.random() * 1000);
nt.set('/Sim/Plan/previewRequest', 'string', JSON.stringify({ id, path }));
for (let i = 0; i < 50; i++) {
  await new Promise((r) => setTimeout(r, 100));
  const r = nt.get('/Sim/Plan/previewResult') as number[] | undefined;
  if (r && r[0] === id) {
    const n = (r.length - 2) / 5;
    const vmax = Math.max(...Array.from({ length: n }, (_, k) => r[2 + 5 * k + 4]));
    console.log(`preview id ${id}: total time ${r[1].toFixed(2)} s, ${n} states, max v ${vmax.toFixed(2)} m/s, start [${r.slice(3, 6).map((x) => x.toFixed(2))}] end [${r.slice(r.length - 4, r.length - 1).map((x) => x.toFixed(2))}]`);
    break;
  }
  const e = nt.get('/Sim/Plan/previewError') as string;
  if (e && e.startsWith(String(id))) { console.log('error', e); break; }
}
nt.close(); process.exit(0);
