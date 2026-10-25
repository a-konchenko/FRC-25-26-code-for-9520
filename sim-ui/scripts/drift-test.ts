// Drift check: run the scripted 2 m square with a drift preset and print the
// final localization error.   node scripts/drift-test.ts [preset] [pattern]
import { NT4Client } from '../src/nt/nt4.ts';

const PRESETS: Record<string, Record<string, number>> = {
  ideal:     { wheelScalePct: 0,   slipCmPerSqrtM: 0, gyroBiasDegPerMin: 0,   gyroScalePct: 0 },
  welltuned: { wheelScalePct: 1.5, slipCmPerSqrtM: 1, gyroBiasDegPerMin: 0.3, gyroScalePct: 0.2 },
  worn:      { wheelScalePct: 3,   slipCmPerSqrtM: 2, gyroBiasDegPerMin: 0.5, gyroScalePct: 0.4 },
};
const preset = process.argv[2] ?? 'welltuned';
const pattern = Number(process.argv[3] ?? 1);
const nt = new NT4Client('localhost', 'drift-test');
nt.subscribe(['/Sim/'], true, 0.05);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
await sleep(1500);
for (const [k, v] of Object.entries(PRESETS[preset])) nt.set(`/Sim/Control/drift/${k}`, 'double', v);
nt.set('/Sim/Control/placeRobot', 'double[]', [Date.now(), 2.0, 2.0, 0], false);
await sleep(500);
nt.set('/Sim/Control/testDrive', 'double[]', [Date.now(), pattern], false);
let maxErr = 0;
const t0 = Date.now();
await sleep(1000);
while (Date.now() - t0 < 15000) {
  const d = nt.get('/Sim/Drift/state') as number[];
  if (d) maxErr = Math.max(maxErr, d[2]);
  if (nt.get('/Sim/Match/mode') === 'disabled' && Date.now() - t0 > 2000) break;
  await sleep(100);
}
const d = nt.get('/Sim/Drift/state') as number[];
const fmt = (a: number[]) => a.map((x) => x.toFixed(3)).join(', ');
console.log(`preset=${preset} pattern=${pattern}`);
console.log('  truth   ', fmt(nt.get('/Sim/Truth') as number[]));
console.log('  estimate', fmt(nt.get('/Sim/Estimate') as number[]));
console.log(`  distance ${d[0].toFixed(2)} m, gyro err ${d[1].toFixed(2)}°, loc err ${(d[2] * 100).toFixed(1)} cm (max ${(maxErr * 100).toFixed(1)}), heading err ${d[3].toFixed(2)}°`);
nt.close();
process.exit(0);
