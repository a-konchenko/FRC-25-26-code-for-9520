// Smoke test: connect to the running robot sim, print /Sim values, send a placeRobot.
//   node scripts/nt-smoke.ts
import { NT4Client } from '../src/nt/nt4.ts';

const nt = new NT4Client('localhost', 'smoke');
nt.subscribe(['/Sim/'], true, 0.05);
setTimeout(() => {
  console.log('connected:', nt.connected);
  for (const n of nt.topicNames().filter((n) => n.startsWith('/Sim/')).sort()) {
    const v = nt.get(n);
    const s = Array.isArray(v) ? `[${v.slice(0, 6).map((x) => (typeof x === 'number' ? +x.toFixed(3) : x)).join(', ')}${v.length > 6 ? `, … (${v.length})` : ''}]` : String(v);
    console.log(n.padEnd(34), s);
  }
  nt.set('/Sim/Control/placeRobot', 'double[]', [Date.now(), 2.0, 2.0, Math.PI / 2], false);
}, 2500);
setTimeout(() => {
  console.log('after place: Truth', nt.get('/Sim/Truth'), 'Estimate', nt.get('/Sim/Estimate'));
  nt.close();
  process.exit(0);
}, 4000);
