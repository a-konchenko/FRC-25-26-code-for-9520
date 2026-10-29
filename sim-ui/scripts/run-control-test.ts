// Run control check (review findings #1, #2, #7):
//   - Play while an auto is running really restarts with the new one
//   - a failed build / stop never leaves an auto queued for the next enable
//   - 300 teleports during a running path don't freeze the sim (no deadlock)
//   node scripts/run-control-test.ts
import { NT4Client } from '../src/nt/nt4.ts';
const nt = new NT4Client('localhost', 'run-control-test');
nt.subscribe(['/Sim/'], true, 0.02);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const g = (t: string) => nt.get(`/Sim/${t}`) as any;
let seq = Date.now();
const act = (k: string, args: number[] = []) => nt.set(`/Sim/Control/${k}`, 'double[]', [++seq, ...args], false);
let failures = 0;
const check = (ok: boolean, msg: string) => { console.log(`  ${ok ? 'PASS' : 'FAIL'} ${msg}`); if (!ok) failures++; };
async function play(name: string, kind = 1) {
  nt.set('/Sim/Control/run/name', 'string', name);
  await sleep(100);
  act('play', [kind, 0, 0, 0, 0]);
}
async function waitFor(cond: () => boolean, ms: number) {
  for (let t = 0; t < ms && !cond(); t += 20) await sleep(20);
  return cond();
}

await sleep(1500);
if (!nt.connected) { console.log('Robot sim not reachable on localhost:5810'); process.exit(2); }
act('clock', [0, 0]);
act('stop'); await sleep(300);

console.log('1. Play while an auto is running restarts with the new one (#2)');
await play('Test path');
check(await waitFor(() => g('Run/state') === 'running' && g('ActivePathName') === 'Test path', 3000), `first run: ${g('Run/state')}, path "${g('ActivePathName')}"`);
await sleep(2000);
const id1 = g('Run/id');
// (The sim passes through "disarming" for one 20 ms loop; NT may coalesce that
// away, so the checks below test the outcome instead.)
await play('Another test');
const restarted = await waitFor(() => g('Run/state') === 'running' && g('ActivePathName') === 'Another test', 3000);
check(restarted, `second run follows "Another test" (now ${g('Run/state')}, path "${g('ActivePathName')}")`);
check(g('Run/id') > id1 && g('Run/elapsed') < 1.5, `new run id ${g('Run/id')}, elapsed ${(g('Run/elapsed') ?? 0).toFixed(2)} s`);
check(g('Run/commandEnd') === -1, `commandEnd ${g('Run/commandEnd')} (-1 = still running)`);
const p = (g('TruthEstimate') ?? []) as number[];
check(p.length > 0 && Math.hypot(p[0] - 1.81, p[1] - 6.55) < 0.6, `robot near "Another test" start (${p.slice(0, 2).map((v) => v.toFixed(2))})`);

console.log('2. A failed build clears the queued auto (#7)');
await play('zz does not exist');
await sleep(600);
check(g('Run/state') === 'idle' && String(g('Run/error')).includes('zz does not exist'), `state ${g('Run/state')}, error "${g('Run/error')}"`);
check(g('Run/overrideActive') === false, `no auto queued (overrideActive ${g('Run/overrideActive')})`);
check(g('Match/mode') === 'disabled', `mode ${g('Match/mode')}`);

console.log('3. Stop clears the queued auto too');
await play('Test path');
await waitFor(() => g('Run/state') === 'running', 3000);
check(g('Run/overrideActive') === true, 'auto queued while running');
act('stop'); await sleep(400);
check(g('Run/overrideActive') === false && g('Match/mode') === 'disabled', `after Stop: overrideActive ${g('Run/overrideActive')}, mode ${g('Match/mode')}`);

console.log('4. Teleport stress while a path runs (#1, deadlock)');
await play('Test path');
await waitFor(() => g('Run/state') === 'running', 3000);
const hb0 = g('Heartbeat');
let maxGapMs = 0, last = Date.now(), lastHb = hb0;
const gaps = setInterval(() => {
  const hb = g('Heartbeat');
  if (hb !== lastHb) { lastHb = hb; last = Date.now(); } else maxGapMs = Math.max(maxGapMs, Date.now() - last);
}, 10);
for (let i = 0; i < 300; i++) {
  const x = 2 + Math.random() * 12, y = 1 + Math.random() * 6, th = Math.random() * 6.28;
  if (i % 3 === 0) act('placeRobot', [x, y, th]);
  else if (i % 3 === 1) act('kidnap', [x, y, th]);
  else act('syncTruth');
  await sleep(10);
}
await sleep(1000);
clearInterval(gaps);
const beats = g('Heartbeat') - hb0;
check(beats > 100, `robot loop kept running: ${beats} loops during ~4 s`);
check(maxGapMs < 500, `longest pause between loops ${maxGapMs} ms (< 500 ms)`);
act('stop');

console.log(failures ? `${failures} check(s) FAILED` : 'All checks passed');
nt.close(); process.exit(failures ? 1 : 0);
