// Stop button check: Stop must disable the robot and halt motion — during a
// test drive, during a Play run, and when pressed while paused.
//   node scripts/stop-test.ts [path name]
import { NT4Client } from '../src/nt/nt4.ts';
const [pathName = 'Test path'] = process.argv.slice(2);
const nt = new NT4Client('localhost', 'stop-test');
nt.subscribe(['/Sim/'], true, 0.02);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const g = (t: string) => nt.get(`/Sim/${t}`) as any;
let seq = Date.now();
const action = (name: string, args: number[] = []) => nt.set(`/Sim/Control/${name}`, 'double[]', [++seq, ...args], false);
const speed = () => { const s = g('Speeds') ?? [0, 0, 0]; return Math.hypot(s[0], s[1]) + Math.abs(s[2]); };
// Truth, not Estimate: after stopping, vision keeps pulling the estimate toward
// the truth, so the estimate can move a few cm while the robot is still.
const pose = () => ((g('TruthEstimate') as number[] | undefined)?.slice(0, 3) ?? [0, 0, 0]);
const estimate = () => (g('Estimate') ?? [0, 0, 0]) as number[];
let failures = 0;
const check = (ok: boolean, msg: string) => { console.log(`  ${ok ? 'PASS' : 'FAIL'} ${msg}`); if (!ok) failures++; };

/** After Stop: disabled, speeds near zero, and the robot stays put. */
async function expectStopped(label: string) {
  await sleep(400);
  check(String(g('Match/mode')).toLowerCase().includes('disabled'), `${label}: mode is ${g('Match/mode')}`);
  check(speed() < 0.05, `${label}: speed ${speed().toFixed(3)} (< 0.05)`);
  const p0 = pose(), e0 = estimate();
  await sleep(1000);
  const p1 = pose(), e1 = estimate();
  const moved = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
  const estMoved = Math.hypot(e1[0] - e0[0], e1[1] - e0[1]);
  check(moved < 0.02, `${label}: robot (truth) moved ${(moved * 100).toFixed(1)} cm in the 1 s after stopping (< 2 cm); estimate moved ${(estMoved * 100).toFixed(1)} cm`);
}

await sleep(1500);
if (!nt.connected) { console.log('Robot sim not reachable on localhost:5810'); process.exit(2); }
action('clock', [0, 0]); // make sure we start unpaused
await sleep(200);

console.log('1. Stop during a test drive');
action('testDrive', [1]);
await sleep(1500);
check(speed() > 0.5, `moving before Stop (speed ${speed().toFixed(2)})`);
action('stop');
await expectStopped('test drive');

console.log(`2. Stop during Play of path '${pathName}'`);
nt.set('/Sim/Control/run/name', 'string', pathName);
await sleep(200);
action('play', [1, 0, 0, 0, 0]);
for (let i = 0; i < 100 && g('Run/state') !== 'running'; i++) await sleep(20);
await sleep(1500);
check(g('Run/state') === 'running' && speed() > 0.3, `running before Stop (state ${g('Run/state')}, speed ${speed().toFixed(2)}, error "${g('Run/error') ?? ''}")`);
action('stop');
await expectStopped('play');
check(g('Run/state') === 'finished', `run state is ${g('Run/state')} (finished)`);

console.log('3. Stop pressed while paused');
action('testDrive', [1]);
await sleep(1500);
action('clock', [1, 0]); // pause
await sleep(200);
action('stop');
await sleep(300);
action('clock', [0, 0]); // resume
await expectStopped('paused stop');

console.log(failures ? `${failures} check(s) FAILED` : 'All checks passed');
nt.close(); process.exit(failures ? 1 : 0);
