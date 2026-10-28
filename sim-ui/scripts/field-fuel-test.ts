// Phase 5a check: FUEL on the field — staging, bumper push, intake pickup,
// chute release, scored FUEL coming back out of the hub, the 504 total, loop time.
//   node scripts/field-fuel-test.ts
import { NT4Client } from '../src/nt/nt4.ts';
const nt = new NT4Client('localhost', 'field-fuel-test');
nt.subscribe(['/Sim/', '/SmartDashboard/Intake'], true, 0.02);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const g = (t: string) => nt.get(`/Sim/${t}`) as any;
let seq = Date.now();
const act = (k: string, args: number[] = []) => nt.set(`/Sim/Control/${k}`, 'double[]', [++seq, ...args], false);
// {in robot, scored, missed, preload, on field, in hubs, blue chute, red chute, capacity, total, picked up}
const fuel = () => (g('Fuel/state') ?? []) as number[];
let failures = 0, maxMs = 0;
const check = (ok: boolean, msg: string) => { console.log(`  ${ok ? 'PASS' : 'FAIL'} ${msg}`); if (!ok) failures++; };
const watchMs = setInterval(() => { maxMs = Math.max(maxMs, g('Fuel/physicsMs') ?? 0); }, 20);
const ballsNear = (x0: number, x1: number, y0: number, y1: number) => {
  const p = (g('Fuel/field') ?? []) as number[];
  let k = 0;
  for (let i = 0; i + 1 < p.length; i += 2) if (p[i] > x0 && p[i] < x1 && p[i + 1] > y0 && p[i + 1] < y1) k++;
  return k;
};

await sleep(1500);
if (!nt.connected) { console.log('Robot sim not reachable on localhost:5810'); process.exit(2); }
act('clock', [0, 0]);
for (const [k, v] of Object.entries({ wheelScalePct: 0, slipCmPerSqrtM: 0, gyroBiasDegPerMin: 0, gyroScalePct: 0 })) nt.set(`/Sim/Control/drift/${k}`, 'double', v);
nt.set('/Sim/Control/fuel/preload', 'double', 8);
nt.set('/Sim/Control/fuel/capacity', 'double', 40);
await sleep(200);

console.log('1. Staging (game manual 6.3.4)');
act('resetFuel'); await sleep(400);
let f = fuel();
check(f[9] === 504, `total ${f[9]} = 504`);
check(f[4] === 448, `on field ${f[4]} = 400 neutral zone + 2×24 depot`);
check(f[6] === 24 && f[7] === 24, `chutes ${f[6]} / ${f[7]} = 24 / 24`);
check(f[0] === 8, `in robot ${f[0]} = 8 preload`);

// Square test drive from (6.2, 3.0) facing +X: the first leg drives 2 m into the neutral-zone pile
async function square(label: string) {
  act('placeRobot', [6.2, 3.0, 0]); await sleep(400);
  act('testDrive', [1]);
  await sleep(1500);
  for (let i = 0; i < 300 && g('Match/mode') !== 'disabled'; i++) await sleep(20);
  await sleep(600); // let pushed balls roll to a stop
  console.log(`  (${label}: square done, mode ${g('Match/mode')})`);
}

console.log('2. Intake OFF: bumpers push, nothing picked up');
act('intake', [0]); await sleep(200);
const pileBefore = ballsNear(7.2, 8.2, 2.5, 3.5);
await square('intake off');
f = fuel();
check(f[10] === 0 && f[0] === 8, `picked up ${f[10]}, in robot ${f[0]} (still 8)`);
check(ballsNear(7.2, 8.2, 2.5, 3.5) < pileBefore, `balls pushed out of the robot's path (${pileBefore} → ${ballsNear(7.2, 8.2, 2.5, 3.5)})`);
check(f[9] === 504, `total ${f[9]} = 504`);

console.log('3. Intake ON: picks up from the front');
act('resetFuel'); await sleep(300);
act('intake', [1]); await sleep(200);
check(nt.get('/SmartDashboard/Intake/Running') === true, 'robot code intake on (setIntake)');
await square('intake on');
f = fuel();
check(f[10] > 5 && f[0] === 8 + f[10], `picked up ${f[10]}, in robot ${f[0]}`);
check(f[0] <= f[8], `never more than capacity (${f[0]} ≤ ${f[8]})`);
check(f[9] === 504, `total ${f[9]} = 504`);
act('intake', [0]);

console.log('4. Capacity: full robot stops picking up');
act('resetFuel'); nt.set('/Sim/Control/fuel/capacity', 'double', 12); await sleep(300);
act('intake', [1]); await sleep(200);
await square('capacity 12');
f = fuel();
check(f[0] === 12, `in robot ${f[0]} = capacity 12`);
act('intake', [0]); nt.set('/Sim/Control/fuel/capacity', 'double', 40);

console.log('5. Human player releases the blue chute');
act('resetFuel'); await sleep(300);
const onField0 = fuel()[4];
act('releaseChute', [0]); await sleep(5000);
f = fuel();
check(f[6] === 0 && f[7] === 24, `chutes ${f[6]} / ${f[7]}`);
check(f[4] === onField0 + 24, `on field ${onField0} → ${f[4]}`);
check(ballsNear(0, 4.0, 0, 2.5) >= 24, `balls rolled out near the blue outpost (${ballsNear(0, 4.0, 0, 2.5)})`);

console.log('6. Scored FUEL comes back out of the hub');
// Shoot All aims with the LL4 — don't depend on what an open browser tab has set
nt.set('/Sim/Control/cam/ll4/enabled', 'double', 1);
act('resetFuel'); await sleep(200);
nt.set('/Sim/Control/run/name', 'string', 'Shoot All (Stationary)'); await sleep(100);
act('placeRobot', [1.8, 3.2, 0.44]); await sleep(500);
const before = fuel()[4];
act('play', [2, 0, 0, 0, 0]);
await sleep(8000);
f = fuel();
check(f[1] >= 6, `scored ${f[1]} (run ${g('Run/state')}, mode ${g('Match/mode')}${g('Run/error') ? `, error "${g('Run/error')}"` : ''})`);
check(f[4] === before + f[1] + f[2] && f[5] === 0, `every shot back on the floor: on field ${before} → ${f[4]}, in hub ${f[5]}`);
check(f[9] === 504, `total ${f[9]} = 504`);
act('stop');

clearInterval(watchMs);
check(maxMs < 4, `physics ≤ ${maxMs.toFixed(2)} ms per robot loop (budget 4 ms)`);
console.log(failures ? `${failures} check(s) FAILED` : 'All checks passed');
nt.close(); process.exit(failures ? 1 : 0);
