// PhotonVision sim check: enable the 4 Jetson cameras, kidnap with LL3 off, see PV correct it.
import { NT4Client } from '../src/nt/nt4.ts';
const nt = new NT4Client('localhost', 'pv-test');
nt.subscribe(['/Sim/', '/SmartDashboard/Status/', '/SmartDashboard/Vision/'], true, 0.02);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const g = (t: string) => nt.get(t) as any;
const set = (k: string, v: number) => nt.set(`/Sim/Control/${k}`, 'double', v);
const act = (k: string, args: number[] = []) => nt.set(`/Sim/Control/${k}`, 'double[]', [Date.now() + Math.random(), ...args], false);
const loc = () => { const p = g('/Sim/TruthEstimate') as number[]; return Math.hypot(p[0] - p[3], p[1] - p[4]) * 100; };
await sleep(1500);
for (const [k, v] of Object.entries({ wheelScalePct: 0, slipCmPerSqrtM: 0, gyroBiasDegPerMin: 0, gyroScalePct: 0 })) set(`drift/${k}`, v);
set('cam/ll3/enabled', 0); set('cam/pv/enabled', 1);
act('placeRobot', [6.5, 4.0, Math.PI]); await sleep(2500);
console.log('Jetson online:', g('/SmartDashboard/Status/Jetson Online'), ' PV tags this loop:', g('/SmartDashboard/Vision/PV Tags This Loop'), ' PV sightings:', (g('/Sim/Vision/pv/visible') ?? []).length / 2);
act('kidnap', [6.0, 3.6, Math.PI]); await sleep(100);
console.log(`kidnap 0.64 m: ${loc().toFixed(1)} cm`); await sleep(1500);
console.log(`1.5 s later (PV only): ${loc().toFixed(1)} cm`);
set('cam/pv/enabled', 0); set('cam/ll3/enabled', 1);
nt.close(); process.exit(0);
