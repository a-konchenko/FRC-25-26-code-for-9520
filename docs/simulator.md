# Team 9520 Autonomy Simulator

Run the **real robot code** on a laptop, on a to-scale 2026 REBUILT field, and see
where the robot *thinks* it is versus where it *really* is.

> Status: **Phases 1–2 complete**: backbone, Plan tab (PathPlanner-style editor)
> and Play for autos. Cameras, shooter and collisions arrive in phases 3–5
> (see [Roadmap](#roadmap)).

---

## Quick start

Prerequisites (one time):

- **WPILib 2026** installer: gives Java 17 and VS Code ([docs.wpilib.org](https://docs.wpilib.org))
- **Node.js 20 LTS or newer** ([nodejs.org](https://nodejs.org))

Then, from the repository root:

```bash
./sim.sh              # robot code in simulation + web UI (opens the WPILib sim GUI too)
./sim.sh --headless   # same, without the WPILib GUI window
```

Open **http://localhost:5173**. `Ctrl+C` stops everything.

Running the two halves separately works too (e.g. from WPILib VS Code):

```bash
cd "Swerve9520 ctre tuner x project (og home base ver.)"
./gradlew simulateJava            # or VS Code: "WPILib: Simulate Robot Code"
cd sim-ui && npm install && npm run dev
```

The web UI connects to the robot sim over NetworkTables on port 5810, the same
way Elastic does. To view a sim running on another laptop: `http://localhost:5173/?robot=<ip>`.

---

## The student workflow

1. **Plan tab**: open or create a path (＋ next to *Paths*) and drag it into shape.
   **◷ Preview** shows the exact timing. Build an auto from paths and named commands.
2. **▶ Run this**: saves, switches to the Run tab and plays it through the real robot code.
3. **Run tab**: watch plan vs fact, then scrub the timeline to see where it went wrong.
   Change drift or placement error and play again.

## Plan tab

The Plan tab edits the robot project's **real PathPlanner files** in
`src/main/deploy/pathplanner/` (`paths/*.path`, `autos/*.auto`, `settings.json`).
The PathPlanner app opens the same files, and PathPlannerLib on the robot runs them.

| On the field | How to use it |
|---|---|
| Purple curve, numbered circles | The path and its waypoints (green = start, orange = end). Drag a waypoint to move it. |
| Small filled dots on lines | Bézier handles of the selected waypoint: drag to bend the curve. Shift-drag breaks the smooth tangent. |
| Purple robot outlines at start/end | Start and end pose. Drag the small circle in front of each outline to set the heading. |
| Dashed outlines along the path | Rotation targets (heading at a point). Drag the dot to slide it along the path, or the circle to turn it. |
| ◆ Orange diamonds | Event markers: a named command that runs **while driving** (drag along the path). |
| Faint outlines | The robot's footprint along the path. **Red dots** mean the bumpers hit the Hub, Tower or a wall. |

- **＋ Waypoint** adds waypoints at the end. *Insert after* and *Delete* are in the Waypoints panel.
- **Undo / redo**: Ctrl/Cmd+Z, Shift+Ctrl/Cmd+Z. **Save**: Ctrl/Cmd+S.
- **◷ Preview**: with the robot sim running, the path is timed by **PathPlannerLib
  itself** (same code and `settings.json` as the robot). The ghost robot replays it
  and the chart shows velocity. Without the sim it falls back to an approximate
  trapezoid profile, and says so.
- **Autos**: a tree of groups (*sequential* = one after another, *parallel* = all
  until all finish, *race* = until any finishes, *deadline* = until the first
  finishes) holding **path**, **named** and **wait** steps. The field draws the
  paths in order.
- **Robot settings** edits `settings.json` (mass, MOI, wheel friction, gearing, max
  speed). Values marked * are placeholders.
- Every first save or delete of a file keeps a backup in `sim-ui/.sim-backup/`
  (outside the deploy folder, so it's never deployed).
- Saving an unchanged file only changes formatting (`180.0` → `180`); PathPlanner reads both.

**Named commands** (registered in `RobotContainer.registerNamedCommands`):

| Name | Does | Use as |
|---|---|---|
| `SpinUp` | Start the flywheel (feeder off) | marker or step |
| `Shoot` / `ShootAll` | Spin up (≤ 1.5 s, until at speed), then feed 1 s / 4 s | marker or step |
| `StopShooter` | Stop shooter + feeder | marker or step |
| `IntakeOn` / `IntakeOff` | Intake roller on/off | marker or step |
| `ArmDown` / `ArmUp` | Run the intake arm 0.6 s | marker or step |
| `AutoAim` | Turn in place until LL4 tx ≈ 0 (≤ 2 s) | **step only** (uses the drivetrain) |
| `AimAndShoot` | AutoAim, then ShootAll | **step only** |
| `WaitShoot1`, `Waitshootall` | Same as Shoot / ShootAll (names used by `Test path`) | marker or step |

Event markers run **while the path is driving**, so they must not use the drivetrain.
AutoAim would stop the path. Put drivetrain commands as auto steps instead.

## Run tab

Top bar: choose **Auto** (`.auto`), **Path** (one `.path`) or **Hand-coded**
(the autos written in Java), pick the name and the alliance, then press **▶ Play**.

What Play does:

1. Stops anything running.
2. Builds the command **fresh from the files**, so edits are picked up without restarting the sim.
3. Puts the robot code's estimate on the planned start pose, and the true robot on
   start + **placement error**.
4. Enables autonomous: the robot code's own `Robot.autonomousInit()` schedules the command,
   exactly like a match.
5. Disables after **20 s**.

Red alliance mirrors everything (rotational symmetry). Hand-coded autos start
wherever the robot is, so use *Place robot* first.

| On the field | Meaning |
|---|---|
| **Purple line** | **Plan**: the path file(s). Thicker while that path is being followed. |
| **Orange outline** | **Setpoint**: where the path follower wants the robot *right now* |
| Dashed cyan outline + trace | **Estimate**: where the robot code thinks it is (`drivetrain.getState().Pose`) |
| Solid robot, green outline + trace | **Truth**: where the robot really is |
| Small orange arrows on the wheels | Each swerve module's steer angle and speed |
| Black bars with numbers | AprilTags, from the same WPILib layout the robot code loads |

The chart under the field splits the blame:

- **Tracking error** = setpoint vs estimate. The path follower isn't keeping up:
  a tuning problem, or a path that is too fast or too sharp.
- **Localization error** = estimate vs truth. The robot doesn't know where it is:
  drift or placement error. Cameras (phase 3) fix this.

Click or drag the chart to **scrub** the recorded run (REPLAY); **● Live** returns.
*Last run* shows max/final errors and when the auto command finished (or "not in time").

**Controls**

- **Placement error**: X/Y offset (±30 cm) and heading offset (±5°) between the planned start and where the robot really starts.
- **Place robot**: click to set position, drag to aim (snaps to 15°, hold Shift for any angle). Moves *both* truth and estimate.
- **Kidnap**: moves *only* the truth. The robot code is not told. Once cameras exist (phase 3), watch them notice.
- **Sync truth**: snaps the truth back onto the estimate and clears drift.
- **Test: square / spin**: scripted teleop drive (a 2 m square at 1.5 m/s, or two turns in place) to see drift without a joystick.
- **Pause / Step 20 ms / Resume**: freezes robot time (code, physics and timers together), then steps one robot loop at a time.
- **Drift panel**: presets plus a slider for each error source; see below.
- **Field** (top right): AndyMark or Welded field. The robot code uses AndyMark.

To drive with the keyboard, run `./sim.sh` (with the GUI) and, in the WPILib sim
GUI, drag "Keyboard 0" onto Joystick 0. The controls are the same as on the real robot.

---

## How it works

```
┌──────────── Java: ./gradlew simulateJava ─────────────────────────────┐
│ YOUR ROBOT CODE (unchanged logic): RobotContainer, drivetrain + pose  │
│ estimator, VisionSubsystem, Shooter, commands, PathPlannerLib          │
│                ▲ simulated sensors          │ motor outputs           │
│ SIM LAYER  frc.robot.sim  (only when RobotBase.isSimulation())        │
│   SimManager · WorldModel · DriftModel · SimClock · SimNT  …          │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ NetworkTables 4, ws://localhost:5810
┌──────────────────────────────▼────────────────────────────────────────┐
│ WEB UI  sim-ui/ (Vite + React + TypeScript)   [ Plan ] [ Run ]        │
└───────────────────────────────────────────────────────────────────────┘
```

1. **CTRE's swerve sim** (already in `CommandSwerveDrivetrain`) spins simulated
   motors. The robot code's odometry reads the encoders and the Pigeon and produces the **Estimate**.
2. **WorldModel** reads the same encoder and gyro samples (250 Hz, like the real
   odometry) but passes them through the **DriftModel** first. That gives the **Truth**.
   With drift at zero, Truth equals Estimate exactly.
3. Cameras (phase 3) will look at the field from the Truth, so vision measurements
   tell the robot code where it really is.

### Drift model

Real robots drift because the floor doesn't move exactly the way the wheels and
gyro report. These are educated guesses, all adjustable in the UI:

| Source | Well-tuned default | What it models |
|---|---|---|
| Wheel scale error | 1.5 % | Worn tread / carpet compression: the wheels over-report distance (typically 1–3 %) |
| Random slip | 1 cm/√m | Random-walk slip; grows with √distance |
| Hard-accel slip | ×3 above 3 m/s² | Wheels break traction under hard acceleration |
| Gyro bias drift | 0.3°/min | Pigeon 2 heading creep (only while enabled) |
| Gyro scale error | 0.2 % | Heading error per rotation; shows up when spinning |

Presets: **Ideal** (all zero), **Well-tuned**, **Worn wheels**, **Rough match**.
With Well-tuned, expect roughly **10–20 cm and ~1° after a 20 s auto** with no
vision. Measured in the sim: about 5 cm peak around a 7 m closed square (scale
errors partly cancel on a loop) and 1.4° after two full spins. The drift panel
estimates about 16 cm after a 10 m straight drive. The **seed** makes the random part repeatable, so two runs
can be compared fairly.

---

## Blocks created for the simulator (★ NEW)

Blocks marked ★ don't exist in the current robot code. They were created for the
simulator and live in `frc.robot.sim` (Java) or `sim-ui/` (web). None of them
run on the real robot.

| Block | In robot code before? | Where | Phase | Status |
|---|---|---|---|---|
| Swerve modules (drive/steer motors, CANcoders) | ✅ CTRE sim | `CommandSwerveDrivetrain` | — | used as is* |
| Pigeon 2 gyro | ✅ CTRE sim | `CommandSwerveDrivetrain` | — | used as is |
| Odometry + pose estimator | ✅ CTRE | `CommandSwerveDrivetrain` | — | used as is: the **Estimate** |
| ★ World model (true pose) | ❌ | `sim/WorldModel.java` | 1 | ✅ done |
| ★ Drift model | ❌ | `sim/DriftModel.java` | 1 | ✅ done |
| ★ Sim clock (pause / step) | ❌ | `sim/SimClock.java` | 1 | ✅ done |
| ★ Sim manager + NT bridge | ❌ | `sim/SimManager.java`, `sim/SimNT.java` | 1 | ✅ done |
| ★ Scripted test drive | ❌ | `SimManager.startTestDrive` | 1 | ✅ done |
| ★ Field model (2026 elements) | ❌ | `sim-ui/scripts/gen-field.mjs` → `field-2026.json` | 1 | ✅ done |
| ★ Web UI (field, robot, traces, drift, chart) | ❌ | `sim-ui/` | 1 | ✅ done |
| ★ NT4 web client | ❌ | `sim-ui/src/nt/nt4.ts` | 1 | ✅ done |
| ★ Plan tab (PathPlanner-style editor) | ❌ | `sim-ui/src/tabs/PlanTab`, `sim-ui/src/pathplanner/` | 2 | ✅ done |
| ★ PathPlanner file API (dev server) | ❌ | `sim-ui/server/deployFiles.ts` | 2 | ✅ done |
| ★ Match control (Play / Reset / 20 s auto) | ❌ | `sim/SimControl.java` | 2 | ✅ done |
| ★ Path logging (plan + setpoint) | ❌ | `sim/PathLogging.java` | 2 | ✅ done |
| ★ Exact preview (PathPlannerLib timing) | ❌ | `sim/PlanPreview.java` | 2 | ✅ done |
| ★ Run recording, timeline, plan-vs-fact charts | ❌ | `sim-ui/src/tabs/RunTab` | 2 | ✅ done |
| ★ Limelight 3 sim (MegaTag2) | ❌ (code reads it) | `sim/LimelightSim.java` | 3 | planned |
| ★ Limelight 4 sim (shooter tx/ty) | ❌ (code reads it) | `sim/LimelightSim.java` | 3 | planned |
| ★ PhotonVision sim (4 cameras) | partial (camera objects only) | `sim/PhotonSim.java` | 3 | planned |
| ★ Shooter flywheel + feeder sim | ❌ | `sim/ShooterSim.java` | 4 | planned |
| ★ Fuel (game piece) sim | ❌ | `sim/FuelSim.java` | 4 | planned |
| ★ Intake / arm / climber sims | ❌ | `sim/MechanismSims.java` | 4 | planned |
| ★ Field collisions + bump impulses | ❌ | `sim/Collisions.java` | 5 | planned |
| ★ Recorder, replay, run compare, export | ❌ | `sim/Recorder.java`, UI | 5 | planned |

\* One sim-only edit in `CommandSwerveDrivetrain.startSimThread()`: it now uses
WPILib's clock (`Timer.getFPGATimestamp()`), which stops when the sim is paused,
instead of CTRE's wall clock, and clamps each step to 20 ms. Without that, the
robot jumped forward after every pause. This method only runs in simulation.

---

## Robot code changes

### Bug fixes (these affect the real robot)

| Fix | File | Before → after |
|---|---|---|
| Auto-aim never turned | `commands/AimController.java` (new, shared by AutoAim + ShootAllAuto) | kP 0.03 was applied to tx **in radians** → ~0.005 rad/s, below the deadband; the sign was also inverted. Now kP = 4 (rad/s per rad), correct sign, no rotational deadband while aiming. **Retune on the robot.** |
| X toggled climber direction | `RobotContainer.java` | "D-pad center + X" was true on every X press, since the D-pad is "center" when untouched. Moved to **right-stick click**. |
| Sticks changed meaning while B was held | `RobotContainer.java` | Default drive used `X=LeftX, Y=-LeftY, rot=+RightX`; AutoAim used `X=-LeftY, Y=-LeftX` at full speed. Both now use the standard mapping `X=-LeftY, Y=-LeftX, rot=-RightX` and the same 0.75× speed. ⚠ If the **real** robot now drives sideways, the swerve encoder offsets are 90° off: re-zero them in Tuner X rather than swapping axes. |
| Feeder ran before aiming | `Shooter.java`, `ShootAllAuto.java` | `runShooter()` started flywheel and feeder together. New `spinUp()` / `feed()` / `isAtSpeed()`. Teleop **Y** spins up and feeds automatically once at speed; ShootAllAuto feeds only once aimed **and** at speed. |
| Back/Start + Y also ran the shooter | `RobotContainer.java` | Y is now blocked while Back or Start is held (SysId combos). |
| LL3 tag-count check read the latency slot | `VisionSubsystem.java` | `bp[6] < 1` → `bp[7] < 1` (index 7 is the tag count). Each LL3 frame is now used only once, timestamped from the NT change time minus latency. |
| Limelight "online" never went offline | `VisionSubsystem.java` | `hb > 0` stayed true after a disconnect; online now means the heartbeat is still changing. LL3 no longer reports OFFLINE just because it sees no tags. |
| Deprecated TalonFX constructor | `Shooter.java` | `TalonFX(id, String)` → `TalonFX(id, CANBus)`. |

### PathPlanner autonomous (phase 2, affects the real robot)

| Change | File | What it does on the robot |
|---|---|---|
| `configureAutoBuilder()` | `CommandSwerveDrivetrain.java` | Standard CTRE + PathPlanner setup: `PPHolonomicDriveController` (translation P = 10, rotation P = 7), `ApplyRobotSpeeds` with wheel force feedforwards, path flipping on red. It also sets `FlippingUtil` to rotational symmetry and the AndyMark field size (the layout the vision code uses). |
| `settings.json` (new) | `deploy/pathplanner/` | Robot config for PathPlannerLib (`RobotConfig.fromGUISettings`). Mass, MOI, wheel COF, motor and current limit are **placeholders**. |
| Named commands | `RobotContainer.java` | `registerNamedCommands()`: see the table in the Plan tab section. Registered before any auto is built. |
| Auto chooser | `RobotContainer.java` | Hand-written autos are kept. Every `.auto` file is added as "PP: <name>". |
| `setIntake(boolean)` | `RobotContainer.java` | Explicit on/off (the X toggle uses it). |
| Telemetry listeners | `CommandSwerveDrivetrain.java` | `registerTelemetry` is overridden so the robot code's telemetry still works and extra listeners can be added (`addTelemetryListener`). Nothing registers one on the robot; the sim's world model does. |
| `setAutonomousOverride` / `getHandCodedAutos` / `getNamedCommandNames` | `RobotContainer.java` | Used only by the simulator; `getAutonomousCommand()` returns the chooser selection unless the sim set an override. |

### Simulator hooks (sim only)

- `Robot.java`: creates `SimManager` only when `RobotBase.isSimulation()`; `simulationPeriodic()` calls `update()`.
- `build.gradle`: `-Pheadless` turns off the WPILib sim GUI (default unchanged).

---

## Field data and sources

All coordinates use the WPILib blue origin, in meters: +X from the blue wall toward red,
+Y to the left as seen from blue, angles counter-clockwise.

- **AprilTags**: come straight from WPILib 2026.2.1 (`2026-rebuilt-andymark.json` /
  `-welded.json`). The robot sim publishes them, so the UI shows exactly what the code uses.
- **Elements**: [`sim-ui/scripts/gen-field.mjs`](../sim-ui/scripts/gen-field.mjs) builds
  `field-2026.json` from the tag anchors plus dimensions in the
  [2026 Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf),
  section 5:
  - Hub 47 × 47 in, centered from its 8 tags
  - Bumps 73.0 × 44.4 in
  - Trenches 65.65 × 47.0 in, with a 22.25 in opening
  - Tower base 39.0 × 45.18 in, centered from tags 31/32
  - Outpost corral 35.8 in, from tags 29/30
  - Depot 42 × 27 in
  - Starting line at the 158.6 in alliance-zone edge
- Red elements are blue elements rotated 180° about the field center. The field is
  rotationally symmetric, which was checked against the tag layout.
- Hover any element in the UI to see its source.
- ⚠ **Depot position is approximate** (marked `*`): the manual text gives its size but
  not where it sits along the wall. Check it against the official Field Dimension
  Drawings and fix `depotCY` in `gen-field.mjs`, then run `npm run gen-field`.
- Game rules used: AUTO = **20 s** (7.4.1); up to **8 FUEL** preloaded; robot
  perimeter ≤ 110 in and height ≤ 30 in (R104); bumpers ≤ 4 in from the
  perimeter (R403), with foam ≥ 2.25 in (R402).

## Placeholders: TO MEASURE

All in [`SimRobotConstants.java`](../Swerve9520%20ctre%20tuner%20x%20project%20(og%20home%20base%20ver.)/src/main/java/frc/robot/sim/SimRobotConstants.java):

| Value | Placeholder | Notes |
|---|---|---|
| Frame length × width | 27 × 24 in | Wheelbase 23 × 20 in is known (TunerConstants) |
| Bumper thickness | 3.25 in | |
| Mass | 52 kg | With battery and bumpers |
| Moment of inertia | 5.5 kg·m² | Estimate from CAD if possible |
| Wheel COF | 1.2 | |
| LL3 mount | x 0.30, z 0.25 m, 20° up | Set in the Limelight web UI, not in code |
| LL4 mount | x 0.20 m (height 0.55 m and pitch 25° are from code) | |
| Shooter gearing / MOI | 1:1, 0.004 kg·m² | |

---

## NetworkTables contract

Everything is under `/Sim`. Poses are `double[] {x, y, θ rad}`.

**Robot → UI**

| Topic | Type | Content |
|---|---|---|
| `/Sim/Truth`, `/Sim/Estimate` | double[3] | Pose (both taken from the same odometry sample) |
| `/Sim/TruthEstimate` | double[10] | {truth x, y, θ, estimate x, y, θ, setpointActive, setpoint x, y, θ}: one value, so errors are never computed across two different loops |
| `/Sim/ActivePath` | double[] | {x, y, θ} × N: the path being followed (already flipped for red); empty when none |
| `/Sim/ActivePathName` | string | |
| `/Sim/Setpoint`, `/Sim/SetpointActive` | double[3], boolean | Where the path follower wants the robot now |
| `/Sim/Run/state` | string | idle / armed / running / finished |
| `/Sim/Run/id`, `name`, `kind`, `elapsed`, `commandEnd`, `duration`, `error` | | Current run: `commandEnd` = seconds when the auto command finished (−1 = still running or never) |
| `/Sim/Autos/pathplanner`, `paths`, `handCoded` | string[] | What can be played |
| `/Sim/NamedCommands` | string[] | Registered named commands |
| `/Sim/Plan/previewResult` | double[] | {id, totalTime, then t, x, y, θ, v per state} |
| `/Sim/Plan/previewError` | string | "id: message" |
| `/Sim/Modules` | double[8] | {steer rad, speed m/s} × 4 |
| `/Sim/Speeds` | double[3] | Chassis vx, vy, ω (robot frame) |
| `/Sim/Drift/state` | double[4] | {distance m, gyro error °, localization error m, heading error °} |
| `/Sim/Robot/size` | double[4] | {length, width with bumpers, frame length, frame width} m |
| `/Sim/Robot/moduleLocations` | double[8] | Module x, y (m, robot frame) |
| `/Sim/Robot/maxSpeedMps`, `ll3Mount`, `ll4Mount` | double / double[] | |
| `/Sim/Field/tags/<andymark\|welded>` | double[] | {id, x, y, z, yaw, pitch} per tag |
| `/Sim/Field/size/<variant>`, `/Sim/Field/robotCodeVariant` | | |
| `/Sim/Match/mode`, `/Sim/Match/time`, `/Sim/Match/alliance` | string / double | |
| `/Sim/Clock/paused` | boolean | |
| `/Sim/Time`, `/Sim/Heartbeat` | double | Robot time (s), loop counter |

**UI → robot** (`/Sim/Control/…`)

| Topic | Type | Meaning |
|---|---|---|
| `drift/<field>` | double | Drift setting (latest value wins) |
| `seed` | double | Random seed |
| `placeRobot` | double[] `[seq, x, y, θ]` | Move estimate + truth |
| `kidnap` | double[] `[seq, x, y, θ]` | Move truth only |
| `syncTruth` | double[] `[seq]` | Truth := estimate, reset drift |
| `testDrive` | double[] `[seq, pattern]` | 1 = square, 2 = spin |
| `stop` | double[] `[seq]` | Cancel test drive, disable |
| `clock` | double[] `[seq, op, arg]` | op 0 resume, 1 pause, 2 step `arg` seconds |
| `run/name` | string | Name for the next play/reset |
| `play` | double[] `[seq, kind, red, errX, errY, errθ]` | kind 0 = auto, 1 = path, 2 = hand-coded; placement error in m / rad (field frame) |
| `reset` | double[] `[seq, kind, red, errX, errY, errθ]` | Stop and place on the start pose |
| `/Sim/Plan/previewRequest` | string | `{"id": n, "path": {….path JSON…}}` |

One-shot actions carry a sequence number. The robot acts once each time it changes,
and the UI never re-sends an action after a reconnect.

## Developer scripts (`sim-ui/scripts/`)

Run each with the robot sim running:

- `node scripts/nt-smoke.ts`: lists every `/Sim` topic and its value.
- `node scripts/drift-test.ts [ideal|welltuned|worn] [1|2]`: runs a test drive and prints the drift.
- `node scripts/pause-test.ts`: checks that pause freezes, step advances exactly, and resume doesn't jump.
- `node scripts/play-test.ts [kind] [name] [red] [preset]`: plays an auto/path and prints tracking and localization error (e.g. `1 "Test path" 0 ideal` → localization 0.0 cm).
- `node scripts/preview-test.ts`: requests the exact preview of `Test path`.
- `npm run gen-field`: regenerates `field-2026.json` (UI and Java copies).

---

## Roadmap

| Phase | Delivers | Status |
|---|---|---|
| 1 Backbone | World model, drift, NT bridge, field, truth vs estimate, pause/step, bug fixes | ✅ |
| 2 Autonomy + Plan tab | AutoBuilder + named commands, PathPlanner-style editor, Play (20 s auto), plan/setpoint layers, error charts, timeline | ✅ |
| 3 Cameras | LL3 MegaTag2, LL4, PhotonVision sims, camera cones, correction dots, kidnap demo | next |
| 4 Mechanisms | Shooter, feeder, fuel scoring, intake, climber; ShootAllAuto playable | |
| 5 Polish | Collisions, bump impulses, run compare, narration, reports, exports, student exercises | |
