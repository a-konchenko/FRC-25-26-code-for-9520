# Team 9520 Autonomy Simulator

Run the **real robot code** on a laptop, on a to-scale 2026 REBUILT field, and see
where the robot *thinks* it is versus where it *really* is.

> **Students:** start with the plain-language [User Guide](USER_GUIDE.md). This page is the technical reference.

> Status: **Phases 1–4 and 5a complete**: backbone, Plan tab, Play, cameras (Limelight 3/4,
> PhotonVision), shooter, mechanisms and fuel, plus all 504 FUEL on the field with
> physics and intake pickup. Robot collisions and polish are the rest of phase 5
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

**VS Code:** open `FRC-9520.code-workspace` (File → Open Workspace from File…) in
WPILib VS Code, not the repository folder. The WPILib commands (Build, Simulate,
Deploy) only work when the robot project is a top-level workspace folder. The
workspace also has tasks (Terminal → Run Task…) that start the simulator.

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

## Cameras (phase 3)

Simulated cameras write the **same NetworkTables entries the real ones do**, so
`VisionSubsystem` runs unchanged. They look at the field from the **true** robot pose
at the moment the frame was taken (now − latency), so their measurements pull the
estimate back toward the truth.

| Camera | What the robot code gets | Notes |
|---|---|---|
| **Limelight 3** (MegaTag2) | `botpose_orb_wpiblue`, `tl`, `cl`, `hb` | Solves position using the heading **your code sends** (`robot_orientation_set`), like the real MegaTag2. So it **cannot fix a heading error**: a heading error becomes a position error (2° ≈ 9 cm at 2.5 m). 62.5° × 48.9° FOV, 6 m range. |
| **Limelight 4** (shooter) | `tv`, `tx`, `ty`, `ta`, `hb` for the pipeline the code selects (0/1 hub blue/red, 2/3 trench) | Aims at the hub opening (`VisionSubsystem.HUB_HEIGHT_M`), using the code's own LL4 height and pitch. 82° × 56.2° FOV. |
| **PhotonVision ×4** (Jetson) | Results for "Front/Back/Left/Right Camera" | PhotonLib's own simulator, with the transforms from `VisionSubsystem.PHOTON_CAMERAS`. Off by default. Properties (1280×800, 100° diagonal, 30 fps) are estimates. |

A tag counts as seen when it is in the field of view, within range, facing the camera
(< 75° off its normal), and the sightline doesn't pass through a Hub. The Tower is open
uprights, so it doesn't block sightlines (`blocksVision: false`), but it still counts
for Plan-tab collision warnings. Tested in `CameraGeometryTest`.

**On the field:**
- Camera view cones (LL3 green, LL4 pink, PV blue), with dashed sightlines to each tag it sees.
- A hollow green circle: the latest Limelight 3 pose.
- **Green dots**: the vision measurements the robot code actually **fused** into its pose estimate.

**Per-camera controls (Cameras panel):**
- On/off, noise ×, latency, dropped frames.
- **Mount error**: the camera is physically somewhere other than the code/Limelight config says, which shows up as a constant bias.
- The LL4 panel shows the image as the code sees it: crosshair plus target at (tx, ty).

**Demos:**
- **Kidnap** with Limelight 3 on: a 72 cm teleport is corrected to under 2 cm within 0.5 s. With it off, the error stays.
- **Placement error with a heading offset**: the cameras fix X/Y but not the heading. That's the MegaTag2 lesson above.
- **Spin fast**: MegaTag2 frames are ignored above 360°/s (robot code). Below that, notice the estimate wobble near the tags while turning.

## Mechanisms and fuel (phase 4)

- The **shooter flywheel and feeder** (CAN 13/14), **intake** (15), **intake arm** (19) and
  **climbers** (16/17) have motor physics on the simulated TalonFX controllers.
  The robot code's own commands drive them (VelocityVoltage, DutyCycleOut).
  - Each ball takes about 8% out of the flywheel speed.
  - The arm and climbers stop at simulated end-of-travel limits.
  - Gearing, inertia and travel are placeholders.
- **Fuel** follows the agreed simple model, scored if aimed:
  - The robot starts with `preload` (≤ 8). While the feeder spins, one ball leaves every 0.25 s.
  - A ball **scores** if the true robot points within ±3° of its alliance hub, is 1.5–5.5 m away, and the flywheel is at ≥ 90% of its setpoint.
  - Otherwise it misses, with a reason: aim, too close, too far, or flywheel slow.
  - The shooter is assumed to face the robot's front (`SimRobotConstants.kShooterYawDeg`).
  - There's no ball flight.
- On the field, shots are drawn as **green** lines into the hub (scored) or **red** dashed (missed).
  The Fuel panel shows in robot / scored / missed and the last shot's reason. Fuel reloads on every Play and Reset.
- `Shoot All (Stationary)` now runs end to end: from 25° off, it aims to within 1°, spins up, and scores 8/8 in about 5 s.

## FUEL on the field and intake pickup (phase 5a)

All **504 FUEL** are in play, staged the way the
[2026 Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)
section 6.3.4 says:

| Where | Count | How |
|---|---|---|
| NEUTRAL ZONE | 408 − preload (400 with 8 preloaded) | Hex-packed in the 206 × 72 in box centered on the CENTER LINE, with the 2 in divider gap; random spots, so not a perfect grid. The manual's 360 plus the 40 the five missing robots would have preloaded (GM: FUEL not preloaded goes to the NEUTRAL ZONE). |
| Each DEPOT | 24 | Loose, inside the depot (its position along the wall is still approximate). |
| Each OUTPOST CHUTE | 24 | Held by the human player until **Blue chute / Red chute** is pressed, then rolls out of the OUTPOST opening, one ball every 0.15 s. |
| Our robot | preload (≤ 8) | The Fuel panel's preload slider. |

`FieldFuel.java` runs top-down 2D physics on the **true** robot pose, inside the robot loop (about 1–2.5 ms per 20 ms loop with 450+ balls, measured):

- Each ball is a 5.91 in disc that rolls, slows down on the carpet (1.2 m/s², so a ball pushed at 2 m/s rolls about 1.7 m), and bounces softly (restitution 0.3) off other balls, the guardrails, the HUBs, TOWERs and OUTPOSTs. BUMPs and TRENCHes don't stop FUEL.
- The bumpers push balls aside. The robot itself is **not** slowed by them (robot collisions are still planned).
- **Intake:** a ball touching the zone across the front bumper (frame width × 10 cm, drawn on the robot, bright while running) is picked up while the intake roller (CAN 15) spins, up to the robot's capacity (40) and intake rate (10 balls/s). Otherwise the bumper pushes it. All three are placeholders (TO MEASURE).
- **Shots stay in play:** a scored ball spends 1 s in the HUB, then comes out of one of its 4 exits into the NEUTRAL ZONE (GM 5.4; exit positions approximate). A missed ball lands at the end of its red miss line.
- **Intake on/off** in the Run tab calls the robot code's own `setIntake()` (what the X button does), so you can test pickup with *Test: square* and no controller. The roller only spins while the robot is enabled.
- The Fuel panel shows in robot / capacity, on field, picked up, in hub, and the total (always 504).
- Play, Reset and *Reset all FUEL* restage everything, using the drift **seed**, so two runs start from the same field.

Not modeled: FUEL pushed into the CORRAL, balls stacking or riding up the BUMPs, ball spin.

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
3. The camera sims look at the field from the Truth (at frame capture time), so vision
   measurements tell the robot code where it really is.
   With Ideal drift and cameras on, the estimate wanders by a centimeter or two because of camera noise.

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
| ★ Limelight 3 sim (MegaTag2) | ❌ (code reads it) | `sim/LimelightSim.java` | 3 | ✅ done |
| ★ Limelight 4 sim (shooter tx/ty) | ❌ (code reads it) | `sim/LimelightSim.java` | 3 | ✅ done |
| ★ PhotonVision sim (4 cameras) | partial (camera objects only) | `sim/PhotonSim.java` | 3 | ✅ done |
| ★ Camera visibility (FOV, range, facing, occlusion) | ❌ | `sim/CameraGeometry.java`, `sim/FieldModel.java` | 3 | ✅ done |
| ★ Truth history (camera latency) | ❌ | `WorldModel.truthAt` | 3 | ✅ done |
| ★ Camera UI (cones, sightlines, fused dots, LL4 view, controls) | ❌ | `sim-ui/src/tabs/RunTab/VisionLayer.tsx`, `CameraPanel.tsx` | 3 | ✅ done |
| ★ Shooter flywheel + feeder sim | ❌ | `sim/MechanismSims.java` | 4 | ✅ done |
| ★ Fuel (game piece) sim | ❌ | `sim/FuelSim.java` | 4 | ✅ done |
| ★ Intake / arm / climber sims | ❌ | `sim/MechanismSims.java` | 4 | ✅ done |
| ★ Mechanism + fuel UI | ❌ | `sim-ui/src/tabs/RunTab/MechPanel.tsx` | 4 | ✅ done |
| ★ FUEL on the field (504, physics, hub return, chutes) | ❌ | `sim/FieldFuel.java` | 5a | ✅ done |
| ★ Intake pickup from the front bumper | ❌ (code runs the roller) | `sim/FuelSim.java`, `FieldFuel.java` | 5a | ✅ done |
| ★ FUEL + intake UI | ❌ | `sim-ui/src/tabs/RunTab/FuelLayer.tsx`, `MechPanel.tsx` | 5a | ✅ done |
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

### Phases 3–4 (affect the real robot)

These two fixes come from problems the simulator exposed. ⚠ Both need checking on the robot.

| Change | File | What it does on the robot |
|---|---|---|
| **Aim never quite finished** | `commands/AimController.java` | Near the target, P alone asked for ~0.2 rad/s, below the drivetrain's static friction, so the robot stalled 2–3° off and `ShootAllAuto` never fired. Now there's a **minimum turn rate of 0.4 rad/s while outside the 1.5° tolerance** (`MIN_ROT_RAD_S`). Tested in `AimControllerTest`. **Tune on the robot.** |
| **Aimed while still sweeping** (review) | `commands/AimController.java` | "Aimed" also requires tx to change ≤ 0.15° per loop (`MAX_SETTLED_TX_CHANGE_DEG`), so a robot overshooting through the band doesn't fire. Tested. |
| **MegaTag2 yaw corrupted the heading** | `subsystems/VisionSubsystem.java` | MegaTag2's yaw is just the heading the code sent it, but it was fused with θ std dev 0.8 rad. While turning, that delayed heading pulled the estimate's heading off by ~14° and positions by up to ~1 m. Now: θ std dev = 9999999 (never trust MT2 yaw), MT2 frames are **ignored while spinning > 360°/s**, and `robot_orientation_set` also sends the yaw rate. A heading jump over 30° in one loop (a pose reset) is not counted as spinning. |
| Camera configs public | `VisionSubsystem.java` | `PHOTON_CAMERAS` / `CamConfig`, `getPhotonCameras()` and the LL4 constants (`LL4_HEIGHT_M`, `LL4_PITCH_DEG`, `HUB_HEIGHT_M`) are public, so the sim uses the code's own values. Read-only, no behavior change. |
| Vision measurement listeners | `CommandSwerveDrivetrain.java` | `addVisionMeasurementListener`: told about each fused measurement (the sim draws them). Nothing registers one on the robot. |

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
| Shooter direction | robot front (`kShooterYawDeg` = 0) | |
| Arm / climber gearing and travel | 60:1, 0.3 rot; 25:1, 8 rot | `MechanismSims.java` |
| PhotonVision camera properties | 1280×800, 100° diag, 30 fps, 30 ms | `PhotonSim.java` |
| Intake zone | frame width × 10 cm in front of the bumper | `kIntakeWidthM`, `kIntakeDepthM` |
| Hopper capacity / intake rate | 40 FUEL, 10 FUEL/s | `kHopperCapacity`, `kIntakeRatePerS` (also UI sliders) |
| FUEL rolling resistance / bounce | 1.2 m/s², restitution 0.3 | `FieldFuel.java` |
| HUB exit positions | 4 exits, 0.3 m apart on the neutral-zone face | `FieldFuel.java` (GM figure 5-8 is an approximation) |

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
| `/Sim/Vision/<ll3\|ll4>/config` | double[12] | {mount x, y, z, pitch, yaw, hFOV, vFOV, range, enabled, mountErrX, mountErrY, mountErrYaw} |
| `/Sim/Vision/<ll3\|ll4>/visible` | double[] | Tag IDs seen this frame |
| `/Sim/Vision/ll3/frame`, `/Sim/Vision/ll4/frame` | double[] | {frame count, x, y, θ, tags} / {frame count, tv, tx, ty, ta} |
| `/Sim/Vision/pv/mounts`, `params`, `visible` | double[] | Mounts {x, y, z, roll, pitch, yaw} × 4; {hFOV, vFOV, range, enabled, errX, errY, errYaw}; {camera, tag} pairs |
| `/Sim/Vision/fused` | double[] | {seq, x, y, θ, timestamp} × last 40: measurements the pose estimator fused |
| `/Sim/Mech/state` | double[8] | {flywheel rps, setpoint, feeder rps, intake rps, arm 0–1, climber L 0–1, climber R 0–1, atSpeed} |
| `/Sim/Fuel/state` | double[4] | {in robot, scored, missed, preload} |
| `/Sim/Fuel/shots` | double[] | {seq, fromX, fromY, toX, toY, result} × last 24; result 0 = scored, 1 aim, 2 close, 3 far, 4 slow |
| `/Sim/Fuel/field` | double[] | {x, y} of every FUEL on the floor (25 Hz) |
| `/Sim/Fuel/physicsMs` | double | Time the FUEL physics took this loop |
| `/Sim/Robot/intake` | double[2] | {depth in front of the bumper, width} m |
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
| `cam/<ll3\|ll4\|pv>/<setting>` | double | enabled (0/1), noise, latencyMs, dropPct, hFovDeg, maxRangeM, mountErrX, mountErrY, mountErrYawDeg |
| `fuel/<setting>` | double | preload, aimTolDeg, minDistM, maxDistM, capacity, intakeRate |
| `resetFuel` | double[] `[seq]` | Preload the robot and restage all 504 FUEL |
| `releaseChute` | double[] `[seq, red]` | That alliance's human player opens the CHUTE |
| `intake` | double[] `[seq, on]` | Robot code intake on/off (`RobotContainer.setIntake`, like the X button) |

One-shot actions carry a sequence number. The robot acts once each time it changes,
and the UI never re-sends an action after a reconnect.

## Developer scripts (`sim-ui/scripts/`)

Run each with the robot sim running:

- `node scripts/nt-smoke.ts`: lists every `/Sim` topic and its value.
- `node scripts/drift-test.ts [ideal|welltuned|worn] [1|2]`: runs a test drive and prints the drift.
- `node scripts/pause-test.ts`: checks that pause freezes, step advances exactly, and resume doesn't jump.
- `node scripts/play-test.ts [kind] [name] [red] [preset]`: plays an auto/path and prints tracking and localization error (e.g. `1 "Test path" 0 ideal` → localization 0.0 cm).
- `node scripts/preview-test.ts`: requests the exact preview of `Test path`.
- `node scripts/vision-test.ts`: kidnap with cameras on/off, LL4 aim, `Shoot All (Stationary)` scoring.
- `node scripts/pv-test.ts`: enables the PhotonVision cameras and checks that they correct a kidnap.
- `node scripts/field-fuel-test.ts`: FUEL staging (504), bumper push, intake pickup and capacity, chute release, scored FUEL coming back out of the hub, physics time.
- `npm run gen-field`: regenerates `field-2026.json` (UI and Java copies).

---

## Roadmap

| Phase | Delivers | Status |
|---|---|---|
| 1 Backbone | World model, drift, NT bridge, field, truth vs estimate, pause/step, bug fixes | ✅ |
| 2 Autonomy + Plan tab | AutoBuilder + named commands, PathPlanner-style editor, Play (20 s auto), plan/setpoint layers, error charts, timeline | ✅ |
| 3 Cameras | LL3 MegaTag2, LL4, PhotonVision sims, camera cones, correction dots, kidnap demo | ✅ |
| 4 Mechanisms | Shooter, feeder, fuel scoring, intake, climber; ShootAllAuto playable | ✅ |
| 5a Field FUEL | 504 FUEL with physics, intake pickup, hub return, human-player chutes | ✅ |
| 5 Polish | Robot collisions, bump impulses, run compare, narration, reports, exports, student exercises | next |
