# Moving the simulator to new robot code: code change plan

**Short answer: it is not plug and play, but most of it carries over.**

The web UI (`sim-ui/`) and about two-thirds of the Java sim (`frc.robot.sim`) don't
care what the robot code looks like. The rest reaches into the robot code in about
**25 specific places**: class names, method names, CAN IDs, camera names and
constants. Each of those has to exist in the new code, or be pointed at its new
equivalent.

How much work it is depends on what "new code" means:

| New code is… | Rough effort | Why |
|---|---|---|
| **A rewrite for the same game (2026), same libraries** (CTRE Tuner X swerve, PathPlannerLib, Limelight + PhotonVision) | **1–2 days** | Copy the sim, rebuild the ~25 hooks below. The UI needs no changes. |
| **Next season's code (new game), same libraries** | **1–2 weeks** | Everything above, plus a new field, game piece and scoring layer (the 2026 FUEL/HUB parts are game-specific). |
| **Different swerve library** (YAGSL, AdvantageKit/IO layers, REV) | **+3–5 days** on top | The truth/estimate model reads CTRE's 250 Hz odometry samples and uses CTRE's swerve sim. It would need a new source for wheel and gyro samples. |

These are estimates for one programmer who knows the code. Step 1 below makes all
later moves faster: do it in the current code first, while everything still works.

---

## What carries over, and what doesn't

### Carries over unchanged
- **The web UI** (`sim-ui/`). It only talks to the robot through the `/Sim/…`
  NetworkTables contract in `docs/simulator.md`. As long as the Java side publishes
  the same topics, the UI works.
  - The Plan tab edits standard PathPlanner files, so it works with any project.
  - Only the robot project folder path is hard-coded (see step 3).
- **Sim classes that don't touch robot code:**
  - `SimClock`: pause and step.
  - `SimNT`: `/Sim` publishing, settings, one-shot actions.
  - `DriftModel`: wheel and gyro error.
  - `CameraGeometry`: field of view, range, blocking.
  - `PathLogging`: plan, setpoint and active path.
  - `PlanPreview`: exact PathPlannerLib timing.
- **Developer scripts** in `sim-ui/scripts/`. They talk to NT only, so they become
  the acceptance tests for the port (step 7).

### Game-specific (2026 REBUILT): redo for a new game
- `FieldModel` + `field-2026.json` + `scripts/gen-field.mjs`: field elements, HUB
  opening, AprilTag layout enum `k2026RebuiltAndymark` / `k2026RebuiltWelded`.
- `FieldFuel` (504 balls, staging, chutes), `FuelSim` (HUB scoring rule), the HUB
  targeting in `LimelightSim` (LL4 aims at the HUB).
- `SimControl.kAutoDurationS` (20 s), `fuelPreloadMax`, rotational field symmetry.
- UI: `FuelLayer`, the Fuel panel in `MechPanel`, field drawing of game elements.

### Robot-coupled: must be matched in the new code
Everything in the next table.

---

## The hooks: what the sim needs from the robot code

Each row is something the sim calls or reads today. In the new code, either keep the
same name, or change the one sim line that uses it. Row numbers are used in the steps
below.

### A. Wiring (always needed)

| # | Sim needs | Today | Used by |
|---|---|---|---|
| A1 | Create the sim only in simulation, call it every loop | `Robot.java`: `RobotBase.isSimulation() ? new SimManager(container) : null`, `simulationPeriodic()` → `update()` | everything |
| A2 | Access to the drivetrain, vision and shooter | public fields `container.drivetrain`, `container.vision`, `container.shooter` | `SimManager` |
| A3 | Hand the sim's auto to the robot code | `RobotContainer.setAutonomousOverride(cmd)` / `hasAutonomousOverride()`; `getAutonomousCommand()` returns the override first | `SimControl` (Play) |
| A4 | List the hand-written autos and named commands | `getHandCodedAutos()` (name → supplier), `getNamedCommandNames()` | `SimControl` (Run tab lists) |
| A5 | Turn the intake on/off from the UI | `RobotContainer.setIntake(boolean)` | `SimManager` |
| A6 | Headless sim | `build.gradle`: `wpi.sim.addGui().defaultEnabled = !project.hasProperty('headless')` | `sim.sh --headless` |

### B. Drivetrain (CTRE Phoenix 6 `SwerveDrivetrain`)

| # | Sim needs | Today | Notes |
|---|---|---|---|
| B1 | **Sim thread on WPILib time, 20 ms clamp** | `CommandSwerveDrivetrain.startSimThread()` uses `Timer.getFPGATimestamp()` and clamps each step | **Critical.** Without it, pause/step breaks and the robot jumps after a pause. Copy this method exactly. |
| B2 | Every odometry sample (250 Hz) | `registerTelemetry` override + `addTelemetryListener(...)` | `WorldModel` builds the Truth from these. |
| B3 | Every fused vision measurement | `addVisionMeasurementListener(...)` (called from `addVisionMeasurement`) | Run tab "fused" dots. |
| B4 | Pose, reset, kinematics, module locations | `getState().Pose`, `getStateCopy()`, `resetPose`, `getKinematics()`, `getModuleLocations()` | Standard CTRE, so no change if the drivetrain is still CTRE. |
| B5 | Drive requests for the UI test drive | `applyRequest(...)` with `SwerveRequest.FieldCentric` etc. | Standard CTRE. |
| B6 | Max speed | `TunerConstants.kSpeedAt12Volts`, `TunerConstants.kCANBus` | Regenerated by Tuner X; same names unless Tuner X changes them. |
| B7 | PathPlanner configured on the drivetrain | `configureAutoBuilder()` (AutoBuilder, rotational flipping, AndyMark field size) | Needed for Play and PlanPreview. |

**Locking rule (keep it):** CTRE calls the telemetry listener while holding its state
lock. Never call `getState()` / `getStateCopy()` while holding `world.lock()`. Read the
state first, then lock (`WorldModel`, `SimManager.placeRobot`).

### C. Vision

| # | Sim needs | Today | Used by |
|---|---|---|---|
| C1 | Limelight NT table names | `"limelight-three"` (MegaTag2 localization), `"limelight-four"` (targeting) | `LimelightSim` writes the real Limelight entries. The names must match what the new vision code reads. |
| C2 | LL4 mount and target constants | `VisionSubsystem.LL4_HEIGHT_M`, `LL4_PITCH_DEG`, `HUB_HEIGHT_M` (public) | `LimelightSim` |
| C3 | PhotonVision cameras + mounts | `VisionSubsystem.PHOTON_CAMERAS` (`CamConfig`: name, robot-to-camera), `getPhotonCameras()` | `PhotonSim` (PhotonLib `VisionSystemSim`) |
| C4 | AprilTag layout | `AprilTagFields.k2026RebuiltAndymark` in VisionSubsystem, CommandSwerveDrivetrain and FieldModel | Must be the same layout everywhere. |
| C5 | Which fields the code reads | Both: heartbeat `hb`, `pipeline`. LL3: `botpose_orb_wpiblue` (tag count at index 7), reads back `robot_orientation_set`. LL4: `tx`, `ty`, `ta`, `tv`. The sim also writes `tl`, `cl`, `getpipe` | If the new code uses `LimelightHelpers` with other entries, `LimelightSim` must write those. |

### D. Mechanisms

| # | Sim needs | Today | Used by |
|---|---|---|---|
| D1 | CAN IDs of every non-drive TalonFX | shooter `Shooter.kShooterMotorId` (13), feeder `kFeedMotorId` (14), intake 15, arm 19, climbers 16/17 (hard-coded in `MechanismSims`) | `MechanismSims` makes extra `TalonFX` objects on the same IDs to drive their sim state. **Wrong ID = mechanism never moves in the sim.** |
| D2 | CAN bus | `TunerConstants.kCANBus` for shooter/feeder, default bus for the rest | `MechanismSims` |
| D3 | Shooter setpoint and speed | `Shooter.getShooterSetpointRPS()` (motor rotations/s) | `FuelSim` "flywheel ≥ 90 %" |
| D4 | Gearing / inertia / travel | `SimRobotConstants` (placeholders, TO MEASURE) | `MechanismSims` |
| D5 | Non-TalonFX motors | none today | A SparkMax/Kraken-on-REV mechanism needs its own sim class (REV `SparkMaxSim`). |

### E. Autos and PathPlanner

| # | Sim needs | Today | Notes |
|---|---|---|---|
| E1 | Named commands registered before autos are built | `registerNamedCommands()` in `RobotContainer` | The Plan tab lists `/Sim/NamedCommands` from A4. |
| E2 | PathPlanner file version | `.path` / `.auto` version `2025.0`, PathPlannerLib 2026.1.2 | If PathPlanner's file format changes, update `sim-ui/src/pathplanner/types.ts`. Unknown fields are kept, so small changes are safe. |
| E3 | `deploy/pathplanner/settings.json` | robot config for `RobotConfig.fromGUISettings` | Needed by AutoBuilder and PlanPreview. |

### F. Repo layout

| # | Hard-coded | Where |
|---|---|---|
| F1 | Robot project folder `Swerve9520 ctre tuner x project (og home base ver.)` | `sim.sh` (`ROBOT_DIR`), `sim-ui/server/deployFiles.ts` (overridable with `ROBOT_PROJECT_DIR`), `sim-ui/scripts/gen-field.mjs` (output path), `FRC-9520.code-workspace` |
| F2 | `deploy/sim/field-2026.json` | read by `FieldModel`; written by `npm run gen-field` |

---

## Plan

Each step ends with something that can be checked. Per `CLAUDE.md`, agree on each
step before coding it and stop for review after.

### Step 1: put an adapter between the sim and the robot code (in the current code, first)

**Goal:** all the hooks above go through **one** file, so a future move means
rewriting that file and nothing else. This is what makes the sim close to plug and play.

- New interface `frc.robot.sim.SimRobot` (sim package, never runs on the robot) with
  what the sim needs, for example:
  - `CommandSwerveDrivetrain drivetrain()` (or a narrower `SimDrive` interface
    for B2–B5),
  - `void setAutonomousOverride(Command c)`, `boolean hasAutonomousOverride()`,
  - `Map<String, Supplier<Command>> handCodedAutos()`, `Collection<String> namedCommands()`,
  - `void setIntake(boolean on)`,
  - `double shooterSetpointRps()`,
  - `List<CamConfig> photonCameras()`, `String ll3Table()`, `String ll4Table()`, LL4 constants,
  - `MechanismIds mechanismIds()` (shooter, feeder, intake, arm, climbers, bus).
- New `frc.robot.sim.RobotAdapter2026 implements SimRobot`, which wraps today's
  `RobotContainer`. All `container.*`, `VisionSubsystem.*`, `Shooter.*` and
  `TunerConstants.*` uses in `frc.robot.sim` move into it.
- `SimManager`, `SimControl`, `FuelSim`, `MechanismSims`, `LimelightSim` and `PhotonSim`
  take a `SimRobot` instead of the concrete classes.
- Move the CAN IDs out of `MechanismSims` (D1) into the adapter. Read them from the
  robot code's constants rather than repeating the numbers.
- `Robot.java` becomes `new SimManager(new RobotAdapter2026(container))`.
- **Robot code changes:** none in behavior. The one robot-side line is in `Robot.java`.
  Record it in the "Robot code changes" table.
- **Check:** `./gradlew build` (all JUnit tests) and every script in step 7 pass,
  same results as before.

### Step 2: game layer split (only if the new code is for a new game)

**Goal:** the 2026-only parts live behind one switch, so a new game adds files
instead of editing the core.

- Group `FieldFuel`, `FuelSim`, the HUB part of `LimelightSim`, the hub/fuel parts of
  `FieldModel` and the field JSON under a `Game2026` class (or package), with a small
  `SimGame` interface: `reset(seed)`, `update(dt, truth)`, `publish()`, `fieldFile()`,
  `tagLayouts()`, `autoDurationS()`.
- UI: the Fuel panel, `FuelLayer` and game-element drawing are shown only when the
  robot publishes `/Sim/Game/name = "2026"`.
- **Check:** same as step 1.

### Step 3: copy the sim into the new project

1. Copy `sim-ui/`, `sim.sh`, `FRC-9520.code-workspace`, `docs/simulator.md`,
   `docs/USER_GUIDE.md` to the new repo.
2. Copy `src/main/java/frc/robot/sim/` and its tests (`src/test/java/frc/robot/sim/`).
3. Copy `src/main/deploy/sim/` (field JSON) and keep `deploy/pathplanner/`.
4. Update the project folder name (F1) in `sim.sh`, `deployFiles.ts`, `gen-field.mjs`
   and the workspace file. Better: make all of them read `ROBOT_PROJECT_DIR`, with
   the folder name in one place in `sim.sh`.
5. `build.gradle`: add the `-Pheadless` line (A6).
6. Check that the new `build.gradle` has the same vendordeps (Phoenix 6, PathPlannerLib,
   PhotonLib) and Jackson (used by `FieldModel`).
- **Check:** `./gradlew build` compiles (the adapter will fail at first; that's step 4).

### Step 4: write the adapter for the new code

Go through tables A–E row by row:

- **A1:** add the two lines to the new `Robot.java`.
- **A2–A5:** add the four small methods to the new `RobotContainer` (or wherever
  autos and subsystems live). `getAutonomousCommand()` must return the sim's
  override first. This is the only robot-logic change, and it does nothing on the
  robot because nothing sets an override there.
- **B1–B3:** copy `startSimThread()`, the `registerTelemetry` override with
  `addTelemetryListener`, and `addVisionMeasurementListener` into the new drivetrain
  class. If the new drivetrain is regenerated from Tuner X, re-apply these three after
  every regeneration (write it in the drivetrain file's header).
- **B7, E1, E3:** make sure AutoBuilder is configured and named commands are
  registered before any auto is built.
- **C1–C5:** check the Limelight table names and which NT entries the new vision code
  reads. Make the camera constants public (read-only).
- **D1–D3:** fill in the CAN IDs and the shooter setpoint getter. Any motor that isn't a
  TalonFX needs its own sim (D5).
- **Check:** `./gradlew simulateJava` starts and the console prints `[Sim] SimManager
  ready`. Then `node sim-ui/scripts/nt-smoke.ts` lists the `/Sim` topics.

### Step 5: drivetrain and PathPlanner checks

- **Check (in order):**
  1. `pause-test.ts`: pause/step works and the robot doesn't jump (B1).
  2. `drift-test.ts ideal`: with zero drift, Truth equals Estimate (B2).
  3. `play-test.ts 1 "<a path>"`: tracking error under ~15 cm.
  4. `preview-test.ts`: exact preview matches the run.
  5. `run-control-test.ts`: Play twice, a failed build, Stop, and 300 teleports with no freeze (A3, locking).

### Step 6: cameras and mechanisms

- **Check:**
  1. `vision-test.ts`: the kidnap correction works, LL4 aims, ShootAllAuto (or the new
     shoot auto) scores.
  2. `pv-test.ts`: the PhotonVision cameras see tags.
  3. Run tab: the flywheel, feeder, intake, arm and climber gauges move when the
     buttons/commands run (D1).
  4. `field-fuel-test.ts` (2026 game only).

### Step 7: acceptance and docs

- All JUnit tests + all scripts above pass.
- Update `CLAUDE.md` (paths, class names), `docs/simulator.md` ("Blocks created" and
  "Robot code changes" tables) and `docs/USER_GUIDE.md` (screenshots, named commands).
- Re-measure the placeholders in `SimRobotConstants` (frame, mass, MOI, camera mounts,
  gearing) for the new robot.

---

## Risks and things that break quietly

- **Forgetting B1** (sim thread time). Everything seems to work until you press
  Pause, then the robot teleports. Test with `pause-test.ts` first.
- **Wrong CAN IDs (D1).** No error; the mechanism just never moves in the sim.
- **Limelight names or entries (C1, C5).** No error; the vision code sees "offline" or
  no tags. The Run tab camera cards show it.
- **Tuner X regeneration** overwrites `CommandSwerveDrivetrain` edits if it regenerates
  that file. Keep the three hooks (B1–B3) clearly marked.
- **New WPILib / Phoenix / PathPlannerLib year.** API renames (e.g. CTRE swerve
  telemetry, `FlippingUtil`, PathPlanner file version) are the most likely compile errors.
  Fix them in the adapter and the drivetrain hooks.
- **Different AprilTag layout in different places (C4).** The sim and the robot code
  must load the same layout, or localization is off by a few centimeters.
- **Locking rule.** Calling `drivetrain.getState()` inside `world.lock()` freezes the
  sim (deadlock). `run-control-test.ts` part 4 catches it.

## Recommendation

Do **step 1 now**, in the current code (about a day, no behavior change, fully tested
by the existing scripts). After that, moving to new code is: copy the folders, write
one adapter file, add the drivetrain hooks, and run the scripts.
