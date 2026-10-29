# Changelog

Short reference. Full details: [docs/simulator.md](docs/simulator.md). Student guide: [docs/USER_GUIDE.md](docs/USER_GUIDE.md).

## 2026-10-29 — branch `coach`: speed markers (Plan tab)

- **Robot code: no changes.** UI only.
- **＋ Speed** in a path: a ▼/▲ marker that sets max velocity and acceleration from that point until the next speed marker or the path end. Saved as standard PathPlanner constraint zones (`Speed marker (N m/s)`), so the robot and the PathPlanner app use them unchanged; zones made in the app are kept.
- Checked with the exact PathPlannerLib preview (sim running): a 1.2 m/s marker caps the whole path at 1.20 m/s; a 1.0 m/s marker at waypoint 1 leaves 3.0 m/s before and 1.00 after; a later 2.5 m/s marker takes over; a 4.0 m/s marker goes above the 3 m/s path limit.

## 2026-10-29 — branch `coach`: Stop & wait marker (Plan tab)

- **Robot code: no changes.** UI only (`sim-ui/src/pathplanner/edit.ts`, `types.ts`, `tabs/PlanTab/`).
- **＋ Stop & wait** in a path's Event markers: a ⏸ marker with a variable wait (default 3 s), dragged along the path like an event marker.
- PathPlannerLib can't pause inside a path, so **Save splits the path** there (exact curve cut, parts end/start at rest, joined by a linked waypoint) and the autos that use it get `path → wait → path 2`; with no auto, one is created. Saving a path also moves its linked waypoints in other paths (like the PathPlanner app).
- Checked: scratch test of the split (parts trace the original within 1e-14 m; targets, markers, zones, links, auto rewrite), and a live run of a split `Test path` through the robot code: part 1 ends at ~1.8 s, the robot holds still 3 s, part 2 starts at ~5 s.

## 2026-10-28 — branch `coach`: review fixes (UNIT_TEST_RECOMMENDATIONS)

- **Real robot: no behavior change.** `shootFor` moved to `commands/ShootCommands.java` and pinned by `ShootCommandsTest` (feeds after 1.5 s even if the flywheel never reaches speed — kept, team decision). `RobotContainer.hasAutonomousOverride()` added (sim only).
- **Sim:** Play while an auto is running now really restarts (#2): new `disarming` state waits one robot loop disabled. A failed build, Stop or a stuck enable never leaves an auto queued (#7, `/Sim/Run/overrideActive`).
- **UI:** clicks made while disconnected are no longer replayed (#5); Rename/Duplicate/New refuse existing names ignoring case, case-only rename is safe on macOS, renaming a path offers to update its autos, Delete warns which autos use a path (#6); waypoint insert/delete shifts constraint and point-towards zones (#8); Save asks before overwriting a file the PathPlanner app changed (#9); unique temp files, leftovers removed at startup (#10); a click without a drag keeps redo (#11).
- **Tests:** 42 JUnit tests. New `scripts/run-control-test.ts` (#1 deadlock stress, #2, #7).
- Still open: #3 feeder stutter while holding Y (real robot), #4 stick mapping (check on blocks).

## 2026-10-28 — branch `coach`: phase 5a, FUEL on the field + intake pickup

- **Robot code: no changes.** Everything is in `frc.robot.sim` and `sim-ui`.
- **504 FUEL**, staged per Game Manual 6.3.4: NEUTRAL ZONE box (400 with 8 preloaded), 24 per DEPOT, 24 per OUTPOST CHUTE, robot preload.
- `FieldFuel.java`: 2D physics on the true pose (rolling, soft bounces, walls, HUB/TOWER/OUTPOST; BUMPs/TRENCHes passable); bumpers push balls; about 1–2.5 ms per loop.
- **Intake pickup** from the front bumper while the roller (CAN 15) spins, up to capacity 40 and 10 balls/s (placeholders, UI sliders).
- Shots stay in play: scored FUEL comes back out of a HUB exit after 1 s; misses land on the field. The total stays 504.
- UI: yellow FUEL layer, intake zone on the robot, **Intake on/off** button (calls the code's `setIntake`), Blue/Red **chute** buttons, new Fuel panel counts.
- Tests: `FieldFuelTest` (13 JUnit tests, 39 total), `scripts/field-fuel-test.ts`.
- sim-ui dependencies: a fresh `npm install` failed after the Vite 8 upgrade (`@vitejs/plugin-react` 4.3.4 only supports Vite 4–6). Now plugin-react **6.1.2** and `@types/node` **20.19.43**; clean install, build and `npm audit` (0 vulnerabilities) checked. `vite.config.ts` imports `deployFiles.ts` with its extension (Vite 8 warning).

## 2026-10-25 — branch `coach` (uncommitted): second code review fixes

- **Real robot (check on blocks):**
  - `AimController`: "aimed" also requires tx to be nearly still (≤ 0.15° change per loop), so an overshoot through the tolerance band doesn't fire.
  - `VisionSubsystem`: a heading jump over 30° in one loop (pose reset) is no longer read as spinning.
- **Simulator:**
  - TOWER tags (15/16/31/32) are visible to cameras. The TOWER is still a collision obstacle in the Plan tab.
  - PhotonVision mount error is applied in the robot frame.
  - Limelight 3 frames stay unique (latency jitter), so zero-noise frames aren't dropped.
  - The fuel flywheel check uses motor units.
  - The stale LL3 marker is cleared when disabled; camera settings merge per camera.
- **Tests:** 26 JUnit tests: new `CameraGeometryTest` (5) and `AimControllerTest.sweepingThroughTheBandIsNotAimed`.
- Review details: `UNIT_TEST_RECOMMENDATIONS.md` → "Second review".
- Student guide added: `docs/USER_GUIDE.md`.

## 2026-10-24 (later) — branch `coach` (uncommitted): simulator phases 3–4

### Robot code (affects the real robot — found by the simulator, check on the robot)
- **Aim stall fixed:** `AimController` uses a minimum turn rate of 0.4 rad/s while outside the 1.5° tolerance.
  - Before: near the target P asked for ~0.2 rad/s, below drivetrain friction, so the robot stopped 2–3° off and `ShootAllAuto` never fired.
- **MegaTag2 heading corruption fixed** (`VisionSubsystem`):
  - The LL3 yaw is no longer trusted (θ std dev 9999999); it was 0.8 rad and pulled the heading ~14° off while turning.
  - MegaTag2 frames are ignored while spinning > 360°/s.
  - `robot_orientation_set` now also sends the yaw rate.
- Read-only access for the sim: `VisionSubsystem` camera list, LL4 constants, `getPhotonCameras()`; drivetrain `addVisionMeasurementListener` (unused on the robot).

### Simulator
- Cameras:
  - `LimelightSim`: LL3 MegaTag2 and LL4 tx/ty, writing the real NT entries.
  - `PhotonSim`: 4 PhotonVision cameras via PhotonLib.
  - `CameraGeometry` + `FieldModel`: field of view, range, tag facing, Hub/Tower blocking.
  - Frames use the true pose at capture time (`WorldModel.truthAt`).
- Mechanisms:
  - `MechanismSims`: flywheel, feeder, intake, arm, climbers on the simulated TalonFX controllers.
  - `FuelSim`: preload, scored if aimed and in range, miss reasons.
- UI:
  - camera cones, sightlines, fused-measurement dots, LL4 view, per-camera controls (noise, latency, drops, mount error)
  - fuel and mechanism panels, shot lines, fuel in the run summary
- Fixes:
  - Review finding #1: deadlock between the odometry thread and Place/Kidnap/Play.
  - Camera frames are skipped for 0.15 s after a teleport, so frames never straddle the move.

### Assumptions (new)
- Camera specs:
  - LL3: 62.5×48.9°, 6 m, 30 ms
  - LL4: 82×56.2°, 7 m, 20 ms
  - PhotonVision: 1280×800, 100° diag, 30 fps
- Camera noise grows with distance² (LL3 ≈ 13 cm σ at 4 m, one tag).
- The shooter fires out of the robot's front. Each ball costs ~8% of flywheel speed. One ball per 0.25 s while feeding.
- Fuel scores at ±3°, 1.5–5.5 m, flywheel ≥ 90%. No ball flight and no pickup.
- Arm, climber and intake gearing and travel are placeholders.

### Things to know
- With Ideal drift and cameras on, the estimate wanders ~1–3 cm (camera noise). That's expected.
- MegaTag2 can't fix a heading error. A 2° heading placement error leaves ~9–13 cm of position error near the tags.
- **With Well-tuned drift, localization peaks are higher with cameras than without** (~30 cm vs ~14 cm), from MegaTag2 while turning.
  - `VisionSubsystem` trusts every LL3 frame equally (0.5 m).
  - Scaling trust with distance and speed is a good student tuning task.
- Review finding #2 is still open: Play while an auto is running doesn't restart it. Press Stop first.
- Tests: `./gradlew build` runs 20 JUnit tests. New live checks: `sim-ui/scripts/vision-test.ts`, `pv-test.ts`.

## 2026-10-24 — branch `autonomy-sim` (uncommitted): simulator phases 1–2 + bug fixes

### Robot code (affects the real robot)
- **Auto-aim fixed.**
  - Before: kP was applied to tx in radians, so the robot never turned, and the sign was inverted.
  - Now: shared `commands/AimController`, kP = 4. **Retune on the robot.**
- **Shooter** split into `spinUp()` / `feed()` / `isAtSpeed()`. Teleop Y and `ShootAllAuto` feed only once at speed (and, in auto, aimed).
- **Stick mapping** is the same for drive and auto-aim: X = −LeftY, Y = −LeftX, rot = −RightX.
- **Climber direction toggle**: D-pad-center + X → **right-stick click**. X no longer flips it.
- **Y is blocked** while Back or Start is held (SysId combos).
- **Limelight**:
  - LL3 tag count is read from `bp[7]` (was `bp[6]`, the latency).
  - Each frame is used once.
  - "Online" means the heartbeat is still changing.
- **PathPlanner** wired up:
  - `configureAutoBuilder()`: P 10 / 7, red paths flipped with rotational symmetry, AndyMark field size.
  - New `deploy/pathplanner/settings.json`.
  - Named commands registered.
  - `.auto` files appear in the chooser as "PP: name".
- `CommandSwerveDrivetrain`:
  - `registerTelemetry` override that allows extra listeners (none on the robot).
  - `startSimThread()` uses WPILib time with a 20 ms clamp (sim only).

### Simulator (new, sim only)
- `frc.robot.sim`:
  - WorldModel (truth), DriftModel, SimClock (pause/step)
  - SimControl (Play/Reset, 20 s auto)
  - PathLogging, PlanPreview, SimNT
- `sim-ui/` web app:
  - **Run** tab: plan / setpoint / estimate / truth, error charts, replay.
  - **Plan** tab: PathPlanner-style editor of the real path files.
- `sim.sh` launcher; Gradle `-Pheadless` option.
- Field model from WPILib AprilTags + 2026 Game Manual (`sim-ui/scripts/gen-field.mjs`).

### Assumptions
- **Robot measurements are placeholders (TO MEASURE)**:
  - frame 27×24 in, bumpers 3.25 in
  - mass 52 kg, MOI 5.5 kg·m², wheel COF 1.2
  - Kraken X60, 60 A
  - LL3/LL4 mounts
  - shooter 1:1, MOI 0.004
- Depot position along the wall is approximate (the manual gives no location).
- Drift defaults ("Well-tuned") are educated guesses: wheel scale 1.5 %, slip 1 cm/√m, gyro bias 0.3°/min, gyro scale 0.2 %.
- AUTO = 20 s, 8 FUEL preload, field symmetry rotational (2026 manual).
- `Shoot` steps feed after 1.5 s even if the flywheel isn't at speed, so the robot still empties.
- If the real robot drives sideways with the new stick mapping, the swerve encoder offsets are wrong: fix them in Tuner X, not in code.

### Things to know
- **Start:** `./sim.sh` (or `--headless`) → http://localhost:5173. Needs Java 17 (any WPILib year's JDK) and Node 20+.
- **"Port in use"** means a sim is already running. Stop it first.
- **Camera and joystick warnings** in the sim are harmless. Camera sims come in phase 3.
- **The flywheel isn't simulated yet** (phase 4): `Shoot` waits 1.5 s, then feeds.
- **The Plan tab writes the real PathPlanner files.**
  - Backups go to `sim-ui/.sim-backup/`.
  - Saving only changes formatting (`180.0` → `180`).
- **Event markers must not use the drivetrain.** `AutoAim` / `AimAndShoot` are auto steps only.
- **Tests:** `./gradlew build` runs 18 JUnit tests. Live checks are in `sim-ui/scripts/`.
- **Code review done for phases 1–2.** Results and recommended tests are in `UNIT_TEST_RECOMMENDATIONS.md`.
