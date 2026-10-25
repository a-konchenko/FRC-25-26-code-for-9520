# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout

Two projects live side by side:

- **`Swerve9520 ctre tuner x project (og home base ver.)/`** — the FRC Team 9520 robot code (WPILib 2026 / GradleRIO 2026.2.1, Java 17, CTRE Phoenix 6 swerve, PathPlanner, Limelight/PhotonLib). The directory name has spaces and parentheses: always quote it in shell commands.
- **`sim-ui/`** — web UI for the autonomy simulator (Vite + React 18 + TypeScript), talking NetworkTables 4 to the robot code running under `simulateJava`.

`docs/simulator.md` is the authoritative simulator spec: architecture, drift model, the full `/Sim` NetworkTables contract, field data sources, placeholder constants, and the phased roadmap. Read it before changing simulator code, and keep it updated (the "Blocks created for the simulator" and "Robot code changes" tables, roadmap status) when you change behavior.

## Commands

Robot code (run from the robot project directory):

```bash
./gradlew build                     # compile + run unit tests
./gradlew test                      # JUnit 5 tests in src/test/java
./gradlew test --tests 'frc.robot.sim.DriftModelTest'                  # one test class
./gradlew test --tests 'frc.robot.commands.AimControllerTest.rateIsClampedToMax'  # one test
./gradlew simulateJava              # run robot code in sim with WPILib GUI
./gradlew simulateJava -Pheadless   # same, no WPILib GUI window
./gradlew deploy                    # deploy to roboRIO (team from .wpilib/wpilib_preferences.json or -PteamNumber=9520)
```

Gradle needs a Java 17 JDK. There is no system Java on the dev machine; `sim.sh` finds the JDK bundled with a WPILib install (`~/wpilib/*/jdk`) or uses `JAVA_HOME`. Ask before installing anything system-wide.

Simulator (from repo root):

```bash
./sim.sh              # robot sim + web UI at http://localhost:5173 (Ctrl+C stops both)
./sim.sh --headless   # without the WPILib GUI
```

Web UI (from `sim-ui/`):

```bash
npm run dev           # Vite dev server, port 5173 (strictPort)
npm run typecheck     # tsc -b
npm run build         # typecheck + production build
npm run gen-field     # regenerate field-2026.json (writes BOTH sim-ui/src/field/ and robot src/main/deploy/sim/)
```

Developer scripts in `sim-ui/scripts/` run directly with `node` (TypeScript, so a Node version with type stripping) and require the robot sim to be running: `nt-smoke.ts` (dump all `/Sim` topics), `drift-test.ts [ideal|welltuned|worn] [1|2]`, `pause-test.ts`, `play-test.ts [kind] [name] [red] [preset]` (play an auto/path, report tracking/localization error), `preview-test.ts` (exact PathPlannerLib preview), `vision-test.ts` (kidnap correction, LL4 aim, ShootAllAuto scoring), `pv-test.ts` (PhotonVision sim). These are the closest thing to integration tests.

## Architecture

### Robot code (`frc.robot`)

Standard command-based structure: `Robot` (TimedRobot) → `RobotContainer` (subsystems, Xbox bindings, auto chooser, PathPlanner `NamedCommands`) → `subsystems/` and `commands/`.

- `generated/TunerConstants.java` is exported from CTRE Tuner X (`tuner-project.json`). Do not hand-edit; regenerate in Tuner X.
- `CommandSwerveDrivetrain` owns the CTRE swerve drivetrain and pose estimator. `drivetrain.getState().Pose` is the robot's own pose estimate.
- `commands/AimController` is the shared aiming controller used by both `AutoAimCommand` and `auto/ShootAllAuto`. Change aiming there, not in each command.
- `Shooter` separates `spinUp()` / `feed()` / `isAtSpeed()`; callers feed only once at speed (and, in auto, aimed).
- Driver stick mapping is standardized: X = −LeftY, Y = −LeftX, rot = −RightX, used by both default drive and auto-aim. If the robot drives sideways, fix the swerve encoder offsets in Tuner X rather than swapping axes.
- `MaxSpeed` in `RobotContainer` is scaled to 0.75× for practice.

### Simulator layer (`frc.robot.sim`, sim only)

`Robot` constructs `SimManager` only when `RobotBase.isSimulation()` and calls `update()` from `simulationPeriodic()`. Nothing in `frc.robot.sim` may run on the real roboRIO, and robot logic should stay unchanged for the sim's sake: the sim wraps the real code rather than forking it.

- **Truth vs Estimate**: CTRE's swerve sim spins simulated motors; the robot's odometry produces the Estimate. `WorldModel` reads the same encoder/gyro samples (250 Hz) through `DriftModel` to produce the Truth. With drift at zero, Truth equals Estimate exactly. Future camera sims (phase 3) observe from Truth.
- `SimClock` implements pause/step. The one sim-only edit in shared code is `CommandSwerveDrivetrain.startSimThread()`, which uses `Timer.getFPGATimestamp()` (stops while paused) and clamps each step to 20 ms. Keep that, or the robot jumps after a pause.
- `SimNT` publishes everything under `/Sim`; `SimControl` handles UI commands under `/Sim/Control/…`. One-shot actions are `double[]` with a leading sequence number: the robot acts once per change, and the UI never re-sends after reconnect. Follow this pattern for new actions.
- `SimRobotConstants` holds physical placeholders (frame size, mass, MOI, camera mounts) still marked TO MEASURE.
- **Locking:** CTRE calls `WorldModel.onSample` while holding its own state lock. Never call `drivetrain.getState()` / `getStateCopy()` while holding `world.lock()`. Read the state first, then lock.
- Cameras: `LimelightSim` writes the real Limelight NT entries (LL3 MegaTag2 uses the heading the robot code sends). `PhotonSim` wraps PhotonLib's simulator. Both observe `WorldModel.truthAt(now − latency)`, and both must skip frames briefly after a teleport (`SimManager.teleported`).
- `MechanismSims` drives the non-drive TalonFX sim states (it creates extra `TalonFX` objects for the same CAN IDs). `FuelSim` implements the "scored if aimed" rule.

### Web UI (`sim-ui/src`)

- `nt/nt4.ts` is a hand-written NT4 client (msgpack over WebSocket to port 5810); `nt/store.ts` exposes topic values to React. `?robot=<ip>` points the UI at another machine.
- `tabs/RunTab`: Play autos/paths, plan/setpoint/estimate/truth layers, run recording + scrubbing, drift and placement-error panels. `tabs/PlanTab`: PathPlanner-style editor for the robot project's real `deploy/pathplanner` files.
- `pathplanner/` holds the PathPlanner file types, Bézier geometry, edits, mirroring and footprint collisions. `server/deployFiles.ts` is a Vite dev-server plugin that reads and writes those files (backups go to `sim-ui/.sim-backup/`, never into the deploy folder). Edits must keep unknown JSON fields, so the PathPlanner app's files round-trip.
- `field/field-2026.json` is generated by `scripts/gen-field.mjs` from WPILib AprilTag anchors plus 2026 Game Manual dimensions. Edit the generator, not the JSON. AprilTag poses themselves come from the robot sim over NT, so the UI matches what the code loads.

Coordinates everywhere use the WPILib blue origin in meters (+X toward red, +Y left from blue, CCW angles); NT poses are `double[] {x, y, θ rad}`.

## Working style

The simulator is built in phases (see the roadmap in `docs/simulator.md`). Agree on a plan before coding each phase, and stop for review when a phase is done. Changes that affect the real robot (not just `frc.robot.sim`) must be called out explicitly and recorded in the "Robot code changes" table in `docs/simulator.md`.
