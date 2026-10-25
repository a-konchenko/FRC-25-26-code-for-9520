# Changelog

Short reference. Full details: [docs/simulator.md](docs/simulator.md).

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
- **Not yet independently code-reviewed.**
