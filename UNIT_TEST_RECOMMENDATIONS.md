# Unit test recommendations (from the code review)

Source: an independent code review of branch `autonomy-sim` (2026-10-24). Each main finding was then checked against the code and the library source. **Nothing has been fixed yet.**

Current coverage:
- JUnit, 18 tests: `AimControllerTest` and `DriftModelTest`, all passing.
- `sim-ui`: no unit tests. Live checks only, in `sim-ui/scripts/`.

## Tooling to add
- **Java (sim classes):** JUnit 5 is already set up. Call `HAL.initialize(500, 0)` in `@BeforeAll` to use `DriverStationSim` / `SimHooks` without the GUI.
- **sim-ui:** add `vitest` as a dev dependency and `npm test`. Most targets are pure functions in `src/pathplanner/` and `src/nt/`.
- **Testability refactors, kept small:**
  - pass state into `WorldModel.setTruth(...)`
  - extract a pure "feed latch" from `Shooter`
  - extract the undo/history logic in `PlanTab` into a small reducer

## Findings → recommended tests

### 1. HIGH (sim) — the sim can deadlock
- **Problem:** `WorldModel.setTruth` (synchronized) calls `drivetrain.getStateCopy()`, which takes CTRE's state lock. Meanwhile CTRE calls `WorldModel.onSample` while already holding that lock. The two threads can wait on each other forever. `SimManager` syncTruth has the same problem.
- **Unit test:** read the state *before* taking the world lock (`setTruth(Pose2d, SwerveDriveState)`). Then a JUnit test hammers `setTruth` from one thread while another thread calls `onSample` with a fake state. Assert it finishes within a timeout (`assertTimeoutPreemptively`).
- **Integration test:** a `sim-ui/scripts` stress script that sends `placeRobot` 200× while a path runs. Assert `/Sim/Heartbeat` keeps increasing.

### 2. MED-HIGH (sim) — Play during a running auto doesn't start the new one
- **Problem:** disable + enable in the same robot loop is never seen, because DS data refreshes once per loop. `autonomousInit` doesn't re-run, and the state jumps ARMED → RUNNING.
- **Unit test (JUnit + HAL):** drive `SimControl` through two consecutive plays, refreshing DS between loops. Assert:
  - it passes through a disarming state,
  - the second command gets scheduled,
  - `commandEnd` stays −1 until that command really ends.
- **Integration test:** `play-test.ts` started twice back-to-back. The second run's setpoint must follow the second path.

### 3. MEDIUM (real robot) — holding Y can make the feeder stutter
- **Problem:** `Shooter.runShooter()` re-checks the 5 % at-speed tolerance every loop, so a speed dip from a ball stops the feeder mid-shot.
- **Unit test:** extract a feed latch (pure class). Assert:
  - not at speed → spin up only
  - first time at speed → feed
  - dip below tolerance → **keep feeding**
  - release → reset

### 4. MEDIUM (real robot, behavior change) — stick mapping
- No unit test. **Check on blocks before a match.** If the robot drives sideways, fix the encoder offsets in Tuner X.
- Optional: a test that `driverVelocityX/Y/RotationRate` follow X = −LeftY, Y = −LeftX, rot = −RightX, using `XboxControllerSim`.

### 5. MEDIUM (sim UI) — clicks made while disconnected are replayed on reconnect
- **Problem:** `nt4.ts` queues one-shot actions while disconnected and sends them after the next reconnect. This breaks the "never re-sent" rule.
- **Unit test (vitest, mock WebSocket):**
  - action while disconnected → reconnect → assert **not** sent
  - retained setting while disconnected → reconnect → assert sent once
  - sent timestamps strictly increase

### 6. MEDIUM (UI, data loss) — Rename and Duplicate can overwrite files
- **Problem:** there is no check for an existing name. A case-only rename on macOS deletes the file. Rename leaves autos pointing at the old path name.
- **Unit test:** a pure name-check helper. Assert:
  - case-insensitive collisions are rejected
  - a case-only rename is allowed only through a temp name
  - a rename reports the autos that reference the old name

### 7. LOW-MED (sim) — a stale autonomous override can stay set
- **Problem:** if the build fails, or ARMED never becomes RUNNING, the previous command stays queued for the next GUI auto enable.
- **Unit test (JUnit + HAL):**
  - a failed build clears the override
  - ARMED times out and clears it
  - `stop()` clears it

### 8. LOW (UI) — waypoint insert/delete doesn't shift zones
- **Problem:** `edit.ts` `insertWaypoint` / `deleteWaypoint` shift rotation targets and markers, but not `constraintZones` / `pointTowardsZones` (`min/maxWaypointRelativePos`).
- **Unit test (vitest):** insert or delete on a path with zones; assert zone positions shift the same way markers do.

### 9. LOW (UI, needs decision) — saving can overwrite PathPlanner-app edits
- **Problem:** there is no file-changed check on save.
- **Unit test:** a save guard compares the mtime from open with the current mtime and warns on mismatch.

### 10. LOW (UI) — concurrent saves share a temp file name
- **Problem:** `deployFiles.ts` uses `${file}.tmp-${pid}` inside the deploy folder. A leftover temp file would get deployed to the roboRIO.
- **Unit test (vitest, temp dir):**
  - two simultaneous PUTs of one file both succeed
  - the result is valid JSON
  - no `.tmp` files are left in the deploy tree

### 11. LOW (UI) — a click without a drag clears redo
- **Problem:** pointer-down always pushes an undo entry.
- **Unit test:** history reducer — down + up with no move → no history entry, and redo is kept.

### 12. Behavior choice (real robot, needs decision)
- **Behavior:** `shootFor()` feeds after 1.5 s even if the flywheel isn't at speed.
- **Test:** whichever way you choose, add a test that pins it (`shootFor` with a never-at-speed shooter feeds, or doesn't, after 1.5 s).

## Coverage gaps worth filling (no bug found)
- **`geometry.ts`:**
  - `pointAt` at the ends and segment joins
  - `rotationAt` wrap-around (179° → −179°)
  - red mirroring is a 180° rotation about the field center
- **`collision.ts`:** a pose overlapping the Hub is flagged, a Bump is not, leaving the field is flagged.
- **PathPlanner round-trip:**
  - parse → save of `Test path.path` keeps every value
  - unknown fields survive an edit
- **`deployFiles.ts` path safety:** `..`, absolute paths, wrong extensions and `/` in names are rejected.
- **Java:** `SimNT.action` acts once per sequence number and ignores the initial empty or zero value.

## Checked and ruled out (no test needed for these concerns)
- **Named commands reused across autos:** PathPlanner wraps each use.
- **Limelight timestamp math:** matches LimelightHelpers.
- **Telemetry override on the robot:** safe.
- **`settings.json`:** keys match `RobotConfig.fromGUISettings`.
- **`AimController` sign:** correct.
- **Null sim object on the real robot:** safe.
- **NT message encoding:** float64 doubles, correct RTT math.
- **`PlanPreview` reflection target:** exists in PathPlannerLib 2026.1.2.
