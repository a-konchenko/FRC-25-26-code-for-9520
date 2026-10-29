# Unit test recommendations (from the code review)

Source: an independent code review of branch `autonomy-sim` (2026-10-24). Each main finding was then checked against the code and the library source.

**Status (2026-10-28):** fixed #1, #2, #5, #6, #7, #8, #9, #10, #11; #12 decided (keep feeding) and pinned by a test. Still open: #3 (real robot, needs approval) and #4 (check on blocks).

Current coverage:
- JUnit, 42 tests, all passing: `AimControllerTest`, `ShootCommandsTest`, `CameraGeometryTest`, `DriftModelTest`, `FieldFuelTest`.
- `sim-ui`: no unit tests (no vitest yet). Live checks in `sim-ui/scripts/`, including `run-control-test.ts` for #1, #2 and #7.

## Tooling to add
- **Java (sim classes):** JUnit 5 is already set up. Call `HAL.initialize(500, 0)` in `@BeforeAll` to use `DriverStationSim` / `SimHooks` without the GUI.
- **sim-ui:** add `vitest` as a dev dependency and `npm test`. Most targets are pure functions in `src/pathplanner/` and `src/nt/`.
- **Testability refactors, kept small:**
  - pass state into `WorldModel.setTruth(...)`
  - extract a pure "feed latch" from `Shooter`
  - extract the undo/history logic in `PlanTab` into a small reducer

## Findings → recommended tests

### 1. HIGH (sim) — the sim can deadlock — ✅ FIXED (phases 3–4); stress test ✅ added
- **Problem:** `WorldModel.setTruth` (synchronized) calls `drivetrain.getStateCopy()`, which takes CTRE's state lock. Meanwhile CTRE calls `WorldModel.onSample` while already holding that lock. The two threads can wait on each other forever. `SimManager` syncTruth has the same problem.
- **Unit test:** read the state *before* taking the world lock (`setTruth(Pose2d, SwerveDriveState)`). Then a JUnit test hammers `setTruth` from one thread while another thread calls `onSample` with a fake state. Assert it finishes within a timeout (`assertTimeoutPreemptively`).
- **Integration test:** a `sim-ui/scripts` stress script that sends `placeRobot` 200× while a path runs. Assert `/Sim/Heartbeat` keeps increasing.
- **Done:** `run-control-test.ts` section 4: 300 place/kidnap/syncTruth during a path; longest pause between robot loops 33 ms.

### 2. MED-HIGH (sim) — Play during a running auto doesn't start the new one — ✅ FIXED (2026-10-28)
- **Problem:** disable + enable in the same robot loop is never seen, because the robot code checks its mode once, at the start of each loop. `autonomousInit` doesn't re-run, and the state jumps ARMED → RUNNING.
- **Fix:** Play while enabled goes through a new `disarming` state: disable, wait at least one full robot loop, then enable. Note `DriverStationSim.notifyNewData()` updates `DriverStation.isEnabled()` immediately, so that can't tell whether the robot code has seen the disable — the loop count does. A 3 s timeout gives up with an error.
- **Tested:** `run-control-test.ts` section 1 (second Play follows the new path, elapsed restarts, `commandEnd` stays −1).
- **Unit test (JUnit + HAL):** drive `SimControl` through two consecutive plays, refreshing DS between loops. Assert:
  - it passes through a disarming state,
  - the second command gets scheduled,
  - `commandEnd` stays −1 until that command really ends.
- **Integration test:** `play-test.ts` started twice back-to-back. The second run's setpoint must follow the second path.

### 3. MEDIUM (real robot) — holding Y can make the feeder stutter — OPEN (now visible in the sim)
- **Problem:** `Shooter.runShooter()` re-checks the 5 % at-speed tolerance every loop, so a speed dip from a ball stops the feeder mid-shot. Since phase 4 the sim shows it: each ball costs the flywheel ~8 %, more than the 5 % tolerance. Autos are not affected (`ShootAllAuto` latches, `shootFor` feeds on a timer).
- **Unit test:** extract a feed latch (pure class). Assert:
  - not at speed → spin up only
  - first time at speed → feed
  - dip below tolerance → **keep feeding**
  - release → reset

### 4. MEDIUM (real robot, behavior change) — stick mapping
- No unit test. **Check on blocks before a match.** If the robot drives sideways, fix the encoder offsets in Tuner X.
- Optional: a test that `driverVelocityX/Y/RotationRate` follow X = −LeftY, Y = −LeftX, rot = −RightX, using `XboxControllerSim`.

### 5. MEDIUM (sim UI) — clicks made while disconnected are replayed on reconnect — ✅ FIXED (2026-10-28)
- **Problem:** `nt4.ts` queues one-shot actions while disconnected and sends them after the next reconnect. This breaks the "never re-sent" rule.
- **Fix:** nothing is queued while disconnected; settings are re-sent from their last value on reconnect, actions are dropped. Values set while connected but before the clock sync (a few ms) are still queued.
- **Tested:** with a fake WebSocket against the real `nt4.ts` (scratch check, not in the repo until vitest exists): the old client replayed a kidnap clicked while disconnected, the new one doesn't; the setting arrives once with its latest value; timestamps increase.
- **Unit test (vitest, mock WebSocket):**
  - action while disconnected → reconnect → assert **not** sent
  - retained setting while disconnected → reconnect → assert sent once
  - sent timestamps strictly increase

### 6. MEDIUM (UI, data loss) — Rename and Duplicate can overwrite files — ✅ FIXED (2026-10-28)
- **Problem:** there is no check for an existing name. A case-only rename on macOS deletes the file. Rename leaves autos pointing at the old path name.
- **Fix:** New / Rename / Duplicate check a fresh file list, ignoring case (`nameTaken`). A case-only rename goes through a temp file. Renaming a path offers to update the autos that use it (`renamePathIn`, keeps other fields); a case-only rename updates them automatically. Delete warns which autos use the path.
- **Tested:** `renamePathIn` (nested groups, other fields kept) by a scratch check; the click flow is typechecked only.
- **Unit test:** a pure name-check helper. Assert:
  - case-insensitive collisions are rejected
  - a case-only rename is allowed only through a temp name
  - a rename reports the autos that reference the old name

### 7. LOW-MED (sim) — a stale autonomous override can stay set — ✅ FIXED (2026-10-28)
- **Problem:** if the build fails, or ARMED never becomes RUNNING, the previous command stays queued for the next GUI auto enable.
- **Fix:** Play and Reset clear the override first; DISARMING/ARMED time out after 3 s with an error. New topic `/Sim/Run/overrideActive` shows whether one is queued.
- **Tested:** `run-control-test.ts` sections 2–3 (failed build and Stop leave nothing queued).
- **Unit test (JUnit + HAL):**
  - a failed build clears the override
  - ARMED times out and clears it
  - `stop()` clears it

### 8. LOW (UI) — waypoint insert/delete doesn't shift zones — ✅ FIXED (2026-10-28)
- **Problem:** `edit.ts` `insertWaypoint` / `deleteWaypoint` shift rotation targets and markers, but not `constraintZones` / `pointTowardsZones` (`min/maxWaypointRelativePos`).
- **Fix:** one `shiftPositions` helper moves all four, same rule; zone fields checked against PathPlannerLib 2026.1.2. Tested by a scratch check.
- **Unit test (vitest):** insert or delete on a path with zones; assert zone positions shift the same way markers do.

### 9. LOW (UI) — saving can overwrite PathPlanner-app edits — ✅ FIXED (2026-10-28)
- **Problem:** there is no file-changed check on save.
- **Fix:** the Plan tab remembers the file's modification time when it opens or saves it. On Save, if the file changed since, it asks before overwriting (a second Save overwrites). Typechecked; click flow not run.
- **Unit test:** a save guard compares the mtime from open with the current mtime and warns on mismatch.

### 10. LOW (UI) — concurrent saves share a temp file name — ✅ FIXED (2026-10-28)
- **Problem:** `deployFiles.ts` uses `${file}.tmp-${pid}` inside the deploy folder. A leftover temp file would get deployed to the roboRIO.
- **Fix:** unique temp name per write, removed if the write fails; leftover temp files are deleted when the dev server starts.
- **Tested (live):** two planted leftovers removed at startup; 5 simultaneous saves of one file → valid JSON, no temp files left.
- **Unit test (vitest, temp dir):**
  - two simultaneous PUTs of one file both succeed
  - the result is valid JSON
  - no `.tmp` files are left in the deploy tree

### 11. LOW (UI) — a click without a drag clears redo — ✅ FIXED (2026-10-28)
- **Problem:** pointer-down always pushes an undo entry.
- **Fix:** the undo snapshot is taken on the first real move of a drag. Typechecked; click flow not run.
- **Unit test:** history reducer — down + up with no move → no history entry, and redo is kept.

### 12. Behavior choice (real robot) — ✅ DECIDED: keep feeding (2026-10-28)
- **Behavior:** `shootFor()` feeds after 1.5 s even if the flywheel isn't at speed.
- **Test:** whichever way you choose, add a test that pins it (`shootFor` with a never-at-speed shooter feeds, or doesn't, after 1.5 s).
- **Done:** `shootFor` moved to `commands/ShootCommands.java` (same behavior, testable without motors). `ShootCommandsTest` pins: feeds at 1.5 s when never at speed, feeds at once when at speed, stops when interrupted.

## Second review (phases 3–4, 2026-10-25) — all fixed

No high-risk findings in the real-robot code. Each item was checked against the code before fixing.

| # | Severity | Finding | Fix | Test |
|---|---|---|---|---|
| R2-1 | Medium (sim) | The TOWER was modeled as a sightline-blocking block, and tags 15/16/31/32 sit inside it, so they were never seen. | The TOWER is still solid for Plan-tab collision warnings, but `blocksVision: false` (`gen-field.mjs` → `FieldModel`). | ✅ `CameraGeometryTest` (5 tests: tower tags visible, HUB blocks, face tags, behind camera, out of range) |
| R2-2 | Medium (sim) | The PhotonVision mount error was applied in each camera's own frame (`Transform3d.plus`), unlike the UI and guide. | Applied in the robot frame (translation + yaw about the robot's vertical axis). | Live: mount error X moves every camera forward |
| R2-3 | Med/low (**real robot**) | The minimum aim turn rate could sweep through the tolerance band, and "aimed" counted loops without checking the robot had slowed, so it could fire mid-overshoot. | "Aimed" now also needs tx to change ≤ 0.15° per loop (`MAX_SETTLED_TX_CHANGE_DEG`). **Still check overshoot on blocks.** | ✅ `AimControllerTest.sweepingThroughTheBandIsNotAimed` |
| R2-4 | Low (**real robot**) | A pose reset makes the heading jump, which read as thousands of °/s and dropped one MegaTag2 frame. | A jump over 30° in one loop is treated as a reset (rate 0). | Suggested: a `VisionSubsystem` test with a fake heading supplier |
| R2-6 | Low (sim) | With zero noise and a parked robot, identical LL3 values were dropped by NetworkTables, so there were no new frames. | Latency jitter of 0–0.5 ms makes every frame unique. | Live: kidnap at noise 0 gets corrected |
| R2-7 | Low (sim) | The flywheel "fast enough" check compared wheel speed with the motor setpoint (only right at 1:1 gearing). | Compares motor (rotor) speed; the Mechanisms panel uses the same units. | Suggested: a `FuelSim` test with gearing ≠ 1 |
| R2-8 | Low (UI) | The stale LL3 marker stayed after disabling it; saved camera settings were merged only at the top level. | `lastFrame` cleared when disabled; per-camera merge with defaults. | — |
| R2-5 | Low (sim) | At most one odometry sample (~4 ms) can be counted twice right after a teleport. | Not fixed (negligible). | — |

## New in phases 3–4 (no bug found, worth testing)
- **`LimelightSim` MegaTag2 math (JUnit):**
  - With zero noise, zero mount error and the true heading told, the solved pose equals the truth.
  - A heading error δ at distance d gives about δ·d of position error.
  - A mount error gives a constant bias.
- **`CameraGeometry`:** ✅ done (`CameraGeometryTest`).
- **`FuelSim` rules:** aim within/outside tolerance, too close / too far, slow flywheel → the expected result codes. Preload is consumed one ball per 0.25 s.
- **`VisionSubsystem` MegaTag2 filter:** frames are ignored above 360°/s, and the θ std dev is effectively infinite.
- **`AimController` minimum turn:** already covered (`justOutsideToleranceStillOvercomesFriction`, `insideToleranceUsesGentleProportionalTurn`).

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
