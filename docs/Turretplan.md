Plan: LL4 turret with 360° rotation and adjustable hood
What changes conceptually
Today the whole robot turns to aim. AimController turns LL4 tx into a chassis rotation rate, and FuelSim assumes the shooter points out of the robot's front. With the LL4 riding on a turret:

tx is measured relative to the turret, not the chassis. Aiming means turning the turret until tx is about 0, while the robot drives and turns freely.
The LL4 moves with the turret. Its position and direction on the robot now depend on the turret angle, so the simulated LL4 has to follow it.
Distance sets the shot. The hood angle and flywheel speed come from a table of distance → {hood angle, flywheel RPS}. Distance comes from LL4 ty (already in getEstimatedDistanceMeters()), or from the robot's pose estimate when no tag is visible.
Hardware assumptions (placeholders, marked TO MEASURE)
Part	Assumption
Turret motor	Kraken X60, CAN 20, about 60:1 to the turret ring
Turret absolute encoder	CANcoder, CAN 21, so the turret knows its angle at power-on without homing
Hood motor	Kraken X60, CAN 22, hood range 15°–60°, homed against a hard stop
Rotation range	"360°" as −200° … +200° with soft limits (wires allow about 400° total). A true continuous turret needs a slip ring; see decision 2.
LL4 mount	On the turret base (turns with the turret, does not tilt with the hood), 0.12 m off the turret axis
CAN IDs 0–17 and 19 are taken, so 20–22 are free.

Phase T1: Turret and hood mechanisms (robot code + sim)
Robot code (new, affects the real robot):

subsystems/Turret.java: MotionMagic position control, fused with the CANcoder, with soft limits.
setAngle(fieldOrRobotDeg) with wrap logic: it picks the equivalent angle (θ ± 360°) that stays within limits and is closest to where the turret is now. So going 170° → −170° turns 20°, not 340°, unless the limit forces the long way around.
getAngle() and isAt(target, tolDeg).
subsystems/Hood.java: MotionMagic angle control, home() against the hard stop using stator current, and isAt().
TurretConstants: IDs, gearing, limits, tolerances, and the LL4 offset from the turret axis.
Feature flag kTurretInstalled. The real robot doesn't have this hardware, and creating TalonFX objects on missing CAN IDs floods the console with errors. With the flag off, the robot behaves exactly as it does today (see decision 1).
Sim:

MechanismSims: turret (DCMotorSim with hard stops at the cable limits) and hood, feeding the CANcoder sim state.
UI: the turret drawn on the robot outline, a direction arrow, turret and hood gauges in the Mechanisms panel.
NT additions: /Sim/Mech/turret = {angle, setpoint, hood, hoodSetpoint, homed}, and /Sim/Robot/turret = {axis x, y, LL4 offset, min, max}.
Tests: wrap/limit math (the 170° → −170° case, near-limit cases, out-of-range targets) and the hood clamp.

Phase T2: Turret aiming with the LL4
LL4 sim follows the turret (LimelightSim): the camera's yaw is chassis heading + true turret angle, and its position circles the turret axis. Mount error still works, as an error relative to the turret.
TurretAimController (new, pure, unit-tested) replaces chassis aiming when the turret is installed. Turret setpoint = current turret angle − tx, plus:
Robot-rotation feedforward: when the robot turns at ω, the turret turns at −ω, so the target doesn't slide out of view while driving.
Pose fallback when no target is visible: aim at the alliance hub using drivetrain.getState().Pose, so the turret is already roughly on target when the LL4 picks up the tags again.
Offset correction (parallax): the LL4 sits off the turret axis, so tx is corrected for distance. This is about 3° at 2 m with a 0.12 m offset, which is enough to miss.
An "aimed" gate built the same way as today: within tolerance, tx steady, and held for 5 loops.
TurretTrackCommand is the turret's default command while in hub mode, so it tracks all the time. The driver gets full chassis control back.
Bindings: B stops turning the chassis and becomes "hold to shoot when aimed". RB (hub/trench) stays the same.
ShootAllAuto / AimAndShoot: they no longer require the drivetrain, so AimAndShoot can become an event marker that shoots while driving a path. That's one of the main reasons to have a turret.
Integration script turret-test.ts: drive the test square and check that tx stays within ±2° the whole way, then shoot from a pose facing away from the hub and check that it scores.

Phase T3: Hood angle and the shot table
ShotMap (new, pure, unit-tested): WPILib InterpolatingDoubleTreeMaps for distance → hood angle and distance → flywheel RPS. It starts with about 6 placeholder rows (1.5 m to 5.5 m) and is clamped at both ends.
Shooter.spinUp() takes the RPS from the table. The fixed 55 RPS remains as the default when there's no distance.
Fire gate: turret aimed and hood at its setpoint and flywheel at speed.
FuelSim scoring upgrade: from "scored if aimed" to simple ballistics. The exit speed comes from flywheel speed × an efficiency constant, the launch angle is the hood angle, and the direction is chassis heading + turret angle. A ball scores if its arc enters the hub opening at the right height (72 in) without hitting the rim. With this, a wrong table row shows up as "too short" or "too long", which turns tuning the table in the sim into a real student exercise. Miss reasons become: aim, short, long, flywheel slow.
UI: hood angle and the expected arc drawn on the shot line, and the table shown in a panel (read-only at first).
Phase T4 (optional): Shoot while moving
Aim at where the hub will be relative to the robot when the ball arrives: hub position − robot velocity × time of flight (from the table). This can be added later; T1–T3 work without it.

What stays unchanged
The drivetrain, PathPlanner setup, MegaTag2 (LL3) and PhotonVision.
With kTurretInstalled = false, everything behaves exactly as today, including chassis aiming and the current FuelSim rule.
Docs and tests (each phase)
docs/simulator.md: blocks table, NT contract, the robot-code-changes table (this work affects the real robot), and placeholders (TO MEASURE).
USER_GUIDE.md: a new scenario, "tune the shot table".
New JUnit tests: turret wrap/limits, TurretAimController (sign, rotation feedforward, offset correction, pose fallback), ShotMap, ballistics hit/miss. Plus turret-test.ts.
Decisions I need from you before T1
How to enable the turret.
Recommended: a constant kTurretInstalled that you set on purpose. It's off for the real robot today, and you turn it on to try the turret in the sim, so the sim and the real code stay identical.
Alternative: "on in sim, off on the robot". This is easier, but it means the sim runs different robot logic, which goes against the repo's rule.
Rotation. ±200° with soft limits (recommended; this is how most FRC turrets are built), or truly continuous with a slip ring (no limits or wrap-around needed, but rarer and more expensive).
Keep chassis aiming as a fallback? For example, if the turret is at a limit, turn the robot to bring the target back into range. It's useful, but more logic. I'd leave it out of T2.
Fuel scoring. Upgrade to ballistics in T3 (recommended, needed to make hood tuning meaningful), or keep "scored if aimed" plus a hood-angle tolerance check.
Tell me your choices, or "go with the recommendations", and I'll start T1. I can also put this plan on a shareable page if you want to show it to the team.