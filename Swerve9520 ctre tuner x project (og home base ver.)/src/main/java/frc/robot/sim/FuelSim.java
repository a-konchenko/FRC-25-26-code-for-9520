package frc.robot.sim;

import java.util.ArrayDeque;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.subsystems.Shooter;

/**
 * ★ NEW (sim only) — FUEL, the simple way (agreed: "scored if aimed").
 *
 * The robot starts with `preload` FUEL (game manual: up to 8). While the
 * feeder spins (|feeder| > 15 rps) one ball leaves every 0.25 s. A ball
 * SCORES in the alliance HUB when, at that moment, the TRUE robot:
 *   - points its shooter within ±aimTolDeg of the hub center,
 *   - is between minDist and maxDist from the hub center, and
 *   - has the flywheel at ≥ 90 % of its setpoint.
 * Otherwise it misses, with a reason (aim / too close / too far / slow wheel).
 * The shooter is assumed to face the robot's front (TO MEASURE — kShooterYawDeg).
 * No ball flight physics.
 *
 * Phase 5a: the other FUEL lies on the field (FieldFuel, 504 in total). A
 * scored ball goes into the HUB and comes back out into the NEUTRAL ZONE; a
 * missed ball lands where it was aimed. While the intake roller spins, balls
 * touching the front of the TRUE robot are picked up (up to `capacity`).
 */
public class FuelSim {
    public static final int HIT = 0, MISS_AIM = 1, MISS_CLOSE = 2, MISS_FAR = 3, MISS_SLOW = 4;
    /** Intake roller speed (rps) above which it counts as running. */
    private static final double kIntakeOnRps = 5.0;

    public int    preload  = SimRobotConstants.kFuelPreload;
    public double aimTolDeg = 3.0;
    public double minDistM = 1.5, maxDistM = 5.5;
    public int    capacity = SimRobotConstants.kHopperCapacity;
    public double intakeRate = SimRobotConstants.kIntakeRatePerS;

    private int remaining = preload, scored = 0, missed = 0, picked = 0, seq = 0;
    private double feedTimer = 0, intakeTokens = 0, physicsMs = 0;
    private Pose2d lastTruth = null;
    private final ArrayDeque<double[]> shots = new ArrayDeque<>(); // {seq, fx, fy, tx, ty, result}

    private final MechanismSims mech;
    private final Shooter shooter;
    private final FieldModel field;
    private final FieldFuel floor;

    public FuelSim(MechanismSims mech, Shooter shooter, FieldModel field) {
        this.mech = mech;
        this.shooter = shooter;
        this.field = field;
        this.floor = new FieldFuel(field);
    }

    public void readSettings(SimNT nt) {
        preload    = (int) Math.round(nt.setting("fuel/preload", preload));
        aimTolDeg  = nt.setting("fuel/aimTolDeg", aimTolDeg);
        minDistM   = nt.setting("fuel/minDistM", minDistM);
        maxDistM   = nt.setting("fuel/maxDistM", maxDistM);
        capacity   = (int) Math.round(nt.setting("fuel/capacity", capacity));
        intakeRate = nt.setting("fuel/intakeRate", intakeRate);
    }

    /** Start of a match: preload the robot and stage the other FUEL on the field (504 in all). */
    public void reset(long seed) {
        remaining = Math.max(0, Math.min(preload, capacity));
        scored = missed = picked = 0;
        feedTimer = 0;
        intakeTokens = 0;
        shots.clear();
        int depots = 2 * SimRobotConstants.kFuelPerDepot, chutes = 2 * SimRobotConstants.kFuelPerChute;
        floor.restage(seed, SimRobotConstants.kFuelTotal - depots - chutes - remaining,
            SimRobotConstants.kFuelPerDepot, SimRobotConstants.kFuelPerChute);
    }

    public void releaseChute(boolean red) { floor.releaseChute(red); }

    public void update(double dt, Pose2d truth) {
        updateField(dt, truth);
        if (Math.abs(mech.feeder.rps()) < 15 || remaining <= 0) {
            feedTimer = 0;
            return;
        }
        feedTimer += dt;
        if (feedTimer < 0.25) return;
        feedTimer -= 0.25;
        shoot(truth);
    }

    /** Field FUEL physics + intake pickup, using the TRUE robot pose. */
    private void updateField(double dt, Pose2d truth) {
        long t0 = System.nanoTime();
        Pose2d prev = lastTruth != null ? lastTruth : truth;
        lastTruth = truth;
        boolean intakeOn = Math.abs(mech.intake.rps()) > kIntakeOnRps;
        // Token bucket: at most intakeRate balls/s, a short burst of 2 allowed
        intakeTokens = intakeOn ? Math.min(2.0, intakeTokens + intakeRate * dt) : 0;
        int canTake = intakeOn ? Math.min(capacity - remaining, (int) intakeTokens) : 0;
        var body = new FieldFuel.RobotBody(prev.getX(), prev.getY(), prev.getRotation().getRadians(),
            truth.getX(), truth.getY(), truth.getRotation().getRadians(),
            SimRobotConstants.kLengthWithBumpersM, SimRobotConstants.kWidthWithBumpersM,
            SimRobotConstants.kIntakeDepthM, SimRobotConstants.kIntakeWidthM);
        int got = floor.step(dt, body, Math.max(0, canTake));
        remaining += got;
        picked += got;
        intakeTokens -= got;
        physicsMs = (System.nanoTime() - t0) / 1e6;
    }

    private void shoot(Pose2d robot) {
        remaining--;
        boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub = field.hubCenter(red);
        double heading = robot.getRotation().getRadians() + Math.toRadians(SimRobotConstants.kShooterYawDeg);
        double toHub = Math.atan2(hub.getY() - robot.getY(), hub.getX() - robot.getX());
        double aimErr = Math.toDegrees(Math.IEEEremainder(heading - toHub, 2 * Math.PI));
        double dist = robot.getTranslation().getDistance(hub);
        double setpoint = Math.abs(shooter.getShooterSetpointRPS());
        // The setpoint is in motor (rotor) rotations/s — the Shooter sets no gear ratio
        boolean fastEnough = setpoint > 1 && Math.abs(mech.shooter.rotorRps()) >= 0.9 * setpoint;

        int result;
        if (!fastEnough)                       result = MISS_SLOW;
        else if (Math.abs(aimErr) > aimTolDeg) result = MISS_AIM;
        else if (dist < minDistM)              result = MISS_CLOSE;
        else if (dist > maxDistM)              result = MISS_FAR;
        else                                   result = HIT;

        if (result == HIT) scored++; else missed++;
        mech.onShot();

        // Where it lands, for drawing: hits end at the hub, misses along the shooter direction
        double range = result == HIT ? dist : result == MISS_SLOW ? Math.min(dist, 1.5) : Math.min(dist, maxDistM);
        double tx = result == HIT ? hub.getX() : robot.getX() + Math.cos(heading) * range;
        double ty = result == HIT ? hub.getY() : robot.getY() + Math.sin(heading) * range;
        shots.addLast(new double[] { ++seq, robot.getX(), robot.getY(), tx, ty, result });
        while (shots.size() > 24) shots.removeFirst();

        // The ball stays in play: into the HUB (comes back out later), or onto the floor where it lands
        if (result == HIT) floor.toHub(red);
        else floor.add(tx, ty, Math.cos(heading) * 1.0, Math.sin(heading) * 1.0);
    }

    private int publishCount = 0;

    public void publish(SimNT nt) {
        // {in robot, scored, missed, preload, on field, in hubs, blue chute, red chute, capacity, total, picked up}
        nt.array("Fuel/state", new double[] { remaining, scored, missed, preload,
            floor.onField(), floor.inHub(), floor.inChute(false), floor.inChute(true), capacity,
            floor.total() + remaining, picked });
        double[] flat = new double[shots.size() * 6];
        int k = 0;
        for (double[] s : shots) for (double v : s) flat[k++] = v;
        nt.array("Fuel/shots", flat);
        // Ball positions at 25 Hz (about 8 KB) — plenty for drawing
        if (publishCount++ % 2 == 0) nt.array("Fuel/field", floor.positions());
        nt.number("Fuel/physicsMs", physicsMs);
    }

    public int getScored() { return scored; }

    /** Teleport (place / kidnap / play): don't treat the jump as the robot driving through balls. */
    public void teleported() { lastTruth = null; }
}
