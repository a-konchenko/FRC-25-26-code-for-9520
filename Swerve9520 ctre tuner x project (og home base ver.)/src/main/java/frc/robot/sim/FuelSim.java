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
 * No ball flight physics and no floor pickup.
 */
public class FuelSim {
    public static final int HIT = 0, MISS_AIM = 1, MISS_CLOSE = 2, MISS_FAR = 3, MISS_SLOW = 4;

    public int    preload  = SimRobotConstants.kFuelPreload;
    public double aimTolDeg = 3.0;
    public double minDistM = 1.5, maxDistM = 5.5;

    private int remaining = preload, scored = 0, missed = 0, seq = 0;
    private double feedTimer = 0;
    private final ArrayDeque<double[]> shots = new ArrayDeque<>(); // {seq, fx, fy, tx, ty, result}

    private final MechanismSims mech;
    private final Shooter shooter;
    private final FieldModel field;

    public FuelSim(MechanismSims mech, Shooter shooter, FieldModel field) {
        this.mech = mech;
        this.shooter = shooter;
        this.field = field;
    }

    public void readSettings(SimNT nt) {
        preload   = (int) Math.round(nt.setting("fuel/preload", preload));
        aimTolDeg = nt.setting("fuel/aimTolDeg", aimTolDeg);
        minDistM  = nt.setting("fuel/minDistM", minDistM);
        maxDistM  = nt.setting("fuel/maxDistM", maxDistM);
    }

    public void reset() {
        remaining = preload;
        scored = missed = 0;
        feedTimer = 0;
        shots.clear();
    }

    public void update(double dt, Pose2d truth) {
        if (Math.abs(mech.feeder.rps()) < 15 || remaining <= 0) {
            feedTimer = 0;
            return;
        }
        feedTimer += dt;
        if (feedTimer < 0.25) return;
        feedTimer -= 0.25;
        shoot(truth);
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
    }

    public void publish(SimNT nt) {
        nt.array("Fuel/state", new double[] { remaining, scored, missed, preload });
        double[] flat = new double[shots.size() * 6];
        int k = 0;
        for (double[] s : shots) for (double v : s) flat[k++] = v;
        nt.array("Fuel/shots", flat);
    }

    public int getScored() { return scored; }
}
