package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.VisionSubsystem;

/**
 * Shared aiming math for AutoAimCommand (teleop) and ShootAllAuto (auto).
 *
 * Turns LL4 tx (degrees, positive = target RIGHT of crosshair) into a
 * rotational rate (rad/s, positive = counter-clockwise).
 *
 * BUG FIX (2026-10): the old code fed tx in radians into kP = 0.03, so the
 * output was ~0.005 rad/s — below the request deadband, the robot never
 * turned. The result was also negated, which would have turned the robot
 * AWAY from the target. Here the PID works in radians with a kP sized for
 * radians, and the sign is correct without negation:
 *   error = 0 - tx  → target right (tx > 0) → negative rate → turn clockwise.
 *
 * TUNING ORDER:
 *   1. Raise kP until the robot snaps toward the target without oscillating
 *   2. Add kD if it overshoots
 *   3. Tighten AIM_TOLERANCE_DEG once stable
 */
public class AimController {

    // ── Tuning ────────────────────────────────────────────────────────────────
    public static final double kP                = 4.0;   // (rad/s) per rad of tx
    public static final double kI                = 0.0;
    public static final double kD                = 0.05;
    public static final double MAX_ROT_RAD_S     = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    public static final double AIM_TOLERANCE_DEG = 1.5;
    public static final double TX_DEADZONE_DEG   = 0.3;
    /**
     * Smallest turn rate used while OUTSIDE the aim tolerance. Near the target
     * the P term alone asks for ~0.2 rad/s — less voltage than the drivetrain's
     * static friction — so the robot stalls a few degrees off and never fires
     * (found in the simulator, 2026-10). Tune on the robot.
     */
    public static final double MIN_ROT_RAD_S     = 0.4;
    public static final int    SETTLE_LOOPS      = 5;     // ~100 ms at 50 Hz
    /**
     * "Aimed" also requires tx to be nearly still (change per loop ≤ this), so a
     * robot sweeping THROUGH the tolerance band while overshooting doesn't count
     * as aimed and fire (code review, 2026-10).
     */
    public static final double MAX_SETTLED_TX_CHANGE_DEG = 0.15;

    private final PIDController pid = new PIDController(kP, kI, kD);
    private int     settleCount = 0;
    private boolean aimed       = false;
    private double  lastTx      = Double.NaN;

    public AimController() {
        pid.setSetpoint(0.0); // tx = 0 means perfectly aimed
    }

    public void reset() {
        pid.reset();
        settleCount = 0;
        aimed       = false;
        lastTx      = Double.NaN;
    }

    /** Call once per loop. Returns the rotational rate to command (rad/s). */
    public double calculate(VisionSubsystem vision) {
        return calculate(vision.hasShooterTarget(), vision.getShooterTx());
    }

    /** Same as calculate(vision), from raw LL4 values (used by unit tests). */
    public double calculate(boolean hasTarget, double tx) {
        if (!hasTarget) {
            reset();
            return 0.0;
        }

        double rot = 0.0;
        if (Math.abs(tx) > TX_DEADZONE_DEG) {
            rot = MathUtil.clamp(pid.calculate(Math.toRadians(tx)), -MAX_ROT_RAD_S, MAX_ROT_RAD_S);
            if (Math.abs(tx) > AIM_TOLERANCE_DEG && Math.abs(rot) < MIN_ROT_RAD_S) {
                rot = Math.copySign(MIN_ROT_RAD_S, -tx); // target right (tx > 0) → clockwise
            }
        }

        boolean still = Double.isNaN(lastTx) || Math.abs(tx - lastTx) <= MAX_SETTLED_TX_CHANGE_DEG;
        lastTx = tx;
        settleCount = (Math.abs(tx) <= AIM_TOLERANCE_DEG && still) ? settleCount + 1 : 0;
        aimed = settleCount >= SETTLE_LOOPS;
        return rot;
    }

    /** True when tx has been within tolerance for SETTLE_LOOPS loops in a row. */
    public boolean isAimed() { return aimed; }
}
