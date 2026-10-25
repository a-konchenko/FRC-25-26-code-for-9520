package frc.robot.sim;

import java.util.Random;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * ★ NEW (sim only) — how the real floor motion differs from what the wheel
 * encoders and gyro report.
 *
 * The robot code's odometry integrates encoder + gyro motion perfectly. The
 * WorldModel integrates the same motion through this model to get where the
 * robot REALLY is. The gap between the two is drift.
 *
 * Default values ("Well-tuned") are educated guesses:
 *   wheel scale 1.5 %   — tread wear / carpet compression, typically 1–3 %
 *   slip 1 cm/√m        — random walk, ×3 while accelerating hard (> 3 m/s²)
 *   gyro bias 0.3°/min  — Pigeon 2 is very stable at rest
 *   gyro scale 0.2 %    — most heading error comes from spinning
 * Expected: ~10–20 cm and ~1° after a 20 s auto, with no vision.
 * All values are live-editable from the web UI.
 */
public class DriftModel {

    // ── Parameters (set from the UI) ────────────────────────────────────────
    public double wheelScalePct      = 1.5;
    public double slipCmPerSqrtM     = 1.0;
    public double slipAccelThreshold = 3.0;  // m/s²
    public double slipAccelMultiplier = 3.0;
    public double gyroBiasDegPerMin  = 0.3;
    public double gyroScalePct       = 0.2;

    private Random rng = new Random(9520);
    private long   seed = 9520;

    // ── Accumulated error (for display) ─────────────────────────────────────
    private double distanceM     = 0.0;
    private double gyroErrorRad  = 0.0;

    public void setSeed(long newSeed) {
        if (newSeed != seed) {
            seed = newSeed;
            rng = new Random(newSeed);
        }
    }

    /** Restart the random sequence and the accumulators (start of a run). */
    public void reset() {
        rng = new Random(seed);
        distanceM = 0.0;
        gyroErrorRad = 0.0;
    }

    /**
     * Turns the motion the sensors report into the motion that really happened.
     *
     * @param measured   robot-relative motion from the encoders (dx, dy) and
     *                   the gyro (dtheta) over this step
     * @param dtSeconds  step length
     * @param accelMps2  current translational acceleration magnitude
     */
    public Twist2d apply(Twist2d measured, double dtSeconds, double accelMps2) {
        double stepDist = Math.hypot(measured.dx, measured.dy);
        distanceM += stepDist;

        // Wheels think they went further than they did (worn tread = smaller radius)
        double scale = 1.0 - wheelScalePct / 100.0;
        double dx = measured.dx * scale;
        double dy = measured.dy * scale;

        // Random slip: variance grows with distance travelled (random walk)
        if (stepDist > 1e-6 && slipCmPerSqrtM > 0) {
            double sigma = slipCmPerSqrtM / 100.0 * Math.sqrt(stepDist);
            if (accelMps2 > slipAccelThreshold) sigma *= slipAccelMultiplier;
            dx += rng.nextGaussian() * sigma;
            dy += rng.nextGaussian() * sigma;
        }

        // Gyro reads (true + error), so true rotation = reported − error step
        double gyroErrStep = Math.toRadians(gyroBiasDegPerMin / 60.0) * dtSeconds
                           + measured.dtheta * gyroScalePct / 100.0;
        gyroErrorRad += gyroErrStep;
        double dtheta = measured.dtheta - gyroErrStep;

        return new Twist2d(dx, dy, dtheta);
    }

    public double getDistanceM()     { return distanceM; }
    public double getGyroErrorDeg()  { return Math.toDegrees(gyroErrorRad); }
}
