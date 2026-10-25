package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Twist2d;

/** Unit tests for the simulator's drift model (how true motion differs from what sensors report). */
class DriftModelTest {
    private static final double DT = 0.004; // one 250 Hz odometry step

    /** Drift model with every error source turned off. */
    private static DriftModel ideal() {
        DriftModel m = new DriftModel();
        m.wheelScalePct = 0;
        m.slipCmPerSqrtM = 0;
        m.gyroBiasDegPerMin = 0;
        m.gyroScalePct = 0;
        return m;
    }

    @Test
    void idealModelChangesNothing() {
        Twist2d out = ideal().apply(new Twist2d(0.01, -0.02, 0.03), DT, 0);
        assertEquals(0.01, out.dx, 1e-12);
        assertEquals(-0.02, out.dy, 1e-12);
        assertEquals(0.03, out.dtheta, 1e-12);
    }

    @Test
    void wornWheelsTravelLessThanReported() {
        DriftModel m = ideal();
        m.wheelScalePct = 1.5;
        Twist2d out = m.apply(new Twist2d(1.0, 0, 0), DT, 0);
        assertEquals(0.985, out.dx, 1e-12); // 1.5 % short
    }

    @Test
    void gyroBiasAccumulatesOverTime() {
        DriftModel m = ideal();
        m.gyroBiasDegPerMin = 0.3;
        // Sit still for 60 s
        for (int i = 0; i < 15000; i++) m.apply(new Twist2d(), DT, 0);
        assertEquals(0.3, m.getGyroErrorDeg(), 1e-6);
    }

    @Test
    void gyroScaleErrorShowsUpWhenSpinning() {
        DriftModel m = ideal();
        m.gyroScalePct = 0.2;
        Twist2d out = m.apply(new Twist2d(0, 0, 2 * Math.PI), DT, 0); // one full turn
        assertEquals(360 * 0.002, m.getGyroErrorDeg(), 1e-9);
        assertTrue(out.dtheta < 2 * Math.PI, "robot really turned slightly less than the gyro says");
    }

    @Test
    void distanceIsAccumulated() {
        DriftModel m = ideal();
        m.apply(new Twist2d(3, 4, 0), DT, 0); // 5 m step
        m.apply(new Twist2d(1, 0, 0), DT, 0);
        assertEquals(6.0, m.getDistanceM(), 1e-12);
    }

    @Test
    void sameSeedGivesSameSlip() {
        DriftModel a = new DriftModel();
        DriftModel b = new DriftModel();
        a.setSeed(42);
        b.setSeed(42);
        for (int i = 0; i < 100; i++) {
            Twist2d ta = a.apply(new Twist2d(0.01, 0, 0), DT, 0);
            Twist2d tb = b.apply(new Twist2d(0.01, 0, 0), DT, 0);
            assertEquals(ta.dx, tb.dx);
            assertEquals(ta.dy, tb.dy);
        }
    }

    @Test
    void resetReplaysTheSameRandomSequence() {
        DriftModel m = new DriftModel();
        Twist2d first = m.apply(new Twist2d(0.01, 0, 0), DT, 0);
        m.reset();
        Twist2d again = m.apply(new Twist2d(0.01, 0, 0), DT, 0);
        assertEquals(first.dx, again.dx);
        assertEquals(0.01, m.getDistanceM(), 1e-12); // accumulator was cleared too
    }

    @Test
    void hardAccelerationSlipsMore() {
        // Same random numbers, so the only difference is the ×3 multiplier.
        DriftModel calm = ideal();
        DriftModel hard = ideal();
        calm.slipCmPerSqrtM = hard.slipCmPerSqrtM = 1.0;
        Twist2d c = calm.apply(new Twist2d(0.01, 0, 0), DT, 0);
        Twist2d h = hard.apply(new Twist2d(0.01, 0, 0), DT, 5.0); // above 3 m/s² threshold
        assertEquals(3 * (c.dx - 0.01), h.dx - 0.01, 1e-12);
    }

    @Test
    void standingStillDoesNotSlip() {
        DriftModel m = new DriftModel();
        m.gyroBiasDegPerMin = 0;
        Twist2d out = m.apply(new Twist2d(), DT, 0);
        assertEquals(0.0, out.dx);
        assertEquals(0.0, out.dy);
    }
}
