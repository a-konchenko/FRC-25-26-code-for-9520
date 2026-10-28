package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for FUEL on the field: staging per the 2026 Game Manual, rolling,
 * bouncing off walls and elements, bumpers pushing, the intake picking up
 * from the front only, and the 504 total never changing.
 */
class FieldFuelTest {
    private static FieldModel field;
    private static final double R = FieldFuel.R;
    private static final double L = SimRobotConstants.kLengthWithBumpersM, W = SimRobotConstants.kWidthWithBumpersM;
    private static final double DT = 0.02;
    private FieldFuel fuel;

    @BeforeAll
    static void loadField() {
        field = new FieldModel();
    }

    @BeforeEach
    void setUp() {
        fuel = new FieldFuel(field);
    }

    /** Robot standing still (or moving from a to b) with the default bumpers and intake. */
    private static FieldFuel.RobotBody robot(double x0, double y0, double x1, double y1, double th) {
        return new FieldFuel.RobotBody(x0, y0, th, x1, y1, th, L, W,
            SimRobotConstants.kIntakeDepthM, SimRobotConstants.kIntakeWidthM);
    }

    private void run(double seconds, FieldFuel.RobotBody body, int maxPickups) {
        for (double t = 0; t < seconds; t += DT) fuel.step(DT, body, maxPickups);
    }

    @Test
    void stagingMatchesTheGameManual() {
        fuel.restage(9520, 400, 24, 24); // 504 − 2×24 depot − 2×24 chute − 8 preload
        assertEquals(400 + 2 * 24, fuel.onField());
        assertEquals(24, fuel.inChute(false));
        assertEquals(24, fuel.inChute(true));
        assertEquals(504 - 8, fuel.total());
        // Neutral zone balls straddle the CENTER LINE, roughly half on each side
        double[] p = fuel.positions();
        int left = 0, right = 0;
        for (int i = 0; i < p.length; i += 2) {
            if (Math.abs(p[i] - field.length / 2) < 1.5) { if (p[i] < field.length / 2) left++; else right++; }
        }
        assertEquals(400, left + right, "400 balls within 1.5 m of the center line");
        assertTrue(Math.abs(left - right) < 40, "split " + left + " / " + right);
    }

    @Test
    void stagedBallsDontOverlapAndStayPut() {
        fuel.restage(1, 408, 24, 24); // the most the NEUTRAL ZONE ever holds (no preloads)
        assertTrue(minGap() >= -1e-9, "no overlaps when staged (gap " + minGap() + " m)");
        double[] before = fuel.positions();
        run(1.0, null, 0);
        double[] after = fuel.positions();
        double moved = 0;
        for (int i = 0; i < before.length; i++) moved = Math.max(moved, Math.abs(after[i] - before[i]));
        assertTrue(moved < 1e-9, "a resting pile doesn't creep (moved " + moved + " m)");
    }

    @Test
    void ballRollsAndStops() {
        fuel.add(8, 2, 2.0, 0); // 2 m/s along +X, open floor
        run(4.0, null, 0);
        double[] p = fuel.positions();
        double rolled = p[0] - 8;
        assertTrue(rolled > 1.0 && rolled < 2.5, "rolled " + rolled + " m");
        assertEquals(0.0, fuel.vx[0], 1e-9, "stopped");
    }

    @Test
    void ballStaysInsideTheWalls() {
        fuel.add(8, 0.5, 0, -8.0); // hard toward the right guardrail
        run(2.0, null, 0);
        assertTrue(fuel.positions()[1] >= R - 1e-9, "inside the guardrail");
    }

    @Test
    void ballBouncesOffTheHub() {
        double[] hub = field.box("blue-hub");
        fuel.add(hub[2] + 0.5, (hub[1] + hub[3]) / 2, -3.0, 0); // at the HUB's neutral-zone face
        run(2.0, null, 0);
        double x = fuel.positions()[0];
        assertTrue(x >= hub[2] + R - 1e-6, "outside the HUB, x = " + x);
    }

    @Test
    void bumpersPushWhenIntakeIsOff() {
        fuel.add(8.0, 2.0, 0, 0);
        // Robot drives +X from 1 m behind the ball to 0.2 m past it, intake off
        double x0 = 8.0 - 1.0;
        for (int k = 0; k < 40; k++) {
            double a = x0 + k * 0.03, b = a + 0.03;
            fuel.step(DT, robot(a, 2.0, b, 2.0, 0), 0);
        }
        assertEquals(1, fuel.onField(), "not picked up");
        assertTrue(fuel.positions()[0] > x0 + 1.2 + L / 2, "pushed ahead of the bumper");
    }

    @Test
    void intakePicksUpFromTheFront() {
        fuel.add(8.0 + L / 2 + R + 0.02, 2.0, 0, 0); // just in front of the bumper
        int got = fuel.step(DT, robot(8.0, 2.0, 8.0, 2.0, 0), 5);
        assertEquals(1, got);
        assertEquals(0, fuel.onField());
    }

    @Test
    void intakeDoesNotPickUpFromBehindOrTheSide() {
        fuel.add(8.0 - L / 2 - R + 0.01, 2.0, 0, 0); // touching the back bumper
        fuel.add(8.0, 2.0 + W / 2 + R - 0.01, 0, 0); // touching the left bumper
        int got = fuel.step(DT, robot(8.0, 2.0, 8.0, 2.0, 0), 5);
        assertEquals(0, got);
        assertEquals(2, fuel.onField());
    }

    @Test
    void intakeRespectsTheLimit() {
        for (int k = -1; k <= 1; k++) fuel.add(8.0 + L / 2 + R, 2.0 + k * 0.17, 0, 0);
        int got = fuel.step(DT, robot(8.0, 2.0, 8.0, 2.0, 0), 1);
        assertEquals(1, got, "only as many as the robot may take");
        assertEquals(2, fuel.onField());
    }

    @Test
    void scoredBallComesBackOutOfTheHub() {
        fuel.toHub(false);
        assertEquals(1, fuel.inHub());
        run(FieldFuel.kHubDelayS + 0.1, null, 0);
        assertEquals(0, fuel.inHub());
        assertEquals(1, fuel.onField());
        double[] hub = field.box("blue-hub");
        assertTrue(fuel.positions()[0] > hub[2], "out of the neutral-zone side of the blue HUB");
    }

    @Test
    void chuteRollsOutWhenReleased() {
        fuel.restage(3, 0, 0, 24);
        run(1.0, null, 0);
        assertEquals(24, fuel.inChute(false), "door closed: nothing comes out");
        fuel.releaseChute(false);
        run(24 * FieldFuel.kChuteIntervalS + 0.5, null, 0);
        assertEquals(0, fuel.inChute(false));
        assertEquals(24, fuel.inChute(true), "red chute untouched");
        assertEquals(24, fuel.onField());
        double[] p = fuel.positions();
        for (int i = 0; i < p.length; i += 2) assertTrue(p[i] < 4.0, "blue chute balls stay on the blue end");
    }

    @Test
    void totalNeverChanges() {
        fuel.restage(7, 400, 24, 24);
        int start = fuel.total();
        fuel.releaseChute(true);
        fuel.toHub(true);
        int inRobot = 0;
        // Drive through the NEUTRAL ZONE with the intake on
        double y = field.width / 2;
        for (int k = 0; k < 150; k++) {
            double a = field.length / 2 - 2.0 + k * 0.03;
            inRobot += fuel.step(DT, robot(a, y, a + 0.03, y, 0), 1);
        }
        assertTrue(inRobot > 10, "picked up " + inRobot);
        assertEquals(start + 1, fuel.total() + inRobot, "floor + hub + chutes + robot is constant");
    }

    @Test
    void stepIsFastEnoughForTheRobotLoop() {
        fuel.restage(9520, 408, 24, 24);
        double y = field.width / 2;
        long t0 = System.nanoTime();
        int loops = 200;
        for (int k = 0; k < loops; k++) {
            double a = field.length / 2 - 3.0 + k * 0.03;
            fuel.step(DT, robot(a, y, a + 0.03, y, 0), 0); // bulldozing, the worst case
        }
        double ms = (System.nanoTime() - t0) / 1e6 / loops;
        assertTrue(ms < 5.0, ms + " ms per 20 ms loop");
    }

    /** Smallest gap between any two ball surfaces (negative = overlap). */
    private double minGap() {
        double[] p = fuel.positions();
        double min = Double.MAX_VALUE;
        for (int i = 0; i < p.length; i += 2) for (int j = i + 2; j < p.length; j += 2) {
            min = Math.min(min, Math.hypot(p[i] - p[j], p[i + 1] - p[j + 1]) - 2 * R);
        }
        return min;
    }
}
