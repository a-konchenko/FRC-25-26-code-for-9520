package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the shared aiming math.
 *
 * Sign convention: tx > 0 means the target is RIGHT of the crosshair, and a
 * positive rate is counter-clockwise. So a target on the right must give a
 * NEGATIVE rate (turn clockwise). The 2026-10 bug got this backwards.
 */
class AimControllerTest {
    private AimController aim;

    @BeforeEach
    void setUp() {
        aim = new AimController();
    }

    @Test
    void noTargetMeansNoTurn() {
        assertEquals(0.0, aim.calculate(false, 10.0));
        assertFalse(aim.isAimed());
    }

    @Test
    void targetOnRightTurnsClockwise() {
        assertTrue(aim.calculate(true, 10.0) < 0, "target right → negative (clockwise) rate");
    }

    @Test
    void targetOnLeftTurnsCounterClockwise() {
        assertTrue(aim.calculate(true, -10.0) > 0, "target left → positive (CCW) rate");
    }

    @Test
    void turnIsBigEnoughToMoveTheRobot() {
        // The old bug produced ~0.005 rad/s, too small to move. 10° off should be well above 0.1 rad/s.
        assertTrue(Math.abs(aim.calculate(true, 10.0)) > 0.1);
    }

    @Test
    void justOutsideToleranceStillOvercomesFriction() {
        // P alone gives ~0.17 rad/s at 2.5° — too little to beat static friction (sim finding)
        double rate = aim.calculate(true, AimController.AIM_TOLERANCE_DEG + 1.0);
        assertEquals(-AimController.MIN_ROT_RAD_S, rate, 1e-9);
        aim.reset();
        assertEquals(AimController.MIN_ROT_RAD_S, aim.calculate(true, -(AimController.AIM_TOLERANCE_DEG + 1.0)), 1e-9);
    }

    @Test
    void insideToleranceUsesGentleProportionalTurn() {
        double rate = aim.calculate(true, 1.0); // inside tolerance, outside deadzone
        assertTrue(rate < 0 && Math.abs(rate) < AimController.MIN_ROT_RAD_S);
    }

    @Test
    void sweepingThroughTheBandIsNotAimed() {
        // Overshooting: tx moves 0.4° per loop across the ±1.5° band — never "aimed"
        for (double tx = 1.4; tx >= -1.4; tx -= 0.4) {
            aim.calculate(true, tx);
            assertFalse(aim.isAimed(), "moving fast through the band at tx=" + tx);
        }
    }

    @Test
    void insideDeadzoneDoesNotTurn() {
        assertEquals(0.0, aim.calculate(true, AimController.TX_DEADZONE_DEG / 2));
    }

    @Test
    void rateIsClampedToMax() {
        assertEquals(-AimController.MAX_ROT_RAD_S, aim.calculate(true, 60.0), 1e-9);
        aim.reset();
        assertEquals(AimController.MAX_ROT_RAD_S, aim.calculate(true, -60.0), 1e-9);
    }

    @Test
    void aimedOnlyAfterSettling() {
        for (int i = 0; i < AimController.SETTLE_LOOPS - 1; i++) {
            aim.calculate(true, 0.5);
            assertFalse(aim.isAimed(), "not aimed yet after " + (i + 1) + " loops");
        }
        aim.calculate(true, 0.5);
        assertTrue(aim.isAimed());
    }

    @Test
    void leavingToleranceClearsAimed() {
        for (int i = 0; i < AimController.SETTLE_LOOPS; i++) aim.calculate(true, 0.5);
        assertTrue(aim.isAimed());

        aim.calculate(true, AimController.AIM_TOLERANCE_DEG + 2.0);
        assertFalse(aim.isAimed());
    }

    @Test
    void losingTargetClearsAimed() {
        for (int i = 0; i < AimController.SETTLE_LOOPS; i++) aim.calculate(true, 0.5);
        aim.calculate(false, 0.0);
        assertFalse(aim.isAimed());
    }
}
