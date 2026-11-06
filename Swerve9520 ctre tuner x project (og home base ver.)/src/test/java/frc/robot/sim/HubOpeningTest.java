package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The HUB scoring opening (2026 Field Dimension Drawings): a hexagon 41.73 in
 * across the flats (inside), flat sides facing ±Y, centered on the HUB. A FUEL
 * ball (5.91 in) scores only if it crosses the rim without touching it.
 */
class HubOpeningTest {
    private static final double IN = 0.0254;
    private static FieldModel field;

    @BeforeAll
    static void loadField() {
        field = new FieldModel();
    }

    @Test
    void openingSizeComesFromTheFieldFile() {
        assertEquals(41.73 * IN, field.hubOpeningAcrossFlatsM, 1e-4);
    }

    @Test
    void hexagonEdgesFlatsAndCorners() {
        double r = 0.5, R = 2 * r / Math.sqrt(3); // inradius, circumradius
        double e = 1e-6;
        assertTrue(FieldModel.insideHexagon(0, r - e, r), "just inside the ±Y flat");
        assertFalse(FieldModel.insideHexagon(0, r + e, r), "just outside the ±Y flat");
        assertTrue(FieldModel.insideHexagon(R - e, 0, r), "just inside the +X corner");
        assertFalse(FieldModel.insideHexagon(-R - e, 0, r), "just outside the -X corner");
        double a = Math.toRadians(30); // the slanted flats face ±30° from X
        assertTrue(FieldModel.insideHexagon((r - e) * Math.cos(a), -(r - e) * Math.sin(a), r));
        assertFalse(FieldModel.insideHexagon((r + e) * Math.cos(a), -(r + e) * Math.sin(a), r));
        assertFalse(FieldModel.insideHexagon(0, 0, 0), "no room at all");
    }

    @Test
    void ballMustClearTheRim() {
        Translation2d hub = field.hubCenter(false);
        double room = 41.73 * IN / 2 - SimRobotConstants.kFuelRadiusM; // ≈ 0.455 m
        // shooting from the alliance zone along +X: a sideways (Y) miss meets a flat
        assertTrue(field.inHubOpening(false, hub.getX(), hub.getY() + room - 0.005, SimRobotConstants.kFuelRadiusM));
        assertFalse(field.inHubOpening(false, hub.getX(), hub.getY() + room + 0.005, SimRobotConstants.kFuelRadiusM));
        // the same offset is fine for a ball that wouldn't touch the rim (no radius)
        assertTrue(field.inHubOpening(false, hub.getX(), hub.getY() + room + 0.005, 0.0));
    }

    @Test
    void aimErrorThatStillScores() {
        // 3 m in front of the blue HUB, shooting toward +X: the ball crosses the rim
        // 3 m along the shooter direction. 8° off → 0.42 m sideways (scores), 10° → 0.52 m (rim).
        Translation2d hub = field.hubCenter(false);
        for (double[] c : new double[][] { { 8, 1 }, { 10, 0 } }) {
            double a = Math.toRadians(c[0]);
            double x = hub.getX() - 3 + 3 * Math.cos(a), y = hub.getY() + 3 * Math.sin(a);
            assertEquals(c[1] == 1, field.inHubOpening(false, x, y, SimRobotConstants.kFuelRadiusM), c[0] + "° off");
        }
    }

    @Test
    void redHubIsTheSameShape() {
        Translation2d blue = field.hubCenter(false), red = field.hubCenter(true);
        for (double[] d : new double[][] { { 0.3, 0.2 }, { 0.5, 0.0 }, { 0.0, 0.46 } }) {
            assertEquals(field.inHubOpening(false, blue.getX() + d[0], blue.getY() + d[1], SimRobotConstants.kFuelRadiusM),
                         field.inHubOpening(true, red.getX() - d[0], red.getY() - d[1], SimRobotConstants.kFuelRadiusM));
        }
    }
}
