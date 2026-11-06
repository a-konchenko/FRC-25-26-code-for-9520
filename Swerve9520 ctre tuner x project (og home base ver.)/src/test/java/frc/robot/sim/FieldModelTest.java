package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

/**
 * The fields the simulator can run on: the 2026 game field (unchanged) and
 * B8, Team 9520's practice space — a 15 ft × 11 ft rectangle with nothing
 * on it yet. Reads deploy/sim/field-2026.json and deploy/sim/field-b8.json.
 */
class FieldModelTest {
    private static FieldModel b8;

    @BeforeAll
    static void loadFields() {
        b8 = new FieldModel(FieldModel.B8);
    }

    @Test
    void b8IsExactly15By11Feet() {
        assertEquals(Units.feetToMeters(15.0), b8.length, 1e-9);
        assertEquals(Units.feetToMeters(11.0), b8.width, 1e-9);
        assertEquals(4.572, b8.length, 1e-9);
        assertEquals(3.3528, b8.width, 1e-9);
        // the (empty) tag layout carries the same size, so anything reading it agrees
        assertEquals(b8.length, b8.tags.getFieldLength(), 1e-9);
        assertEquals(b8.width, b8.tags.getFieldWidth(), 1e-9);
        assertEquals(FieldModel.B8, b8.id);
    }

    @Test
    void b8HasNoTargetsYet() {
        assertTrue(b8.tags.getTags().isEmpty(), "no AprilTags on B8 yet");
        assertFalse(b8.hasGamePieces(), "no HUBs, so no FUEL");
        assertTrue(b8.fuelSolids.isEmpty());
        assertNull(b8.box("blue-hub"));
        // nothing blocks a sightline across the whole room
        assertFalse(b8.blocked(0.1, 0.1, b8.length - 0.1, b8.width - 0.1));
    }

    @Test
    void b8StartsInTheMiddleFacingPlusX() {
        Pose2d start = b8.defaultStartPose();
        assertEquals(b8.length / 2, start.getX(), 1e-9);
        assertEquals(b8.width / 2, start.getY(), 1e-9);
        assertEquals(0.0, start.getRotation().getRadians(), 1e-9);
    }

    @Test
    void b8StagesNoFuel() {
        FieldFuel floor = new FieldFuel(b8);
        floor.restage(9520, 0, 0, 0);
        assertEquals(0, floor.total());
        assertEquals(0, floor.onField());
        floor.step(0.02, null, 0);
        assertEquals(0, floor.onField());
        assertEquals(0, floor.positions().length);
    }

    @Test
    void the2026FieldIsUnchanged() {
        FieldModel game = new FieldModel();
        assertEquals(FieldModel.REBUILT_2026, game.id);
        assertTrue(game.hasGamePieces());
        assertFalse(game.tags.getTags().isEmpty());
        assertNotNull(game.box("blue-hub"), "elements from field-2026.json");
        assertTrue(game.length > 16 && game.width > 8);
        double[] s = SimRobotConstants.kDefaultStartPose;
        assertEquals(s[0], game.defaultStartPose().getX(), 1e-9);
        assertEquals(s[1], game.defaultStartPose().getY(), 1e-9);
    }

    @Test
    void unknownFieldIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new FieldModel("mars"));
    }
}
