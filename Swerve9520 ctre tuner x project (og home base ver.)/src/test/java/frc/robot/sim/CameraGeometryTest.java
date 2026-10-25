package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Unit tests for what a simulated camera can see (field of view, range,
 * tag facing, and sightlines blocked by the HUB but not the TOWER).
 * Uses the real 2026 AndyMark tag layout and deploy/sim/field-2026.json.
 */
class CameraGeometryTest {
    private static FieldModel field;
    // Limelight 3 placeholder mount: 0.30 m forward, 0.25 m up, tilted 20° up
    private static final double MX = 0.30, MY = 0.0, MZ = 0.25, YAW = 0.0, PITCH = 20.0;
    private static final double HFOV = 62.5, VFOV = 48.9;

    @BeforeAll
    static void loadField() {
        field = new FieldModel();
    }

    private static List<Integer> seen(double x, double y, double headingDeg, double range) {
        double[] cam = CameraGeometry.cameraOnField(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)), MX, MY, MZ, YAW, PITCH);
        List<Integer> ids = new ArrayList<>();
        CameraGeometry.visibleTags(field, cam, HFOV, VFOV, range, ids, null);
        return ids;
    }

    @Test
    void towerTagsAreVisibleFromInFront() {
        // Facing the blue alliance wall, 2.5 m out, lined up with the blue TOWER (tags 31/32)
        List<Integer> ids = seen(2.5, 3.95, 180, 6.0);
        assertTrue(ids.contains(31) && ids.contains(32), "tower tags should be visible, saw " + ids);
    }

    @Test
    void hubBlocksTagsBehindIt() {
        // Same line, but from the far side of the blue HUB: the HUB is in the way
        List<Integer> ids = seen(6.2, 3.95, 180, 10.0);
        assertFalse(ids.contains(31) || ids.contains(32), "HUB should block tower tags, saw " + ids);
    }

    @Test
    void hubFaceTagsAreVisible() {
        // In front of the blue HUB's alliance-side face (tags 25/26 face the blue wall)
        List<Integer> ids = seen(2.6, 4.0, 0, 6.0);
        assertTrue(ids.contains(25) && ids.contains(26), "hub face tags should be visible, saw " + ids);
    }

    @Test
    void tagsBehindTheCameraAreNotVisible() {
        // Facing away from the blue wall: tower tags are behind the camera
        List<Integer> ids = seen(2.5, 3.95, 0, 6.0);
        assertFalse(ids.contains(31) || ids.contains(32), "tags behind the camera, saw " + ids);
    }

    @Test
    void tagsOutOfRangeAreNotVisible() {
        List<Integer> ids = seen(2.5, 3.95, 180, 1.0); // range 1 m, tags ~2.2 m away
        assertTrue(ids.isEmpty(), "out of range, saw " + ids);
    }
}
