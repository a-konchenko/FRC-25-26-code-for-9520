package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * ★ NEW (sim only) — what a camera can see.
 *
 * A camera is described in the robot frame (x forward, y left, z up; yaw
 * left-positive, pitch UP-positive, degrees). A tag is visible when it is in
 * front of the lens, inside the horizontal and vertical field of view, within
 * range, facing the camera (< 75° off its normal), and the 2D sightline does
 * not pass through the HUB or TOWER.
 */
public final class CameraGeometry {
    private CameraGeometry() {}

    public static final double kMaxViewAngleDeg = 110.0;

    /** Camera pose on the field: {x, y, z, yawRad, pitchRad}. */
    public static double[] cameraOnField(Pose2d robot, double mx, double my, double mz, double yawDeg, double pitchDeg) {
        double c = robot.getRotation().getCos(), s = robot.getRotation().getSin();
        return new double[] {
            robot.getX() + mx * c - my * s,
            robot.getY() + mx * s + my * c,
            mz,
            robot.getRotation().getRadians() + Math.toRadians(yawDeg),
            Math.toRadians(pitchDeg),
        };
    }

    /**
     * Angles of a field point as seen by the camera: {horizontal (left +) rad,
     * vertical (up +) rad, forward distance m, horizontal distance m}.
     */
    public static double[] view(double[] cam, double px, double py, double pz) {
        double dx = px - cam[0], dy = py - cam[1], dz = pz - cam[2];
        double c = Math.cos(cam[3]), s = Math.sin(cam[3]);
        double fwd = dx * c + dy * s;
        double left = -dx * s + dy * c;
        double cp = Math.cos(cam[4]), sp = Math.sin(cam[4]);
        double fwdP = fwd * cp + dz * sp;
        double upP = -fwd * sp + dz * cp;
        return new double[] { Math.atan2(left, fwdP), Math.atan2(upP, fwdP), fwdP, Math.hypot(dx, dy) };
    }

    public static boolean inFov(double[] v, double hFovDeg, double vFovDeg) {
        return v[2] > 0.05
            && Math.abs(v[0]) <= Math.toRadians(hFovDeg / 2)
            && Math.abs(v[1]) <= Math.toRadians(vFovDeg / 2);
    }

    /** IDs and poses of tags this camera can see. */
    public static List<Pose3d> visibleTags(FieldModel field, double[] cam, double hFovDeg, double vFovDeg,
                                           double maxRange, List<Integer> idsOut, List<Integer> onlyIds) {
        List<Pose3d> out = new ArrayList<>();
        for (var tag : field.tags.getTags()) {
            if (onlyIds != null && !onlyIds.contains(tag.ID)) continue;
            Pose3d p = tag.pose;
            double[] v = view(cam, p.getX(), p.getY(), p.getZ());
            if (!inFov(v, hFovDeg, vFovDeg) || v[3] > maxRange) continue;
            // Tag must face the camera
            double nx = Math.cos(p.getRotation().getZ()), ny = Math.sin(p.getRotation().getZ());
            double tcx = cam[0] - p.getX(), tcy = cam[1] - p.getY();
            double d = Math.hypot(tcx, tcy);
            if (d < 1e-6 || (nx * tcx + ny * tcy) / d < Math.cos(Math.toRadians(kMaxViewAngleDeg))) continue;
            // Sightline (stop 5 cm short of the tag: the tag sits ON an element's face)
            double ex = p.getX() + tcx / d * 0.05, ey = p.getY() + tcy / d * 0.05;
            if (field.blocked(cam[0], cam[1], ex, ey)) continue;
            out.add(p);
            if (idsOut != null) idsOut.add(tag.ID);
        }
        return out;
    }
}
