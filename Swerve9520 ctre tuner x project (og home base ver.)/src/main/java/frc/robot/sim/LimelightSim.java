package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.VisionSubsystem;

/**
 * ★ NEW (sim only) — a Limelight, writing the same NetworkTables entries the
 * real camera does, so VisionSubsystem can't tell the difference.
 *
 * MEGATAG2 (LL3, table "limelight-three"):
 *   Reads robot_orientation_set (the heading YOUR CODE sends) and solves the
 *   robot position with it, like real MegaTag2 — so a heading error in the
 *   robot code turns into a position error, and a wrong mount (the camera
 *   isn't where the Limelight config says) turns into a bias.
 *   Publishes botpose_orb_wpiblue = [x, y, 0, 0, 0, yaw, latency ms, tag
 *   count, tag span, avg distance, avg area], tl, cl, hb.
 *
 * TARGETING (LL4, table "limelight-four"):
 *   Reads the pipeline the robot code selected (0 Hub Blue, 1 Hub Red,
 *   2 Trench Blue, 3 Trench Red). If any of that pipeline's tags is visible,
 *   publishes tv=1 and tx/ty to the aim point (hub opening at
 *   VisionSubsystem.HUB_HEIGHT_M, or the nearest trench tag), plus ta, hb.
 *   tx > 0 = target right of the crosshair (Limelight convention).
 *
 * Each frame uses the TRUE robot pose at capture time (now − latency).
 */
public class LimelightSim {
    public enum Mode { MEGATAG2, TARGETING }

    private static final int[][] PIPELINE_TAGS = {
        { 18, 19, 20, 21, 24, 25, 26, 27 }, // 0 Hub Blue
        { 2, 3, 4, 5, 8, 9, 10, 11 },       // 1 Hub Red
        { 17, 22, 23, 28 },                 // 2 Trench Blue
        { 1, 6, 7, 12 },                    // 3 Trench Red
    };
    private static final double TAG_SIZE_M = 0.1651; // 6.5 in

    public final String key;      // "ll3" / "ll4" (UI + settings)
    public final Mode mode;
    private final NetworkTable table;
    private final double[] mount; // configured mount: x, y, z, roll, pitch(up+), yaw (deg)

    // ── settings (UI: /Sim/Control/cam/<key>/...) ───────────────────────────
    public boolean enabled = true;
    public double hFovDeg, vFovDeg, maxRangeM;
    public double noise = 1.0;          // multiplier on the noise model
    public double latencyMs;            // capture → NT
    public double dropPct = 0.0;        // % of frames lost
    public double mountErrX = 0, mountErrY = 0, mountErrYawDeg = 0; // physical − configured

    private final Random rng = new Random(95201);
    private double heartbeat = 0;
    private double suppressUntil = 0;
    private double frameLatencyMs = 0;

    // ── for the UI ──────────────────────────────────────────────────────────
    public final List<Integer> visibleIds = new ArrayList<>();
    public double[] lastFrame = new double[0]; // MT2: {x, y, θ, tags}; targeting: {tv, tx, ty, ta}
    public int frameCount = 0;

    public LimelightSim(String key, Mode mode, String tableName, double[] mount,
                        double hFovDeg, double vFovDeg, double maxRangeM, double latencyMs) {
        this.key = key;
        this.mode = mode;
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
        this.mount = mount;
        this.hFovDeg = hFovDeg;
        this.vFovDeg = vFovDeg;
        this.maxRangeM = maxRangeM;
        this.latencyMs = latencyMs;
    }

    public double[] getMount() { return mount; }

    /**
     * The robot was teleported (place / kidnap / play). Skip frames briefly:
     * a frame timestamped before the teleport would be compared, by the pose
     * estimator, with where the robot was BEFORE it — a bogus jump that a
     * real (non-teleporting) robot never sees.
     */
    public void suppress(double seconds) {
        suppressUntil = Timer.getFPGATimestamp() + seconds;
    }

    public void readSettings(SimNT nt) {
        String p = "cam/" + key + "/";
        enabled        = nt.setting(p + "enabled", enabled ? 1 : 0) > 0.5;
        noise          = nt.setting(p + "noise", noise);
        latencyMs      = nt.setting(p + "latencyMs", latencyMs);
        dropPct        = nt.setting(p + "dropPct", dropPct);
        hFovDeg        = nt.setting(p + "hFovDeg", hFovDeg);
        maxRangeM      = nt.setting(p + "maxRangeM", maxRangeM);
        mountErrX      = nt.setting(p + "mountErrX", mountErrX);
        mountErrY      = nt.setting(p + "mountErrY", mountErrY);
        mountErrYawDeg = nt.setting(p + "mountErrYawDeg", mountErrYawDeg);
    }

    /** Call once per robot loop. */
    public void update(WorldModel world, FieldModel field, Pose2d estimate) {
        visibleIds.clear();
        if (!enabled) { // a dead camera: no heartbeat → VisionSubsystem reports it offline
            lastFrame = new double[0];
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (now < suppressUntil) {
            table.getEntry("hb").setDouble(++heartbeat); // still alive, just no new frame
            return;
        }
        if (rng.nextDouble() * 100 < dropPct) return;

        Pose2d truth = world.truthAt(now - latencyMs / 1000.0);
        // Physical camera = configured mount + mount error
        double[] cam = CameraGeometry.cameraOnField(truth, mount[0] + mountErrX, mount[1] + mountErrY,
            mount[2], mount[5] + mountErrYawDeg, mount[4]);

        heartbeat++;
        frameCount++;
        // Real latency jitters a little. It also keeps every frame's values unique —
        // NetworkTables drops an identical repeated value, so with zero noise and a
        // parked robot the robot code would otherwise stop seeing new frames.
        frameLatencyMs = latencyMs + rng.nextDouble() * 0.5;
        double cl = Math.min(12.0, frameLatencyMs);
        table.getEntry("tl").setDouble(frameLatencyMs - cl);
        table.getEntry("cl").setDouble(cl);
        if (mode == Mode.MEGATAG2) megaTag2(field, cam, estimate);
        else targeting(field, cam);
        table.getEntry("hb").setDouble(heartbeat);
    }

    // ── LL3 ────────────────────────────────────────────────────────────────
    private void megaTag2(FieldModel field, double[] cam, Pose2d estimate) {
        List<Pose3d> tags = CameraGeometry.visibleTags(field, cam, hFovDeg, vFovDeg, maxRangeM, visibleIds, null);
        if (tags.isEmpty()) {
            table.getEntry("tv").setDouble(0);
            table.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[11]);
            lastFrame = new double[0];
            return;
        }
        // Heading the robot code told the Limelight (falls back to its estimate)
        double[] ori = table.getEntry("robot_orientation_set").getDoubleArray(new double[0]);
        double yawTold = Math.toRadians(ori.length > 0 ? ori[0] : estimate.getRotation().getDegrees());
        // What the Limelight believes about its own mount (its config — no error)
        double camYawBelieved = yawTold + Math.toRadians(mount[5]);
        double cT = Math.cos(yawTold), sT = Math.sin(yawTold);
        double mxW = mount[0] * cT - mount[1] * sT, myW = mount[0] * sT + mount[1] * cT;

        double sx = 0, sy = 0, sumDist = 0, sumArea = 0;
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (Pose3d t : tags) {
            // Tag position in the PHYSICAL camera's frame (what the sensor measures) + noise
            double dx = t.getX() - cam[0], dy = t.getY() - cam[1];
            double d = Math.hypot(dx, dy);
            double c = Math.cos(cam[3]), s = Math.sin(cam[3]);
            double vf = dx * c + dy * s, vl = -dx * s + dy * c;
            double sigma = noise * 0.008 * d * d / Math.sqrt(tags.size()); // m, grows with distance²
            vf += rng.nextGaussian() * sigma;
            vl += rng.nextGaussian() * sigma;
            // Solve with the BELIEVED camera heading/position: tag = camBelieved + R(camYaw)·v
            double cb = Math.cos(camYawBelieved), sb = Math.sin(camYawBelieved);
            double camX = t.getX() - (vf * cb - vl * sb);
            double camY = t.getY() - (vf * sb + vl * cb);
            sx += camX - mxW;
            sy += camY - myW;
            sumDist += d;
            sumArea += area(d);
            minX = Math.min(minX, t.getX()); maxX = Math.max(maxX, t.getX());
            minY = Math.min(minY, t.getY()); maxY = Math.max(maxY, t.getY());
        }
        int n = tags.size();
        double x = sx / n, y = sy / n;
        double span = Math.hypot(maxX - minX, maxY - minY);
        table.getEntry("tv").setDouble(1);
        table.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[] {
            x, y, 0, 0, 0, Math.toDegrees(yawTold), frameLatencyMs, n, span, sumDist / n, sumArea / n });
        lastFrame = new double[] { x, y, yawTold, n };
    }

    // ── LL4 ────────────────────────────────────────────────────────────────
    private void targeting(FieldModel field, double[] cam) {
        int pipe = (int) table.getEntry("pipeline").getDouble(0);
        pipe = Math.max(0, Math.min(PIPELINE_TAGS.length - 1, pipe));
        List<Integer> ids = new ArrayList<>();
        for (int id : PIPELINE_TAGS[pipe]) ids.add(id);
        List<Pose3d> tags = CameraGeometry.visibleTags(field, cam, hFovDeg, vFovDeg, maxRangeM, visibleIds, ids);
        table.getEntry("getpipe").setDouble(pipe);

        double[] v = null;
        double dist = 0;
        if (!tags.isEmpty()) {
            double px, py, pz;
            if (pipe <= 1) { // hub: aim at the opening above the hub center
                Translation2d hub = field.hubCenter(pipe == 1);
                px = hub.getX(); py = hub.getY(); pz = VisionSubsystem.HUB_HEIGHT_M;
            } else {         // trench: nearest visible trench tag
                Pose3d best = tags.get(0);
                for (Pose3d t : tags) if (Math.hypot(t.getX() - cam[0], t.getY() - cam[1]) < Math.hypot(best.getX() - cam[0], best.getY() - cam[1])) best = t;
                px = best.getX(); py = best.getY(); pz = best.getZ();
            }
            double[] tv = CameraGeometry.view(cam, px, py, pz);
            if (CameraGeometry.inFov(tv, hFovDeg, vFovDeg)) { v = tv; dist = tv[3]; }
        }
        if (v == null) {
            table.getEntry("tv").setDouble(0);
            table.getEntry("tx").setDouble(0);
            table.getEntry("ty").setDouble(0);
            table.getEntry("ta").setDouble(0);
            lastFrame = new double[] { 0, 0, 0, 0 };
            return;
        }
        double sigmaDeg = 0.05 * noise;
        double tx = -Math.toDegrees(v[0]) + rng.nextGaussian() * sigmaDeg;
        double ty = Math.toDegrees(v[1]) + rng.nextGaussian() * sigmaDeg;
        double ta = area(dist);
        table.getEntry("tv").setDouble(1);
        table.getEntry("tx").setDouble(tx);
        table.getEntry("ty").setDouble(ty);
        table.getEntry("ta").setDouble(ta);
        lastFrame = new double[] { 1, tx, ty, ta };
    }

    /** Approximate tag area as % of the image. */
    private double area(double d) {
        double w = 2 * d * Math.tan(Math.toRadians(hFovDeg / 2)), h = 2 * d * Math.tan(Math.toRadians(vFovDeg / 2));
        return 100.0 * TAG_SIZE_M * TAG_SIZE_M / Math.max(1e-6, w * h);
    }
}
