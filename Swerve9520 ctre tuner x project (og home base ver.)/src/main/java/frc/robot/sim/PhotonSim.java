package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.subsystems.VisionSubsystem;

/**
 * ★ NEW (sim only) — the 4 PhotonVision cameras on the Jetson, simulated with
 * PhotonLib's own VisionSystemSim / PhotonCameraSim. Uses the camera names and
 * robot-to-camera transforms from VisionSubsystem.PHOTON_CAMERAS, so the robot
 * code's PhotonPoseEstimator (multi-tag on the coprocessor) runs unchanged.
 *
 * Off by default (the Jetson is optional). Camera properties are estimates for
 * the Arducam global-shutter cameras: 1280×800, 100° diagonal FOV, 30 fps,
 * ~30 ms latency — TO MEASURE / calibrate.
 */
public class PhotonSim {
    public static final double kHFovDeg = 90, kVFovDeg = 62, kMaxRangeM = 6.0;

    private final VisionSystemSim visionSim = new VisionSystemSim("sim");
    private final List<PhotonCameraSim> cameraSims = new ArrayList<>();
    private final List<Transform3d> configured = new ArrayList<>();
    public final List<String> names = new ArrayList<>();

    public boolean enabled = false;
    public double noise = 1.0;
    public double latencyMs = 30;
    public double mountErrX = 0, mountErrY = 0, mountErrYawDeg = 0;
    private double appliedErrX = 0, appliedErrY = 0, appliedErrYaw = 0, appliedNoise = -1, appliedLatency = -1;
    private double suppressUntil = 0;

    public PhotonSim(VisionSubsystem vision, FieldModel field) {
        visionSim.addAprilTags(field.tags);
        List<PhotonCamera> cams = vision.getPhotonCameras();
        for (int i = 0; i < cams.size() && i < VisionSubsystem.PHOTON_CAMERAS.size(); i++) {
            var cfg = VisionSubsystem.PHOTON_CAMERAS.get(i);
            var sim = new PhotonCameraSim(cams.get(i), properties(), field.tags);
            sim.enableRawStream(false);
            sim.enableProcessedStream(false);
            sim.setMaxSightRange(kMaxRangeM);
            visionSim.addCamera(sim, cfg.robotToCamera());
            cameraSims.add(sim);
            configured.add(cfg.robotToCamera());
            names.add(cfg.name());
        }
    }

    private SimCameraProperties properties() {
        var p = new SimCameraProperties();
        p.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
        p.setCalibError(0.25 * Math.max(noise, 0.01), 0.08 * Math.max(noise, 0.01));
        p.setFPS(30);
        p.setAvgLatencyMs(latencyMs);
        p.setLatencyStdDevMs(5);
        return p;
    }

    public void readSettings(SimNT nt) {
        enabled        = nt.setting("cam/pv/enabled", enabled ? 1 : 0) > 0.5;
        noise          = nt.setting("cam/pv/noise", noise);
        latencyMs      = nt.setting("cam/pv/latencyMs", latencyMs);
        mountErrX      = nt.setting("cam/pv/mountErrX", mountErrX);
        mountErrY      = nt.setting("cam/pv/mountErrY", mountErrY);
        mountErrYawDeg = nt.setting("cam/pv/mountErrYawDeg", mountErrYawDeg);
    }

    /** Robot teleported: restart PhotonLib's pose history there and skip frames briefly (see LimelightSim.suppress). */
    public void teleported(Pose2d truth, double seconds) {
        visionSim.resetRobotPose(truth);
        suppressUntil = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + seconds;
    }

    /** Call once per loop with the CURRENT true pose (PhotonLib applies latency itself). */
    public void update(Pose2d truthNow) {
        if (!enabled || cameraSims.isEmpty()) return;
        if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() < suppressUntil) {
            visionSim.resetRobotPose(truthNow);
            return;
        }
        applySettings();
        visionSim.update(truthNow);
    }

    private void applySettings() {
        if (noise != appliedNoise || latencyMs != appliedLatency) {
            for (var sim : cameraSims) {
                sim.prop.setCalibError(0.25 * Math.max(noise, 0.01), 0.08 * Math.max(noise, 0.01));
                sim.prop.setAvgLatencyMs(latencyMs);
            }
            appliedNoise = noise;
            appliedLatency = latencyMs;
        }
        if (mountErrX != appliedErrX || mountErrY != appliedErrY || mountErrYawDeg != appliedErrYaw) {
            // Physical camera = configured transform + error, in the ROBOT frame (X = robot
            // forward, Y = robot left, yaw about the robot's vertical axis) — the same way
            // the UI draws it. (Transform3d.plus would apply it in each camera's own frame.)
            for (int i = 0; i < cameraSims.size(); i++) {
                Transform3d cfg = configured.get(i);
                Transform3d physical = new Transform3d(
                    cfg.getTranslation().plus(new Translation3d(mountErrX, mountErrY, 0)),
                    cfg.getRotation().rotateBy(new Rotation3d(0, 0, Math.toRadians(mountErrYawDeg))));
                visionSim.adjustCamera(cameraSims.get(i), physical);
            }
            appliedErrX = mountErrX; appliedErrY = mountErrY; appliedErrYaw = mountErrYawDeg;
        }
    }

    /** Configured mounts as {x, y, z, roll, pitch(up+), yaw} degrees, for the UI. */
    public double[] mount(int i) {
        Transform3d t = configured.get(i);
        return new double[] { t.getX(), t.getY(), t.getZ(), 0,
            -Math.toDegrees(t.getRotation().getY()), Math.toDegrees(t.getRotation().getZ()) };
    }

    public int count() { return cameraSims.size(); }
}
