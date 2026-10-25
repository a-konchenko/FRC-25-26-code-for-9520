// ============================================================
//  VisionSubsystem.java  —  FRC Team 9520  —  2026 REBUILT
// ============================================================
//
//  CAMERA ARCHITECTURE:
//    4x Arducam GS (166deg FOV) -> Jetson PhotonVision [OPTIONAL]
//    Limelight 3 -> MegaTag2 pose + color driver stream
//    Limelight 4 -> shooter targeting (tx/ty)
//
//  ELASTIC DASHBOARD INTEGRATION:
//    Camera streams registered via CameraServer -> auto-discovered
//    by Elastic under CameraPublisher in the Add Widget dialog.
//    Status booleans published under "Status/" for indicator widgets.
//    Elastic notifications sent when Jetson/LLs connect/disconnect.
//    Field2d published for robot position widget.
// ============================================================

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cameraserver.CameraServer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import frc.robot.commands.Elastic;
import frc.robot.commands.Elastic.NotificationLevel;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

    @FunctionalInterface
    public interface VisionConsumer {
        void addVisionMeasurement(Pose2d pose, double ts, Matrix<N3, N1> stdDevs);
    }

    public enum TargetMode {
        HUB_BLUE(0), HUB_RED(1), TRENCH_BLUE(2), TRENCH_RED(3);
        public final int pipelineIndex;
        TargetMode(int i) { this.pipelineIndex = i; }
    }

    /** A PhotonVision camera: name (as in the PhotonVision UI) and where it sits on the robot. */
    public record CamConfig(String name, Transform3d robotToCamera) {}

    // -- Camera transforms -- replace with measured values
    public static final List<CamConfig> PHOTON_CAMERAS = List.of(
        new CamConfig("Front Camera", new Transform3d(
            new Translation3d( 0.35,  0.00, 0.50), new Rotation3d(0, Math.toRadians(-10), 0))),
        new CamConfig("Back Camera",  new Transform3d(
            new Translation3d(-0.35,  0.00, 0.50), new Rotation3d(0, Math.toRadians(-10), Math.toRadians(180)))),
        new CamConfig("Left Camera",  new Transform3d(
            new Translation3d( 0.00,  0.35, 0.50), new Rotation3d(0, Math.toRadians(-10), Math.toRadians(90)))),
        new CamConfig("Right Camera", new Transform3d(
            new Translation3d( 0.00, -0.35, 0.50), new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-90))))
    );

    private static final String JETSON_IP = "10.95.20.11";
    private static final String LL3_IP    = "10.95.20.12";
    private static final String LL4_IP    = "10.95.20.13";
    private static final int[]  PV_PORTS  = { 1182, 1184, 1186, 1188 };
    private static final String LL3_NAME  = "limelight-three";
    private static final String LL4_NAME  = "limelight-four";

    // -- Std devs -- tune after field testing
    private static final Matrix<N3, N1> PV_MULTI_TAG  = VecBuilder.fill(0.4, 0.4, 0.6);
    private static final Matrix<N3, N1> PV_SINGLE_TAG = VecBuilder.fill(0.9, 0.9, 1.2);
    // MegaTag2's yaw is just the heading WE sent it (robot_orientation_set) — not a
    // measurement — so it must never correct our heading (θ std dev ≈ infinite).
    // BUG FIX (2026-10, found in the simulator): θ was 0.8 rad, so while turning
    // our own delayed heading pulled the estimate's heading off, and MegaTag2
    // positions (which use that heading) went wrong by up to ~1 m.
    private static final Matrix<N3, N1> LL3_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 9999999);
    /** Ignore MegaTag2 while spinning faster than this (Limelight guidance). */
    private static final double MAX_MT2_YAW_RATE_DEG_S = 360.0;
    private static final double MAX_SINGLE_TAG_DIST_M  = 4.0;

    // -- LL4 distance calc
    public static final double LL4_HEIGHT_M  = 0.55;
    public static final double LL4_PITCH_DEG = 25.0;
    public static final double HUB_HEIGHT_M  = 1.83;

    private static final double JETSON_TIMEOUT_S = 3.0;
    private static final double LL_TIMEOUT_S     = 2.0;

    // -- State
    private final VisionConsumer            poseConsumer;
    private final Supplier<Rotation2d>      headingSupplier;
    private final List<PhotonPoseEstimator> photonEstimators;
    private final List<PhotonCamera>        photonCameras;
    private final AprilTagFieldLayout       fieldLayout;
    private final NetworkTable              ll3Table;
    private final NetworkTable              ll4Table;
    private final Field2d                   field = new Field2d();

    private TargetMode currentMode         = TargetMode.HUB_BLUE;
    private double     shooterTx           = 0.0;
    private double     shooterTy           = 0.0;
    private double     shooterTa           = 0.0;
    private boolean    shooterHasTarget    = false;
    private double     lastJetsonUpdateTime = 0.0;
    private double     lastLL3UpdateTime    = 0.0;
    private double     lastLL4UpdateTime    = 0.0;
    private double     lastLL3PoseTime      = 0.0;
    private double     lastLL3Heartbeat     = -1;
    private double     lastLL4Heartbeat     = -1;
    private long       lastLL3FrameTime     = 0;
    private double     lastHeadingDeg       = Double.NaN;
    private double     lastHeadingTime      = 0;
    private double     yawRateDegPerSec     = 0;
    private boolean    jetsonWasOnline     = false;
    private boolean    ll3WasOnline        = false;
    private boolean    ll4WasOnline        = false;
    private int        pvMeasurementsThisLoop = 0;

    // Reusable Elastic notifications (avoids GC churn)
    private final Elastic.Notification jetsonOnlineNotif = new Elastic.Notification(
        NotificationLevel.INFO, "Jetson ONLINE", "PhotonVision cameras active");
    private final Elastic.Notification jetsonOfflineNotif = new Elastic.Notification(
        NotificationLevel.WARNING, "Jetson OFFLINE", "Falling back to LL3 only", 5000);
    private final Elastic.Notification ll3OnlineNotif = new Elastic.Notification(
        NotificationLevel.INFO, "LL3 ONLINE", "MegaTag2 pose active");
    private final Elastic.Notification ll3OfflineNotif = new Elastic.Notification(
        NotificationLevel.ERROR, "LL3 OFFLINE", "No MegaTag2 pose!", 5000);
    private final Elastic.Notification ll4OnlineNotif = new Elastic.Notification(
        NotificationLevel.INFO, "LL4 ONLINE", "Shooter targeting active");
    private final Elastic.Notification ll4OfflineNotif = new Elastic.Notification(
        NotificationLevel.ERROR, "LL4 OFFLINE", "No shooter targeting!", 5000);

    public VisionSubsystem(VisionConsumer poseConsumer,
                           Supplier<Rotation2d> headingSupplier) {
        this.poseConsumer    = poseConsumer;
        this.headingSupplier = headingSupplier;
        this.ll3Table = NetworkTableInstance.getDefault().getTable(LL3_NAME);
        this.ll4Table = NetworkTableInstance.getDefault().getTable(LL4_NAME);
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        List<PhotonPoseEstimator> estimators = new ArrayList<>();
        List<PhotonCamera>        cameras    = new ArrayList<>();
        for (CamConfig cfg : PHOTON_CAMERAS) {
            try {
                var cam = new PhotonCamera(cfg.name());
                var est = new PhotonPoseEstimator(fieldLayout, cfg.robotToCamera());
                estimators.add(est);
                cameras.add(cam);
            } catch (Exception e) {
                System.out.println("[VisionSubsystem] Camera '" + cfg.name()
                    + "' init skipped: " + e.getMessage());
            }
        }
        this.photonEstimators = List.copyOf(estimators);
        this.photonCameras    = List.copyOf(cameras);

        setAllianceDefaultMode();
        registerCameraStreams();

        // Field2d widget -- drag "Field" from Add Widget in Elastic
        SmartDashboard.putData("Field", field);

        // Initialize status indicators
        SmartDashboard.putBoolean("Status/Jetson Online",      false);
        SmartDashboard.putBoolean("Status/Limelight 3 Online", false);
        SmartDashboard.putBoolean("Status/Limelight 4 Online", false);
        SmartDashboard.putBoolean("Status/LL4 Has Target",     false);
        SmartDashboard.putString ("Status/Target Mode",        currentMode.name());
        SmartDashboard.putString ("Status/Vision Mode",        "Initializing...");
    }

    /**
     * Register all camera MJPEG streams with CameraServer so they appear
     * under /CameraPublisher in NetworkTables. Elastic auto-discovers them
     * in Add Widget -> CameraPublisher.
     */
    private void registerCameraStreams() {
        String[] pvLabels = {"PV Front", "PV Back", "PV Left", "PV Right"};
        for (int i = 0; i < 4; i++) {
            try {
                HttpCamera cam = new HttpCamera(pvLabels[i],
                    "http://" + JETSON_IP + ":" + PV_PORTS[i] + "/stream.mjpeg",
                    HttpCameraKind.kMJPGStreamer);
                CameraServer.startAutomaticCapture(cam);
            } catch (Exception e) {
                System.out.println("[VisionSubsystem] Could not register " + pvLabels[i] + ": " + e.getMessage());
            }
        }

        try {
            HttpCamera ll3Cam = new HttpCamera("LL3 Driver",
                "http://" + LL3_IP + ":5800",
                HttpCameraKind.kMJPGStreamer);
            CameraServer.startAutomaticCapture(ll3Cam);
        } catch (Exception e) {
            System.out.println("[VisionSubsystem] Could not register LL3: " + e.getMessage());
        }

        try {
            HttpCamera ll4Cam = new HttpCamera("LL4 Shooter",
                "http://" + LL4_IP + ":5800",
                HttpCameraKind.kMJPGStreamer);
            CameraServer.startAutomaticCapture(ll4Cam);
        } catch (Exception e) {
            System.out.println("[VisionSubsystem] Could not register LL4: " + e.getMessage());
        }

        System.out.println("[VisionSubsystem] Camera streams registered with CameraServer.");
        System.out.println("  In Elastic: Add Widget -> CameraPublisher -> drag a camera to your layout.");
    }
    //  PERIODIC
    @Override
    public void periodic() {
        pvMeasurementsThisLoop = 0;
        updateRobotOrientationForLL3();
        updatePhotonVision();
        updateLimelight3();
        updateLimelight4();

        if (pvMeasurementsThisLoop > 0)
            lastJetsonUpdateTime = Timer.getFPGATimestamp();

        checkConnectionStatus();
        publishTelemetry();
    }

    private void checkConnectionStatus() {
        boolean jetsonOnline = isJetsonOnline();
        if (jetsonOnline != jetsonWasOnline) {
            System.out.println("[VisionSubsystem] Jetson " + (jetsonOnline ? "ONLINE" : "OFFLINE"));
            Elastic.sendNotification(jetsonOnline ? jetsonOnlineNotif : jetsonOfflineNotif);
            jetsonWasOnline = jetsonOnline;
        }

        boolean ll3Online = isLL3Online();
        if (ll3Online != ll3WasOnline) {
            System.out.println("[VisionSubsystem] Limelight 3 " + (ll3Online ? "ONLINE" : "OFFLINE"));
            Elastic.sendNotification(ll3Online ? ll3OnlineNotif : ll3OfflineNotif);
            ll3WasOnline = ll3Online;
        }

        boolean ll4Online = isLL4Online();
        if (ll4Online != ll4WasOnline) {
            System.out.println("[VisionSubsystem] Limelight 4 " + (ll4Online ? "ONLINE" : "OFFLINE"));
            Elastic.sendNotification(ll4Online ? ll4OnlineNotif : ll4OfflineNotif);
            ll4WasOnline = ll4Online;
        }
    }

    private void updateRobotOrientationForLL3() {
        double heading = headingSupplier.get().getDegrees();
        double now = Timer.getFPGATimestamp();
        if (!Double.isNaN(lastHeadingDeg) && now > lastHeadingTime) {
            double d = Math.IEEEremainder(heading - lastHeadingDeg, 360.0);
            // A jump this big in one loop is a pose reset (auto start, vision heading
            // correction), not real spinning — the robot can't turn 30° in 20 ms.
            yawRateDegPerSec = Math.abs(d) > 30.0 ? 0.0 : d / (now - lastHeadingTime);
        }
        lastHeadingDeg  = heading;
        lastHeadingTime = now;
        ll3Table.getEntry("robot_orientation_set")
                .setDoubleArray(new double[]{ heading, yawRateDegPerSec, 0, 0, 0, 0 });
    }

    private void updatePhotonVision() {
        if (photonEstimators.isEmpty()) return;
        for (int i = 0; i < photonEstimators.size(); i++) {
            var estimator = photonEstimators.get(i);
            var camera    = photonCameras.get(i);
            try {
                for (var result : camera.getAllUnreadResults()) {
                    var estOpt = estimator.estimateCoprocMultiTagPose(result);
                    estOpt.ifPresent(est -> {
                        var pose = est.estimatedPose.toPose2d();
                        if (!isOnField(pose)) return;

                        int tagCount = est.targetsUsed.size();
                        Matrix<N3,N1> std;
                        if (tagCount >= 2) {
                            std = PV_MULTI_TAG;
                        } else {
                            double d = est.targetsUsed.get(0)
                                .getBestCameraToTarget().getTranslation().getNorm();
                            if (d > MAX_SINGLE_TAG_DIST_M) return;
                            std = scaleByDistance(PV_SINGLE_TAG, d);
                        }
                        poseConsumer.addVisionMeasurement(pose, est.timestampSeconds, std);
                        pvMeasurementsThisLoop++;
                    });
                }
            } catch (Exception ignored) { }
        }
    }

    // botpose array layout (Limelight docs):
    //   [0..5] x, y, z, roll, pitch, yaw   [6] total latency ms   [7] tag count
    //   [8] tag span   [9] avg tag distance   [10] avg tag area
    private void updateLimelight3() {
        // Online = heartbeat still changing (works even when no tags are seen)
        double hb3 = ll3Table.getEntry("hb").getDouble(-1);
        if (hb3 >= 0 && hb3 != lastLL3Heartbeat) {
            lastLL3Heartbeat  = hb3;
            lastLL3UpdateTime = Timer.getFPGATimestamp();
        }

        NetworkTableEntry bpEntry = ll3Table.getEntry("botpose_orb_wpiblue");
        // Only use each camera frame once (the entry keeps its last value between frames)
        long changed = bpEntry.getLastChange();
        if (changed == lastLL3FrameTime) return;
        lastLL3FrameTime = changed;

        double[] bp = bpEntry.getDoubleArray(new double[0]);
        // BUG FIX (2026-10): was bp[6] < 1, but [6] is latency, not tag count
        if (bp.length < 8 || bp[7] < 1) return;
        // Spinning fast: the heading MegaTag2 used is already out of date → bad position
        if (Math.abs(yawRateDegPerSec) > MAX_MT2_YAW_RATE_DEG_S) return;
        var pose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));
        if (!isOnField(pose)) return;
        // NT change time (µs, robot clock) minus capture+pipeline latency
        double ts = changed / 1e6 - bp[6] / 1000.0;
        poseConsumer.addVisionMeasurement(pose, ts, LL3_STD_DEVS);
        lastLL3PoseTime = Timer.getFPGATimestamp();
    }

    private void updateLimelight4() {
        shooterHasTarget = ll4Table.getEntry("tv").getDouble(0) == 1.0;
        shooterTx        = ll4Table.getEntry("tx").getDouble(0);
        shooterTy        = ll4Table.getEntry("ty").getDouble(0);
        shooterTa        = ll4Table.getEntry("ta").getDouble(0);

        // BUG FIX (2026-10): was "hb > 0", which stays true forever after the
        // camera disconnects. Online now means the heartbeat is still changing.
        double hb = ll4Table.getEntry("hb").getDouble(-1);
        if (hb >= 0 && hb != lastLL4Heartbeat) {
            lastLL4Heartbeat  = hb;
            lastLL4UpdateTime = Timer.getFPGATimestamp();
        }
    }
    //  TARGET MODE
    public void setTargetMode(TargetMode mode) {
        currentMode = mode;
        ll4Table.getEntry("pipeline").setNumber(mode.pipelineIndex);
    }

    public void toggleHubTrench() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        TargetMode hub    = isBlue ? TargetMode.HUB_BLUE    : TargetMode.HUB_RED;
        TargetMode trench = isBlue ? TargetMode.TRENCH_BLUE  : TargetMode.TRENCH_RED;
        setTargetMode(currentMode == hub ? trench : hub);
    }

    public void setAllianceDefaultMode() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        setTargetMode(isRed ? TargetMode.HUB_RED : TargetMode.HUB_BLUE);
    }
    //  GETTERS
    /** PhotonVision cameras that initialized (same order as PHOTON_CAMERAS). Used by the simulator. */
    public List<PhotonCamera> getPhotonCameras() { return photonCameras; }
    public boolean    hasShooterTarget()        { return shooterHasTarget; }
    public double     getShooterTx()            { return shooterTx; }
    public double     getShooterTy()            { return shooterTy; }
    public double     getShooterTa()            { return shooterTa; }
    public TargetMode getTargetMode()           { return currentMode; }
    public boolean    isHubMode()               { return currentMode == TargetMode.HUB_BLUE || currentMode == TargetMode.HUB_RED; }
    public boolean    isTrenchMode()            { return currentMode == TargetMode.TRENCH_BLUE || currentMode == TargetMode.TRENCH_RED; }

    public boolean isJetsonOnline() {
        return lastJetsonUpdateTime > 0 && (Timer.getFPGATimestamp() - lastJetsonUpdateTime) < JETSON_TIMEOUT_S;
    }

    public boolean isLL3Online() {
        return lastLL3UpdateTime > 0 && (Timer.getFPGATimestamp() - lastLL3UpdateTime) < LL_TIMEOUT_S;
    }

    public boolean isLL4Online() {
        return lastLL4UpdateTime > 0 && (Timer.getFPGATimestamp() - lastLL4UpdateTime) < LL_TIMEOUT_S;
    }

    public double getEstimatedDistanceMeters() {
        if (!shooterHasTarget) return -1.0;
        double a = Math.toRadians(LL4_PITCH_DEG + shooterTy);
        return Math.abs(a) < 1e-6 ? -1.0 : (HUB_HEIGHT_M - LL4_HEIGHT_M) / Math.tan(a);
    }

    /** Update the Field2d pose externally (call from Telemetry or robotPeriodic). */
    public void updateFieldPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    private boolean isOnField(Pose2d p) {
        return p.getX() > 0 && p.getX() < 16.54 && p.getY() > 0 && p.getY() < 8.07;
    }

    private Matrix<N3,N1> scaleByDistance(Matrix<N3,N1> base, double d) {
        double s = 1.0 + d * 0.15;
        return VecBuilder.fill(base.get(0,0)*s, base.get(1,0)*s, base.get(2,0)*s);
    }

    /** Publish telemetry to NT4 for Elastic widgets. */
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Status/Jetson Online",      isJetsonOnline());
        SmartDashboard.putBoolean("Status/Limelight 3 Online", isLL3Online());
        SmartDashboard.putBoolean("Status/Limelight 4 Online", isLL4Online());
        SmartDashboard.putBoolean("Status/LL4 Has Target",     shooterHasTarget);
        SmartDashboard.putString ("Status/Target Mode",        currentMode.name());
        SmartDashboard.putString ("Status/Vision Mode",
            isJetsonOnline() ? "Full (Jetson+LL3)" : "Fallback (LL3 only)");
        SmartDashboard.putBoolean("Status/LL3 Pose Active",
            lastLL3PoseTime > 0 && (Timer.getFPGATimestamp() - lastLL3PoseTime) < LL_TIMEOUT_S);
        SmartDashboard.putNumber ("Vision/LL4 TX",             shooterTx);
        SmartDashboard.putNumber ("Vision/LL4 TY",             shooterTy);
        SmartDashboard.putNumber ("Vision/LL4 TA",             shooterTa);
        SmartDashboard.putNumber ("Vision/Est Distance (m)",   getEstimatedDistanceMeters());
        SmartDashboard.putNumber ("Vision/PV Tags This Loop",  pvMeasurementsThisLoop);
    }
}
