package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * JetsonVision
 *
 * Handles all NetworkTables communication between the roboRIO and the Jetson.
 *
 * ── Table layout ────────────────────────────────────────────────────────────
 *
 *  /Jetson  (data FROM the Jetson → read by the robot)
 *      targetX          double   Target X offset in metres (field frame)
 *      targetY          double   Target Y offset in metres (field frame)
 *      driveVX          double   Requested drive velocity X (m/s)
 *      driveVY          double   Requested drive velocity Y (m/s)
 *      driveOmega       double   Requested rotation (rad/s)
 *      gamePieceDetected boolean Whether a game piece is currently visible
 *      gamePieceType    string   "coral" / "algae" / "none" etc.
 *
 *  /Robot  (data FROM the robot → read by the Jetson)
 *      poseX            double   Robot X on field (metres)
 *      poseY            double   Robot Y on field (metres)
 *      poseHeading      double   Robot heading (degrees)
 *      speedVX          double   Current chassis vX (m/s)
 *      speedVY          double   Current chassis vY (m/s)
 *      speedOmega       double   Current chassis omega (rad/s)
 *      matchState       string   "disabled" / "auto" / "teleop" / "test"
 *
 * ────────────────────────────────────────────────────────────────────────────
 *
 * The Jetson should connect to NT server at 10.95.20.2 (or whatever the
 * roboRIO IP is for team 9520) using robotpy-ntcore / ntcore4 on Python.
 */
public class JetsonVision extends SubsystemBase {

    // ── NetworkTables setup ──────────────────────────────────────────────────
    private final NetworkTableInstance m_nt = NetworkTableInstance.getDefault();

    // Tables
    private final NetworkTable m_jetsonTable = m_nt.getTable("Jetson");
    private final NetworkTable m_robotTable  = m_nt.getTable("Robot");

    // ── Subscribers (FROM Jetson) ────────────────────────────────────────────
    private final DoubleSubscriber  m_targetX           = m_jetsonTable.getDoubleTopic("targetX").subscribe(0.0);
    private final DoubleSubscriber  m_targetY           = m_jetsonTable.getDoubleTopic("targetY").subscribe(0.0);
    private final DoubleSubscriber  m_driveVX           = m_jetsonTable.getDoubleTopic("driveVX").subscribe(0.0);
    private final DoubleSubscriber  m_driveVY           = m_jetsonTable.getDoubleTopic("driveVY").subscribe(0.0);
    private final DoubleSubscriber  m_driveOmega        = m_jetsonTable.getDoubleTopic("driveOmega").subscribe(0.0);
    private final BooleanSubscriber m_gamePieceDetected = m_jetsonTable.getBooleanTopic("gamePieceDetected").subscribe(false);
    private final DoubleSubscriber  m_targetTimestamp   = m_jetsonTable.getDoubleTopic("timestamp").subscribe(0.0);

    // ── Publishers (TO Jetson) ───────────────────────────────────────────────
    private final DoublePublisher  m_pubPoseX      = m_robotTable.getDoubleTopic("poseX").publish();
    private final DoublePublisher  m_pubPoseY      = m_robotTable.getDoubleTopic("poseY").publish();
    private final DoublePublisher  m_pubHeading    = m_robotTable.getDoubleTopic("poseHeading").publish();
    private final DoublePublisher  m_pubSpeedVX    = m_robotTable.getDoubleTopic("speedVX").publish();
    private final DoublePublisher  m_pubSpeedVY    = m_robotTable.getDoubleTopic("speedVY").publish();
    private final DoublePublisher  m_pubSpeedOmega = m_robotTable.getDoubleTopic("speedOmega").publish();
    private final StringPublisher  m_pubMatchState = m_robotTable.getStringTopic("matchState").publish();

    // ── Stale-data timeout ───────────────────────────────────────────────────
    // If the Jetson hasn't sent a fresh timestamp in this many seconds,
    // treat its data as stale and ignore drive / align commands.
    private static final double kStaleTimeoutSeconds = 0.5;
    private double m_lastTimestamp = 0.0;

    public JetsonVision() {
        // Nothing extra needed – NT server starts automatically on the roboRIO
    }

    // ── Periodic: publish robot state to Jetson every loop ──────────────────
    @Override
    public void periodic() {
        // Match state string so Jetson knows what mode we're in
        String matchState;
        if      (DriverStation.isDisabled())   matchState = "disabled";
        else if (DriverStation.isAutonomous()) matchState = "auto";
        else if (DriverStation.isTeleop())     matchState = "teleop";
        else                                   matchState = "test";
        m_pubMatchState.set(matchState);

        // Track freshness of Jetson data
        double incoming = m_targetTimestamp.get();
        if (incoming != m_lastTimestamp) {
            m_lastTimestamp = incoming;
        }
    }

    /**
     * Call this every loop from RobotContainer (or a command) to push the
     * latest odometry and speed data to the Jetson.
     *
     * @param pose   Current robot pose from the drivetrain
     * @param speeds Current chassis speeds from the drivetrain
     */
    public void updateRobotState(Pose2d pose, ChassisSpeeds speeds) {
        m_pubPoseX.set(pose.getX());
        m_pubPoseY.set(pose.getY());
        m_pubHeading.set(pose.getRotation().getDegrees());
        m_pubSpeedVX.set(speeds.vxMetersPerSecond);
        m_pubSpeedVY.set(speeds.vyMetersPerSecond);
        m_pubSpeedOmega.set(speeds.omegaRadiansPerSecond);
    }

    // ── Getters: data from Jetson ────────────────────────────────────────────

    /** X position of the detected target (field frame, metres). */
    public double getTargetX() { return m_targetX.get(); }

    /** Y position of the detected target (field frame, metres). */
    public double getTargetY() { return m_targetY.get(); }

    /** Drive velocity X requested by the Jetson (m/s). */
    public double getDriveVX() { return m_driveVX.get(); }

    /** Drive velocity Y requested by the Jetson (m/s). */
    public double getDriveVY() { return m_driveVY.get(); }

    /** Rotation rate requested by the Jetson (rad/s). */
    public double getDriveOmega() { return m_driveOmega.get(); }

    /** True if the Jetson currently sees a game piece. */
    public boolean isGamePieceDetected() { return m_gamePieceDetected.get(); }

    /**
     * Returns true if the Jetson data is fresh enough to trust.
     * Uses the timestamp the Jetson publishes each frame.
     */
    public boolean isDataFresh() {
        return (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_lastTimestamp) < kStaleTimeoutSeconds;
    }

    /**
     * Tells the Jetson which set of AprilTags to target.
     * Call from autonomous before driving toward a new destination.
     * @param mode "hub" or "tower"
     */
    public void setAlignMode(String mode) {
        m_robotTable.getStringTopic("alignMode").publish().set(mode);
    }
}
