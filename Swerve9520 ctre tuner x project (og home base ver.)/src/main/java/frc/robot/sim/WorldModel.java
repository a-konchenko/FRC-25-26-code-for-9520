package frc.robot.sim;

import java.util.ArrayDeque;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * ★ NEW (sim only) — where the robot REALLY is ("Truth").
 *
 * CTRE's swerve sim spins the simulated motors, and the robot code's pose
 * estimator turns encoder + gyro readings into the "Estimate". This class
 * receives the same module positions and gyro heading — every odometry sample
 * (250 Hz, via the drivetrain's telemetry listener), exactly what the robot's
 * odometry integrates — but sends the motion through the DriftModel before
 * integrating it. With drift set to zero, Truth and Estimate are identical.
 * Cameras look at the field from Truth at the moment each frame was taken
 * (truthAt), so vision pulls the Estimate back toward it.
 *
 * Locking: CTRE calls onSample while holding its own state lock, so this
 * class must NEVER call into the drivetrain (getState / getStateCopy) while
 * holding its own lock — that is a lock-order inversion and can deadlock.
 * Read the drivetrain state first, then lock (see setTruth).
 */
public class WorldModel {
    private static final double kHistorySeconds = 1.5;

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDriveKinematics   kinematics;
    private final DriftModel              drift;

    private Pose2d                 truth = Pose2d.kZero;
    private Pose2d                 estimateAtTruth = Pose2d.kZero;
    private SwerveModulePosition[] lastPositions;
    private Rotation2d             lastHeading;
    private double                 lastTimestamp = Double.NaN;
    private double                 lastSimTime = Double.NaN;
    private double                 lastSpeed = 0.0;
    private double                 accel = 0.0;
    private volatile boolean       driftEnabled = true;

    /** {sim time, x, y, θ} of Truth over the last kHistorySeconds (camera latency). */
    private final ArrayDeque<double[]> history = new ArrayDeque<>();

    public WorldModel(CommandSwerveDrivetrain drivetrain, DriftModel drift) {
        this.drivetrain = drivetrain;
        this.kinematics = drivetrain.getKinematics();
        this.drift      = drift;
        capture(drivetrain.getStateCopy());
        drivetrain.addTelemetryListener(this::onSample);
    }

    private void capture(SwerveDriveState state) {
        lastPositions = copy(state.ModulePositions);
        lastHeading   = state.RawHeading;
        lastTimestamp = state.Timestamp;
        estimateAtTruth = state.Pose;
    }

    /**
     * Teleport Truth (does not touch the robot code's estimate). Reads the
     * drivetrain BEFORE locking — do not call this while holding lock();
     * use setTruth(pose, state) there instead.
     */
    public void setTruth(Pose2d pose) {
        setTruth(pose, drivetrain.getStateCopy());
    }

    /** Teleport Truth using a drivetrain state the caller read beforehand (safe inside lock()). */
    public synchronized void setTruth(Pose2d pose, SwerveDriveState state) {
        truth = pose;
        capture(state);
        history.clear();
    }

    /** Drift only accumulates while enabled (gyro bias would otherwise spin a parked robot). */
    public void setDriftEnabled(boolean enabled) { driftEnabled = enabled; }

    /** Runs on CTRE's odometry thread (CTRE's state lock held) for every odometry sample. */
    private synchronized void onSample(SwerveDriveState state) {
        if (state.Timestamp == lastTimestamp) return;
        SwerveModulePosition[] now = state.ModulePositions;
        if (now == null || lastPositions == null || now.length != lastPositions.length
                || Double.isNaN(lastTimestamp)) {
            capture(state);
            return;
        }
        // Robot (sim) time, which stops while the simulator is paused — so gyro
        // bias doesn't accumulate during a pause.
        double simNow = Timer.getFPGATimestamp();
        double dt = Double.isNaN(lastSimTime) ? 0.0 : Math.max(0.0, simNow - lastSimTime);
        lastSimTime = simNow;

        // Robot-relative motion the encoders measured since the last sample
        var deltas = new SwerveModulePosition[now.length];
        for (int i = 0; i < now.length; i++) {
            deltas[i] = new SwerveModulePosition(
                now[i].distanceMeters - lastPositions[i].distanceMeters, now[i].angle);
        }
        Twist2d wheelTwist = kinematics.toTwist2d(deltas);

        // Heading change the gyro measured (the pose estimator uses the gyro, not the wheels)
        double dTheta = state.RawHeading.minus(lastHeading).getRadians();
        Twist2d measured = new Twist2d(wheelTwist.dx, wheelTwist.dy, dTheta);

        double speed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
        if (dt > 1e-6) accel = 0.8 * accel + 0.2 * Math.abs(speed - lastSpeed) / dt; // smoothed
        lastSpeed = speed;

        Twist2d real = driftEnabled ? drift.apply(measured, dt, accel) : measured;
        truth = truth.exp(real);

        history.addLast(new double[] { simNow, truth.getX(), truth.getY(), truth.getRotation().getRadians() });
        while (!history.isEmpty() && simNow - history.peekFirst()[0] > kHistorySeconds) history.removeFirst();

        capture(state);
    }

    public synchronized Pose2d getTruth() { return truth; }

    /** {truth, estimate} from the SAME odometry sample, so errors aren't skewed by timing. */
    public synchronized Pose2d[] getTruthAndEstimate() { return new Pose2d[] { truth, estimateAtTruth }; }

    /**
     * Where the robot really was at sim time t (a camera frame taken `latency`
     * ago). Interpolated from the history; the current truth if t is newer
     * than the last sample or there is no history yet.
     */
    public synchronized Pose2d truthAt(double t) {
        double[] prev = null;
        for (double[] s : history) {
            if (s[0] >= t) {
                if (prev == null) return new Pose2d(s[1], s[2], Rotation2d.fromRadians(s[3]));
                double f = (t - prev[0]) / Math.max(1e-9, s[0] - prev[0]);
                double dth = Math.IEEEremainder(s[3] - prev[3], 2 * Math.PI);
                return new Pose2d(prev[1] + (s[1] - prev[1]) * f, prev[2] + (s[2] - prev[2]) * f,
                    Rotation2d.fromRadians(prev[3] + dth * f));
            }
            prev = s;
        }
        return truth;
    }

    public double getAccelMps2() { return accel; }

    /** Lock shared with the odometry thread — hold it while changing DriftModel state. Never call the drivetrain while holding it. */
    public Object lock() { return this; }

    private static SwerveModulePosition[] copy(SwerveModulePosition[] src) {
        if (src == null) return null;
        var out = new SwerveModulePosition[src.length];
        for (int i = 0; i < src.length; i++) out[i] = src[i].copy();
        return out;
    }
}
