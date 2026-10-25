package frc.robot.sim;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * ★ NEW (sim only) — owns every simulator block and the link to the web UI.
 *
 * Created by Robot only when RobotBase.isSimulation(); update() runs from
 * Robot.simulationPeriodic() every 20 ms loop, after the robot code's own
 * periodic work. Nothing in this package runs on the real roboRIO.
 */
public class SimManager {
    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer container;
    private final DriftModel  drift = new DriftModel();
    private final WorldModel  world;
    private final SimNT       nt = new SimNT();
    private final SimClock    clock = new SimClock();
    private final SimControl  control;
    private final PathLogging pathLogging = new PathLogging(nt);
    private final PlanPreview planPreview = new PlanPreview(nt);
    private final FieldModel  field = new FieldModel();
    private final LimelightSim ll3 = new LimelightSim("ll3", LimelightSim.Mode.MEGATAG2, "limelight-three",
        SimRobotConstants.kLL3Mount, 62.5, 48.9, 6.0, 30.0);   // LL3: 62.5° × 48.9° FOV
    private final LimelightSim ll4 = new LimelightSim("ll4", LimelightSim.Mode.TARGETING, "limelight-four",
        SimRobotConstants.kLL4Mount, 82.0, 56.2, 7.0, 20.0);   // LL4: 82° × 56.2° FOV
    private final PhotonSim   photon;
    private final MechanismSims mech = new MechanismSims();
    private final FuelSim     fuel;

    /** Vision measurements the robot code actually fused: {seq, x, y, θ, timestamp}. */
    private final ArrayDeque<double[]> fused = new ArrayDeque<>();
    private int fusedSeq = 0;
    private double lastUpdateTime = Timer.getFPGATimestamp();

    private boolean placed   = false;
    private int     heartbeat = 0;
    private Command testDrive = null;
    /** Command waiting for the robot to actually be enabled before scheduling. */
    private Command pendingSchedule = null;

    public SimManager(RobotContainer container) {
        this.container  = container;
        this.drivetrain = container.drivetrain;
        this.world      = new WorldModel(drivetrain, drift);
        this.control    = new SimControl(this, container, nt);
        this.photon     = new PhotonSim(container.vision, field);
        this.fuel       = new FuelSim(mech, container.shooter, field);

        // Every vision measurement the pose estimator receives (main thread, from VisionSubsystem)
        drivetrain.addVisionMeasurementListener((pose, ts, std) -> {
            fused.addLast(new double[] { ++fusedSeq, pose.getX(), pose.getY(), pose.getRotation().getRadians(), ts });
            while (fused.size() > 40) fused.removeFirst();
        });

        publishStatic();
        System.out.println("[Sim] SimManager ready — open the web UI at http://localhost:5173");
    }

    /** Things that never change: robot size, module layout, tag layouts. */
    private void publishStatic() {
        nt.array("Robot/size", new double[] {
            SimRobotConstants.kLengthWithBumpersM, SimRobotConstants.kWidthWithBumpersM,
            SimRobotConstants.kFrameLengthM, SimRobotConstants.kFrameWidthM,
        });
        var locs = drivetrain.getModuleLocations();
        double[] moduleXY = new double[locs.length * 2];
        for (int i = 0; i < locs.length; i++) {
            moduleXY[2 * i]     = locs[i].getX();
            moduleXY[2 * i + 1] = locs[i].getY();
        }
        nt.array("Robot/moduleLocations", moduleXY);
        nt.number("Robot/maxSpeedMps", TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond));
        nt.array("Robot/ll3Mount", SimRobotConstants.kLL3Mount);
        nt.array("Robot/ll4Mount", SimRobotConstants.kLL4Mount);

        publishTags("andymark", AprilTagFields.k2026RebuiltAndymark);
        publishTags("welded",   AprilTagFields.k2026RebuiltWelded);
        // VisionSubsystem loads the AndyMark layout
        nt.string("Field/robotCodeVariant", "andymark");
    }

    /** Tags as flat double[]: {id, x, y, z, yawRad, pitchRad} per tag. */
    private void publishTags(String name, AprilTagFields field) {
        try {
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(field);
            List<Double> flat = new ArrayList<>();
            for (var tag : layout.getTags()) {
                var p = tag.pose;
                flat.add((double) tag.ID);
                flat.add(p.getX());
                flat.add(p.getY());
                flat.add(p.getZ());
                flat.add(p.getRotation().getZ());
                flat.add(p.getRotation().getY());
            }
            nt.array("Field/tags/" + name, flat.stream().mapToDouble(Double::doubleValue).toArray());
            nt.array("Field/size/" + name, new double[] { layout.getFieldLength(), layout.getFieldWidth() });
        } catch (Exception e) {
            System.out.println("[Sim] Could not load tag layout " + name + ": " + e.getMessage());
        }
    }

    /** Put the robot somewhere: the robot code's estimate AND the truth. */
    public void placeRobot(Pose2d pose) {
        placeRobot(pose, pose);
    }

    /** Estimate (what the code believes) and truth (where it really is) separately. */
    public void placeRobot(Pose2d estimate, Pose2d truth) {
        drivetrain.resetPose(estimate);
        var state = drivetrain.getStateCopy(); // read BEFORE locking (see WorldModel)
        synchronized (world.lock()) {
            world.setTruth(truth, state);
            drift.reset();
        }
        teleported(truth);
    }

    /** Cameras must not report frames that straddle a teleport (see LimelightSim.suppress). */
    private void teleported(Pose2d truth) {
        double s = 0.15;
        ll3.suppress(s);
        ll4.suppress(s);
        photon.teleported(truth, s);
    }

    public void cancelTestDrive() {
        if (testDrive != null) testDrive.cancel();
        pendingSchedule = null;
    }

    public void update() {
        if (!placed) {
            double[] s = SimRobotConstants.kDefaultStartPose;
            placeRobot(new Pose2d(s[0], s[1], Rotation2d.fromRadians(s[2])));
            placed = true;
        }

        readControls();
        control.update();
        pathLogging.update();
        planPreview.update();

        if (pendingSchedule != null && DriverStation.isEnabled()) {
            CommandScheduler.getInstance().schedule(pendingSchedule);
            pendingSchedule = null;
        }

        // Truth integrates on its own 250 Hz thread (see WorldModel)
        world.setDriftEnabled(DriverStation.isEnabled());

        double now = Timer.getFPGATimestamp();
        double dt = Math.min(now - lastUpdateTime, 0.05);
        lastUpdateTime = now;
        Pose2d[] te = world.getTruthAndEstimate();

        // Mechanisms + fuel (phase 4)
        mech.update(dt);
        fuel.update(dt, te[0]);

        // Cameras (phase 3). Frames are written at the END of this loop, so the
        // robot code reads them next loop — like a real camera, never "instant".
        ll3.update(world, field, te[1]);
        ll4.update(world, field, te[1]);
        photon.update(te[0]);

        publishState();
        publishVision(te[0]);
        publishMech();
        fuel.publish(nt);
    }

    public void resetFuel() { fuel.reset(); }

    private void publishVision(Pose2d truth) {
        for (LimelightSim ll : new LimelightSim[] { ll3, ll4 }) {
            double[] m = ll.getMount();
            String p = "Vision/" + ll.key + "/";
            nt.array(p + "config", new double[] { m[0], m[1], m[2], m[4], m[5], ll.hFovDeg, ll.vFovDeg, ll.maxRangeM,
                ll.enabled ? 1 : 0, ll.mountErrX, ll.mountErrY, ll.mountErrYawDeg });
            nt.array(p + "visible", ll.visibleIds.stream().mapToDouble(Integer::doubleValue).toArray());
            double[] f = new double[ll.lastFrame.length + 1];
            f[0] = ll.frameCount;
            System.arraycopy(ll.lastFrame, 0, f, 1, ll.lastFrame.length);
            nt.array(p + "frame", f);
        }

        // PhotonVision: configured mounts {x, y, z, roll, pitch, yaw} per camera, params, and
        // which tags each could see (display only — PhotonLib does the real projection)
        int n = photon.count();
        double[] mounts = new double[n * 6];
        List<Double> visible = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double[] m = photon.mount(i);
            System.arraycopy(m, 0, mounts, 6 * i, 6);
            if (!photon.enabled) continue;
            double[] cam = CameraGeometry.cameraOnField(truth, m[0] + photon.mountErrX, m[1] + photon.mountErrY, m[2],
                m[5] + photon.mountErrYawDeg, m[4]);
            List<Integer> ids = new ArrayList<>();
            CameraGeometry.visibleTags(field, cam, PhotonSim.kHFovDeg, PhotonSim.kVFovDeg, PhotonSim.kMaxRangeM, ids, null);
            for (int id : ids) { visible.add((double) i); visible.add((double) id); }
        }
        nt.array("Vision/pv/mounts", mounts);
        nt.array("Vision/pv/params", new double[] { PhotonSim.kHFovDeg, PhotonSim.kVFovDeg, PhotonSim.kMaxRangeM,
            photon.enabled ? 1 : 0, photon.mountErrX, photon.mountErrY, photon.mountErrYawDeg });
        nt.array("Vision/pv/visible", visible.stream().mapToDouble(Double::doubleValue).toArray());

        double[] flat = new double[fused.size() * 5];
        int k = 0;
        for (double[] f : fused) for (double v : f) flat[k++] = v;
        nt.array("Vision/fused", flat);
    }

    private void publishMech() {
        nt.array("Mech/state", new double[] {
            mech.shooter.rotorRps(), container.shooter.getShooterSetpointRPS(), mech.feeder.rps(), mech.intake.rps(),
            mech.arm.fraction(), mech.climbLeft.fraction(), mech.climbRight.fraction(),
            container.shooter.isAtSpeed() ? 1 : 0 });
    }

    private void readControls() {
        synchronized (world.lock()) { readDriftSettings(); }
        ll3.readSettings(nt);
        ll4.readSettings(nt);
        photon.readSettings(nt);
        fuel.readSettings(nt);
        if (nt.action("resetFuel") != null) fuel.reset();
        readActions();
    }

    private void readDriftSettings() {
        drift.wheelScalePct       = nt.setting("drift/wheelScalePct",       drift.wheelScalePct);
        drift.slipCmPerSqrtM      = nt.setting("drift/slipCmPerSqrtM",      drift.slipCmPerSqrtM);
        drift.slipAccelThreshold  = nt.setting("drift/slipAccelThreshold",  drift.slipAccelThreshold);
        drift.slipAccelMultiplier = nt.setting("drift/slipAccelMultiplier", drift.slipAccelMultiplier);
        drift.gyroBiasDegPerMin   = nt.setting("drift/gyroBiasDegPerMin",   drift.gyroBiasDegPerMin);
        drift.gyroScalePct        = nt.setting("drift/gyroScalePct",        drift.gyroScalePct);
        drift.setSeed((long) nt.setting("seed", 9520));
    }

    private void readActions() {
        // [x, y, thetaRad] — move the robot (estimate + truth), e.g. "Place robot"
        double[] place = nt.action("placeRobot");
        if (place != null && place.length >= 3) {
            placeRobot(new Pose2d(place[0], place[1], Rotation2d.fromRadians(place[2])));
        }

        // [x, y, thetaRad] — move ONLY the truth ("kidnap"): the robot code doesn't know
        double[] kidnap = nt.action("kidnap");
        if (kidnap != null && kidnap.length >= 3) {
            Pose2d k = new Pose2d(kidnap[0], kidnap[1], Rotation2d.fromRadians(kidnap[2]));
            world.setTruth(k);
            teleported(k);
        }

        // [pattern] — 1 = 2 m square, 2 = spin in place 2 turns
        double[] td = nt.action("testDrive");
        if (td != null) startTestDrive(td.length > 0 ? (int) td[0] : 1);

        // [] — stop everything and disable
        if (nt.action("stop") != null) {
            cancelTestDrive();
            control.stop();
            setEnabled(false, false);
        }

        // [] — snap truth back onto the estimate (clears accumulated drift)
        if (nt.action("syncTruth") != null) {
            var state = drivetrain.getStateCopy(); // read BEFORE locking (see WorldModel)
            synchronized (world.lock()) {
                world.setTruth(state.Pose, state);
                drift.reset();
            }
            teleported(state.Pose);
        }
    }

    /** Enable the robot from the sim (no Driver Station needed). */
    public void setEnabled(boolean enabled, boolean autonomous) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(autonomous);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    /**
     * ★ NEW (sim only) — scripted teleop drive to feel the drift without a
     * joystick. Field-centric velocities; the robot code's default drive is
     * interrupted while this runs (it requires the drivetrain).
     */
    private void startTestDrive(int pattern) {
        control.stop();
        if (testDrive != null) testDrive.cancel();
        final var req = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final double v = 1.5, side = 2.0, t = side / v;
        Command c;
        if (pattern == 2) {
            c = drivetrain.applyRequest(() -> req.withVelocityX(0).withVelocityY(0).withRotationalRate(Math.PI))
                .withTimeout(4.0);
        } else {
            c = Commands.sequence(
                drivetrain.applyRequest(() -> req.withVelocityX( v).withVelocityY(0).withRotationalRate(0)).withTimeout(t),
                drivetrain.applyRequest(() -> req.withVelocityX(0).withVelocityY( v).withRotationalRate(0)).withTimeout(t),
                drivetrain.applyRequest(() -> req.withVelocityX(-v).withVelocityY(0).withRotationalRate(0)).withTimeout(t),
                drivetrain.applyRequest(() -> req.withVelocityX(0).withVelocityY(-v).withRotationalRate(0)).withTimeout(t)
            );
        }
        testDrive = Commands.sequence(
                c,
                drivetrain.applyRequest(() -> req.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).withTimeout(0.5))
            .finallyDo(() -> setEnabled(false, false))
            .withName("Sim Test Drive");
        setEnabled(true, false);
        // The enable is seen by the robot code on the NEXT loop; a command
        // scheduled while still disabled would be dropped.
        pendingSchedule = testDrive;
    }

    private void publishState() {
        var state = drivetrain.getState();
        Pose2d[] pair = world.getTruthAndEstimate();
        Pose2d truth = pair[0];
        Pose2d est   = pair[1];

        nt.pose("Truth", truth);
        nt.pose("Estimate", est);
        // Truth, estimate and setpoint in ONE value, so readers never pair this
        // loop's truth with last loop's estimate (errors would look larger):
        // {truth x, y, θ, estimate x, y, θ, setpointActive 0/1, setpoint x, y, θ}
        Pose2d sp = pathLogging.getSetpoint();
        nt.array("TruthEstimate", new double[] {
            truth.getX(), truth.getY(), truth.getRotation().getRadians(),
            est.getX(), est.getY(), est.getRotation().getRadians(),
            pathLogging.isActive() ? 1 : 0, sp.getX(), sp.getY(), sp.getRotation().getRadians() });

        // Per module: {steer angle rad, wheel speed m/s} × 4 (robot frame)
        var ms = state.ModuleStates;
        if (ms != null) {
            double[] mod = new double[ms.length * 2];
            for (int i = 0; i < ms.length; i++) {
                mod[2 * i]     = ms[i].angle.getRadians();
                mod[2 * i + 1] = ms[i].speedMetersPerSecond;
            }
            nt.array("Modules", mod);
        }
        nt.array("Speeds", new double[] {
            state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, state.Speeds.omegaRadiansPerSecond });

        // Drift summary: {distance m, gyro error deg, localization error m, heading error deg}
        double locErr  = est.getTranslation().getDistance(truth.getTranslation());
        double headErr = est.getRotation().minus(truth.getRotation()).getDegrees();
        nt.array("Drift/state", new double[] { drift.getDistanceM(), drift.getGyroErrorDeg(), locErr, headErr });

        String mode = DriverStation.isDisabled() ? "disabled"
                    : DriverStation.isAutonomous() ? "auto"
                    : DriverStation.isTeleop() ? "teleop" : "test";
        nt.string("Match/mode", mode);
        nt.number("Match/time", DriverStation.getMatchTime());
        nt.string("Match/alliance", DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
        nt.number("Time", Timer.getFPGATimestamp());
        nt.number("Heartbeat", ++heartbeat);
    }
}
