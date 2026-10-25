package frc.robot.sim;

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
    private final DriftModel  drift = new DriftModel();
    private final WorldModel  world;
    private final SimNT       nt = new SimNT();
    private final SimClock    clock = new SimClock();
    private final SimControl  control;
    private final PathLogging pathLogging = new PathLogging(nt);
    private final PlanPreview planPreview = new PlanPreview(nt);

    private boolean placed   = false;
    private int     heartbeat = 0;
    private Command testDrive = null;
    /** Command waiting for the robot to actually be enabled before scheduling. */
    private Command pendingSchedule = null;

    public SimManager(RobotContainer container) {
        this.drivetrain = container.drivetrain;
        this.world      = new WorldModel(drivetrain, drift);
        this.control    = new SimControl(this, container, nt);

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
        synchronized (world.lock()) {
            world.setTruth(truth);
            drift.reset();
        }
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

        publishState();
    }

    private void readControls() {
        synchronized (world.lock()) { readDriftSettings(); }
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
            world.setTruth(new Pose2d(kidnap[0], kidnap[1], Rotation2d.fromRadians(kidnap[2])));
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
            synchronized (world.lock()) {
                world.setTruth(drivetrain.getState().Pose);
                drift.reset();
            }
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
