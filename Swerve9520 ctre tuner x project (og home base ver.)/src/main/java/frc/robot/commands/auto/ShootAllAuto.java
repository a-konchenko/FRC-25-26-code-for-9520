package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * ShootAllAuto — Autonomous command for 2026 REBUILT.
 *
 * Sequence:
 *   1. Spin up shooter wheels (pre-spool)
 *   2. Rotate robot via swerve to zero LL4 tx (auto-aim)
 *   3. Once aimed & shooter at speed, run feeder to fire
 *   4. Continue feeding for FEED_DURATION_S to clear all preloaded balls
 *   5. Stop shooter & feeder, end
 *
 * The robot stays stationary (no X/Y translation). If no target is
 * found within TIMEOUT_S the command ends gracefully.
 */
public class ShootAllAuto extends Command {

    // ── Tuning constants
    private static final double kP               = 0.03;
    private static final double kI               = 0.0;
    private static final double kD               = 0.002;
    private static final double MAX_ROT_RAD_S    = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    private static final double MAX_ROT_ACCEL    = Math.PI;
    private static final double AIM_TOLERANCE_DEG = 1.5;
    private static final double TX_DEADZONE_DEG  = 0.3;
    private static final int    SETTLE_LOOPS     = 5;

    /** How long to feed balls after we start firing (seconds). */
    private static final double FEED_DURATION_S  = 4.0;
    /** Abort if we never find a target within this time (seconds). */
    private static final double TIMEOUT_S        = 12.0;
    /** Minimum shooter spool-up time before feeding (seconds). */
    private static final double SPOOL_UP_TIME_S  = 0.8;

    // ── Dependencies 
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter                shooter;
    private final VisionSubsystem        vision;

    // ── Control 
    private final ProfiledPIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    private int     settleCount     = 0;
    private boolean isAimed         = false;
    private boolean isFiring        = false;
    private double  startTime       = 0;
    private double  fireStartTime   = 0;
    private double  spoolStartTime  = 0;

    public ShootAllAuto(
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            VisionSubsystem vision) {

        this.drivetrain = drivetrain;
        this.shooter    = shooter;
        this.vision     = vision;

        rotationPID = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(MAX_ROT_RAD_S, MAX_ROT_ACCEL));
        rotationPID.setTolerance(Math.toRadians(AIM_TOLERANCE_DEG));
        rotationPID.setGoal(0.0);

        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(MAX_ROT_RAD_S * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, shooter, vision);
    }

    @Override
    public void initialize() {
        rotationPID.reset(0.0);
        settleCount    = 0;
        isAimed        = false;
        isFiring       = false;
        startTime      = Timer.getFPGATimestamp();
        fireStartTime  = 0;
        spoolStartTime = Timer.getFPGATimestamp();

        // Start spinning shooter wheels immediately
        shooter.runShooter();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double rot = 0.0;

        // ── Aim 
        if (vision.hasShooterTarget()) {
            double tx = vision.getShooterTx();

            if (Math.abs(tx) > TX_DEADZONE_DEG) {
                rot = MathUtil.clamp(
                    -rotationPID.calculate(Math.toRadians(tx)),
                    -MAX_ROT_RAD_S, MAX_ROT_RAD_S);
            }

            settleCount = Math.abs(tx) <= AIM_TOLERANCE_DEG ? settleCount + 1 : 0;
            isAimed = settleCount >= SETTLE_LOOPS;
        } else {
            rotationPID.reset(0.0);
            settleCount = 0;
            isAimed     = false;
        }

        // Apply rotation only, no translation
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rot)
        );

        // ── Fire when aimed and spooled 
        boolean spooledUp = (now - spoolStartTime) >= SPOOL_UP_TIME_S;

        if (isAimed && spooledUp && !isFiring) {
            isFiring      = true;
            fireStartTime = now;
        }

        // If we lose aim while firing, that's OK — keep feeding.
        // Balls are already committed once the feeder starts.

        // ── Telemetry 
        SmartDashboard.putBoolean("Auto/IsAimed",   isAimed);
        SmartDashboard.putBoolean("Auto/IsFiring",   isFiring);
        SmartDashboard.putBoolean("Auto/SpooledUp",  spooledUp);
        SmartDashboard.putNumber ("Auto/TX",         vision.getShooterTx());
        SmartDashboard.putNumber ("Auto/ElapsedS",   now - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        drivetrain.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

        SmartDashboard.putBoolean("Auto/IsAimed",  false);
        SmartDashboard.putBoolean("Auto/IsFiring",  false);
    }

    @Override
    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();

        // Timeout — no target found or something went wrong
        if ((now - startTime) >= TIMEOUT_S) return true;

        // Done feeding all balls
        if (isFiring && (now - fireStartTime) >= FEED_DURATION_S) return true;

        return false;
    }
}
