package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.commands.AimController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ShootAllAuto — Autonomous command for 2026 REBUILT.
 *
 * Sequence:
 *   1. Spin up shooter wheels (pre-spool) — feeder stays OFF
 *   2. Rotate robot via swerve to zero LL4 tx (auto-aim)
 *   3. Once aimed & shooter at speed, run feeder to fire
 *   4. Continue feeding for FEED_DURATION_S to clear all preloaded balls
 *   5. Stop shooter & feeder, end
 *
 * The robot stays stationary (no X/Y translation). If no target is
 * found within TIMEOUT_S the command ends gracefully.
 *
 * BUG FIX (2026-10): the feeder used to start in initialize() together with
 * the flywheel, so balls left before aiming. isFiring now actually starts
 * the feeder. Aim math moved to AimController (also fixed there).
 */
public class ShootAllAuto extends Command {

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
    private final AimController aim = new AimController();
    private final SwerveRequest.FieldCentric driveRequest;

    private boolean isFiring        = false;
    private double  startTime       = 0;
    private double  fireStartTime   = 0;

    public ShootAllAuto(
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            VisionSubsystem vision) {

        this.drivetrain = drivetrain;
        this.shooter    = shooter;
        this.vision     = vision;

        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, shooter);
    }

    @Override
    public void initialize() {
        aim.reset();
        isFiring       = false;
        startTime      = Timer.getFPGATimestamp();
        fireStartTime  = 0;

        // Start spinning the flywheel immediately (feeder stays off)
        shooter.spinUp();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ── Aim — rotation only, no translation
        double rot = aim.calculate(vision);
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rot)
        );

        // ── Fire when aimed and spooled
        boolean spooledUp = (now - startTime) >= SPOOL_UP_TIME_S && shooter.isAtSpeed();

        if (aim.isAimed() && spooledUp && !isFiring) {
            isFiring      = true;
            fireStartTime = now;
        }

        // If we lose aim while firing, that's OK — keep feeding.
        // Balls are already committed once the feeder starts.
        if (isFiring) shooter.feed(); else shooter.spinUp();

        // ── Telemetry
        SmartDashboard.putBoolean("Auto/IsAimed",   aim.isAimed());
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
