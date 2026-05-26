package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.generated.TunerConstants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

/**
 * AutoAimCommand — rotates robot via swerve to zero LL4 tx.
 *
 * USAGE: joystick.b().whileTrue(autoAimCommand)
 *
 * Driver keeps full X/Y translation while held.
 * Release B → default field-centric drive resumes automatically.
 *
 * isAimed() returns true when tx has been within tolerance for
 * SETTLE_LOOPS consecutive loops (~100ms). Gate shooter fire on this.
 *
 * TUNING ORDER:
 *   1. Increase kP until robot rotates toward target without oscillating
 *   2. Add kD if you see overshoot
 *   3. Tighten AIM_TOLERANCE_DEG once PID is stable
 */
public class AutoAimCommand extends Command {

    // ── Tuning ────────────────────────────────────────────────────────────────
    private static final double kP                  = 0.03;  // start here, tune up
    private static final double kI                  = 0.0;
    private static final double kD                  = 0.002;
    private static final double MAX_ROT_RAD_S       =
        RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    private static final double MAX_ROT_ACCEL       = Math.PI; // rad/s²
    private static final double AIM_TOLERANCE_DEG   = 1.5;
    private static final double TX_DEADZONE_DEG     = 0.3;
    private static final int    SETTLE_LOOPS        = 5;

    // ── Dependencies ─────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem         vision;
    private final DoubleSupplier          xSupplier;
    private final DoubleSupplier          ySupplier;
    private final double                  maxSpeed;

    // ── Control ───────────────────────────────────────────────────────────────
    private final ProfiledPIDController   rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    private int     settleCount = 0;
    private boolean isAimed     = false;

    public AutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        this.drivetrain = drivetrain;
        this.vision     = vision;
        this.xSupplier  = xSupplier;
        this.ySupplier  = ySupplier;
        this.maxSpeed   = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        rotationPID = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(MAX_ROT_RAD_S, MAX_ROT_ACCEL));
        rotationPID.setTolerance(Math.toRadians(AIM_TOLERANCE_DEG));
        rotationPID.setGoal(0.0); // tx = 0 means perfectly aimed

        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(MAX_ROT_RAD_S * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        rotationPID.reset(0.0);
        settleCount = 0;
        isAimed     = false;
    }

    @Override
    public void execute() {
        double xVel = xSupplier.getAsDouble() * maxSpeed;
        double yVel = ySupplier.getAsDouble() * maxSpeed;
        double rot  = 0.0;

        if (vision.hasShooterTarget()) {
            double tx = vision.getShooterTx();

            if (Math.abs(tx) > TX_DEADZONE_DEG) {
                // Positive tx = target right of crosshair → rotate right (negate for CCW-positive)
                rot = MathUtil.clamp(
                    -rotationPID.calculate(Math.toRadians(tx)),
                    -MAX_ROT_RAD_S, MAX_ROT_RAD_S
                );
            }

            settleCount = Math.abs(tx) <= AIM_TOLERANCE_DEG ? settleCount + 1 : 0;
            isAimed = settleCount >= SETTLE_LOOPS;
        } else {
            rotationPID.reset(0.0);
            settleCount = 0;
            isAimed     = false;
        }

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xVel)
                .withVelocityY(yVel)
                .withRotationalRate(rot)
        );

        SmartDashboard.putBoolean("AutoAim/IsAimed",    isAimed);
        SmartDashboard.putBoolean("AutoAim/HasTarget",  vision.hasShooterTarget());
        SmartDashboard.putNumber ("AutoAim/TX",         vision.getShooterTx());
        SmartDashboard.putNumber ("AutoAim/RotOutput",  rot);
        SmartDashboard.putNumber ("AutoAim/Distance_m", vision.getEstimatedDistanceMeters());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        isAimed     = false;
        settleCount = 0;
        SmartDashboard.putBoolean("AutoAim/IsAimed", false);
    }

    @Override
    public boolean isFinished() { return false; } // whileTrue handles termination

    /** True when tx settled within tolerance for ~100ms. Gate shooter fire on this. */
    public boolean isAimed() { return isAimed; }
}
