package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * AutoAimCommand — rotates robot via swerve to zero LL4 tx.
 *
 * USAGE: joystick.b().whileTrue(autoAimCommand)
 *
 * Driver keeps full X/Y translation while held (same stick mapping and
 * speed scale as the default drive — the suppliers already return m/s).
 * Release B → default field-centric drive resumes automatically.
 *
 * isAimed() returns true when tx has been within tolerance for
 * SETTLE_LOOPS consecutive loops (~100ms). Gate shooter fire on this.
 * Aim tuning lives in AimController.
 */
public class AutoAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem         vision;
    private final DoubleSupplier          xVelSupplier;
    private final DoubleSupplier          yVelSupplier;

    private final AimController              aim = new AimController();
    private final SwerveRequest.FieldCentric driveRequest;

    /**
     * @param xVelSupplier field-centric X velocity (m/s), already deadbanded/scaled
     * @param yVelSupplier field-centric Y velocity (m/s), already deadbanded/scaled
     * @param deadbandMps  translation deadband (m/s), same as the default drive
     */
    public AutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            double deadbandMps) {

        this.drivetrain   = drivetrain;
        this.vision       = vision;
        this.xVelSupplier = xVelSupplier;
        this.yVelSupplier = yVelSupplier;

        // No rotational deadband: small aim corrections must reach the wheels.
        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(deadbandMps)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        aim.reset();
    }

    @Override
    public void execute() {
        double rot = aim.calculate(vision);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xVelSupplier.getAsDouble())
                .withVelocityY(yVelSupplier.getAsDouble())
                .withRotationalRate(rot)
        );

        SmartDashboard.putBoolean("AutoAim/IsAimed",    aim.isAimed());
        SmartDashboard.putBoolean("AutoAim/HasTarget",  vision.hasShooterTarget());
        SmartDashboard.putNumber ("AutoAim/TX",         vision.getShooterTx());
        SmartDashboard.putNumber ("AutoAim/RotOutput",  rot);
        SmartDashboard.putNumber ("AutoAim/Distance_m", vision.getEstimatedDistanceMeters());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        aim.reset();
        SmartDashboard.putBoolean("AutoAim/IsAimed", false);
    }

    @Override
    public boolean isFinished() { return false; } // whileTrue handles termination

    /** True when tx settled within tolerance for ~100ms. Gate shooter fire on this. */
    public boolean isAimed() { return aim.isAimed(); }
}
