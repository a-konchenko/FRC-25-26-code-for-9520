package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.JetsonVision;

/**
 * AlignToTargetCommand
 *
 * Drives the robot toward the target coordinates provided by the Jetson.
 * Intended for teleop use — bind it to a button (e.g. left trigger) so the
 * driver can tap a button to snap to the nearest detected target.
 *
 * Uses two PID controllers:
 *   - X axis (forward/back)
 *   - Y axis (left/right)
 *
 * Rotation is NOT corrected here — the driver keeps rotational control.
 * Add a third PID + heading target if you want full auto-rotate too.
 *
 * Tune kP values on the robot — start low (0.5) and increase until snappy.
 */
public class AlignToTargetCommand extends Command {

    // ── PID tuning ───────────────────────────────────────────────────────────
    // kP: how aggressively to correct. kI/kD: start at 0, tune later if needed.
    private static final double kPX         = 1.0;   // tune me
    private static final double kPY         = 1.0;   // tune me
    private static final double kToleranceM = 0.05;  // within 5 cm = "close enough"

    private final CommandSwerveDrivetrain m_drivetrain;
    private final JetsonVision            m_vision;

    private final PIDController m_xController = new PIDController(kPX, 0, 0);
    private final PIDController m_yController = new PIDController(kPY, 0, 0);

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToTargetCommand(CommandSwerveDrivetrain drivetrain, JetsonVision vision) {
        m_drivetrain = drivetrain;
        m_vision     = vision;

        m_xController.setTolerance(kToleranceM);
        m_yController.setTolerance(kToleranceM);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Snap the setpoint to wherever the target is right now
        m_xController.setSetpoint(m_vision.getTargetX());
        m_yController.setSetpoint(m_vision.getTargetY());
    }

    @Override
    public void execute() {
        if (!m_vision.isDataFresh()) {
            // Jetson data is stale – stop driving, don't slam into something
            m_drivetrain.setControl(
                m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
            return;
        }

        // Get current robot pose for PID feedback
        double currentX = m_drivetrain.getState().Pose.getX();
        double currentY = m_drivetrain.getState().Pose.getY();

        double vx = m_xController.calculate(currentX);
        double vy = m_yController.calculate(currentY);

        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(0) // driver keeps rotation control
        );
    }

    @Override
    public boolean isFinished() {
        // Done when both axes are within tolerance, or Jetson loses the target
        return (m_xController.atSetpoint() && m_yController.atSetpoint())
            || !m_vision.isGamePieceDetected();
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop motors when command ends
        m_drivetrain.setControl(
            m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }
}
