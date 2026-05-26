package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.JetsonVision;

/**
 * JetsonDriveCommand
 *
 * Hands full drive authority to the Jetson during autonomous.
 * The Jetson publishes driveVX, driveVY, and driveOmega to NetworkTables
 * and this command passes them straight to the swerve drivetrain.
 *
 * Safety: if the Jetson data goes stale (no update within 0.5 s),
 * the robot stops immediately rather than continuing blind.
 *
 * Use in getAutonomousCommand() instead of (or after) the simple drive-forward.
 */
public class JetsonDriveCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final JetsonVision            m_vision;

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public JetsonDriveCommand(CommandSwerveDrivetrain drivetrain, JetsonVision vision) {
        m_drivetrain = drivetrain;
        m_vision     = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!m_vision.isDataFresh()) {
            // Safety stop if Jetson goes quiet
            m_drivetrain.setControl(
                m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
            return;
        }

        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(m_vision.getDriveVX())
                .withVelocityY(m_vision.getDriveVY())
                .withRotationalRate(m_vision.getDriveOmega())
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(
            m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted or auto ends
    }
}
