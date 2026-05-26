package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.TargetMode;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.auto.ShootAllAuto;

public class RobotContainer {

    // -- Speeds (change 0.75 back to 1.0 for comp) --
    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // -- Subsystems --
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Shooter shooter = new Shooter();

    public final VisionSubsystem vision = new VisionSubsystem(
        (pose, timestamp, stdDevs) ->
            drivetrain.addVisionMeasurement(pose, timestamp, stdDevs),
        () -> drivetrain.getState().Pose.getRotation()
    );

    private AutoAimCommand autoAimCommand;

    // -- Auto chooser (Elastic picks this up as a combo box) --
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // -- Target mode chooser (Elastic combo box to switch LL4 pipeline) --
    private final SendableChooser<TargetMode> targetModeChooser = new SendableChooser<>();
    private TargetMode lastAppliedTargetMode = null;

    // -- Slew rate limiters --
    //private final SlewRateLimiter m_xLimiter   = new SlewRateLimiter(2.7);
    //private final SlewRateLimiter m_yLimiter   = new SlewRateLimiter(2.7);
    //private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.7);

    // -- Swerve requests --
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    // -- Intake --
    private static final int    kIntakeMotorId = 15;
    private static final double kIntakeSpeed   = -1.00;
    private final TalonFX       m_intakeMotor  = new TalonFX(kIntakeMotorId);
    private final DutyCycleOut  m_intakeDutyCycle = new DutyCycleOut(0);
    private boolean             m_intakeRunning   = false;

    // -- Intake arm --
    private static final int    kIntakeArmMotorId = 19;
    private static final double kIntakeArmSpeed   = 0.4;
    private final TalonFX       m_intakeArmMotor  = new TalonFX(kIntakeArmMotorId);
    private final DutyCycleOut  m_intakeArmDutyCycle = new DutyCycleOut(0);

    // -- Climbers --
    private static final int    kClimberLeftMotorId  = 16;
    private static final int    kClimberRightMotorId = 17;
    private static final double kClimberSpeed        = 0.4;
    private final TalonFX       m_climberLeft  = new TalonFX(kClimberLeftMotorId);
    private final TalonFX       m_climberRight = new TalonFX(kClimberRightMotorId);
    private final DutyCycleOut  m_climberLeftDutyCycle  = new DutyCycleOut(0);
    private final DutyCycleOut  m_climberRightDutyCycle = new DutyCycleOut(0);
    private boolean m_climberIndividualDirectionUp = true;

    // =========================================================
    public RobotContainer() {
        autoAimCommand = new AutoAimCommand(
            drivetrain, vision,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX()
        );

        configureIntake();
        configureIntakeArm();
        configureClimbers();
        configureAuto();
        configureBindings();
    }

    // -- Hardware config --
    private void configureIntake() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode            = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit       = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_intakeMotor.getConfigurator().apply(cfg);
    }

    private void configureIntakeArm() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode            = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit       = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_intakeArmMotor.getConfigurator().apply(cfg);
    }

    private void configureClimbers() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode            = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit       = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_climberLeft.getConfigurator().apply(cfg);
        m_climberRight.getConfigurator().apply(cfg);
    }

    // -- Auto config --
    private void configureAuto() {
        autoChooser.setDefaultOption("Shoot All (Stationary)",
            new ShootAllAuto(drivetrain, shooter, vision));
        autoChooser.addOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward 5s", driveForwardAuto());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Target mode chooser — dropdown in Elastic to switch LL4 pipeline.
        // Alliance-correct hub is the default. Change from the dashboard
        // any time the robot is disabled or during a match.
        targetModeChooser.setDefaultOption("Hub (Blue)",    TargetMode.HUB_BLUE);
        targetModeChooser.addOption("Hub (Red)",            TargetMode.HUB_RED);
        targetModeChooser.addOption("Trench (Blue)",        TargetMode.TRENCH_BLUE);
        targetModeChooser.addOption("Trench (Red)",         TargetMode.TRENCH_RED);

        SmartDashboard.putData("Target Mode", targetModeChooser);
    }

    private Command driveForwardAuto() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)
            ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }

    // -- Bindings --
    private void configureBindings() {

        // Default drive: -LeftY = forward (VelocityX), -LeftX = left (VelocityY)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX((joystick.getLeftX() * MaxSpeed))
                    .withVelocityY((-joystick.getLeftY() * MaxSpeed))
                    .withRotationalRate((joystick.getRightX() * MaxAngularRate))
            )
        );

        // Idle when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Sync LL4 pipeline to alliance color on disable
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(vision::setAllianceDefaultMode)
        );

        // A = Brake
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B = Auto-aim (hold)
        joystick.b().whileTrue(autoAimCommand);

        // RB = Toggle hub/trench targeting
        joystick.rightBumper().onTrue(Commands.runOnce(vision::toggleHubTrench));

        // LB = Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Y = Shoot (hold to spin, release to stop)
        joystick.y().whileTrue(Commands.runEnd(
            () -> shooter.runShooter(),
            () -> shooter.stop(),
            shooter
        ));

        // X = Intake toggle (guarded from SysId combos)
        joystick.x()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(this::toggleIntake));

        // RT = Intake arm up
        joystick.rightTrigger().whileTrue(Commands.runEnd(
            () -> m_intakeArmMotor.setControl(m_intakeArmDutyCycle.withOutput( kIntakeArmSpeed)),
            () -> m_intakeArmMotor.setControl(m_intakeArmDutyCycle.withOutput(0.0))
        ));

        // LT = Intake arm down
        joystick.leftTrigger().whileTrue(Commands.runEnd(
            () -> m_intakeArmMotor.setControl(m_intakeArmDutyCycle.withOutput(-kIntakeArmSpeed)),
            () -> m_intakeArmMotor.setControl(m_intakeArmDutyCycle.withOutput(0.0))
        ));

        // SysId (Back/Start + face buttons)
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Climbers
        joystick.povUp().whileTrue(Commands.runEnd(
            () -> setClimbers( kClimberSpeed), () -> setClimbers(0.0)));
        joystick.povDown().whileTrue(Commands.runEnd(
            () -> setClimbers(-kClimberSpeed), () -> setClimbers(0.0)));
        joystick.povLeft().whileTrue(Commands.runEnd(
            () -> setClimberLeft( m_climberIndividualDirectionUp ?  kClimberSpeed : -kClimberSpeed),
            () -> setClimberLeft(0.0)));
        joystick.povRight().whileTrue(Commands.runEnd(
            () -> setClimberRight(m_climberIndividualDirectionUp ?  kClimberSpeed : -kClimberSpeed),
            () -> setClimberRight(0.0)));

        // D-pad center + X = toggle climber direction
        joystick.povCenter().and(joystick.x()).onTrue(
            Commands.runOnce(this::toggleClimberDirection));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // -- Helpers --
    private void toggleIntake() {
        m_intakeRunning = !m_intakeRunning;
        m_intakeMotor.setControl(
            m_intakeDutyCycle.withOutput(m_intakeRunning ? kIntakeSpeed : 0.0));
        SmartDashboard.putBoolean("Intake/Running", m_intakeRunning);
    }

    private void setClimbers(double speed) {
        m_climberLeft.setControl(m_climberLeftDutyCycle.withOutput(speed));
        m_climberRight.setControl(m_climberRightDutyCycle.withOutput(speed));
    }

    private void setClimberLeft(double speed) {
        m_climberLeft.setControl(m_climberLeftDutyCycle.withOutput(speed));
    }

    private void setClimberRight(double speed) {
        m_climberRight.setControl(m_climberRightDutyCycle.withOutput(speed));
    }

    private void toggleClimberDirection() {
        m_climberIndividualDirectionUp = !m_climberIndividualDirectionUp;
        SmartDashboard.putString("Climber/Dir",
            m_climberIndividualDirectionUp ? "UP" : "DOWN");
    }

    // -- Autonomous --
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Call from robotPeriodic(). Checks if the dashboard target mode
     * chooser changed and applies the new pipeline to the LL4.
     */
    public void pollTargetModeChooser() {
        TargetMode selected = targetModeChooser.getSelected();
        if (selected != null && selected != lastAppliedTargetMode) {
            vision.setTargetMode(selected);
            lastAppliedTargetMode = selected;
        }
    }

    public boolean isAimed() { return autoAimCommand.isAimed(); }
}
