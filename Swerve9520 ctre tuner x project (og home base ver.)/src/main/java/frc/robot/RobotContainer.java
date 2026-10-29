package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.ShootCommands;
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
    /** Hand-written autos by name. Suppliers, so a fresh command can be built each time. */
    private final Map<String, Supplier<Command>> m_handCodedAutos = new LinkedHashMap<>();
    /** Names registered with PathPlanner's NamedCommands (shown in the simulator's Plan tab). */
    private final List<String> m_namedCommandNames = new ArrayList<>();
    /** Simulator only: the auto to run instead of the chooser selection (null = use chooser). */
    private Command m_autoOverride = null;

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
            this::driverVelocityX,
            this::driverVelocityY,
            MaxSpeed * 0.1
        );

        configureIntake();
        configureIntakeArm();
        configureClimbers();
        registerNamedCommands(); // must happen before any PathPlanner auto is built
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

    // -- Named commands (used by PathPlanner autos and path event markers) --
    //
    // Two ways to use them in PathPlanner:
    //   * as a step in an .auto (runs in order) — anything is allowed
    //   * as an EVENT MARKER on a path (runs WHILE the robot drives) — must NOT
    //     require the drivetrain, or it would stop the path. Shooter/intake
    //     commands are fine; AutoAim / AimAndShoot are auto steps only.
    private void registerNamedCommands() {
        // Turn in place until LL4 tx is zero (drivetrain — auto steps only)
        var aimInPlace = new AutoAimCommand(drivetrain, vision, () -> 0.0, () -> 0.0, 0.0);
        registerNamed("AutoAim", aimInPlace.until(aimInPlace::isAimed).withTimeout(2.0));
        var aimThenShoot = new AutoAimCommand(drivetrain, vision, () -> 0.0, () -> 0.0, 0.0);
        registerNamed("AimAndShoot", Commands.sequence(
            aimThenShoot.until(aimThenShoot::isAimed).withTimeout(2.0),
            shootFor(4.0)));

        // Shooter / intake (safe as event markers)
        registerNamed("SpinUp",      Commands.runOnce(shooter::spinUp, shooter));
        registerNamed("Shoot",       shootFor(1.0));
        registerNamed("ShootAll",    shootFor(4.0));
        registerNamed("StopShooter", Commands.runOnce(shooter::stop, shooter));
        registerNamed("IntakeOn",    Commands.runOnce(() -> setIntake(true)));
        registerNamed("IntakeOff",   Commands.runOnce(() -> setIntake(false)));
        registerNamed("ArmDown",     armFor(-kIntakeArmSpeed, 0.6));
        registerNamed("ArmUp",       armFor( kIntakeArmSpeed, 0.6));

        // Names already used by "Test path.path" event markers
        registerNamed("WaitShoot1",   shootFor(1.0));
        registerNamed("Waitshootall", shootFor(4.0));
    }

    private void registerNamed(String name, Command command) {
        NamedCommands.registerCommand(name, command);
        m_namedCommandNames.add(name);
    }

    /** Spin up, feed for feedSeconds, stop (see ShootCommands.shootFor for the timing). */
    private Command shootFor(double feedSeconds) {
        return ShootCommands.shootFor(shooter, shooter::spinUp, shooter::isAtSpeed,
            shooter::feed, shooter::stop, feedSeconds);
    }

    private Command armFor(double output, double seconds) {
        return Commands.runEnd(
            () -> m_intakeArmMotor.setControl(m_intakeArmDutyCycle.withOutput(output)),
            () -> m_intakeArmMotor.setControl(m_intakeArmDutyCycle.withOutput(0.0))
        ).withTimeout(seconds);
    }

    // -- Auto config --
    private void configureAuto() {
        m_handCodedAutos.put("Shoot All (Stationary)", () -> new ShootAllAuto(drivetrain, shooter, vision));
        m_handCodedAutos.put("Do Nothing",             Commands::none);
        m_handCodedAutos.put("Drive Forward 5s",       this::driveForwardAuto);

        boolean first = true;
        for (var entry : m_handCodedAutos.entrySet()) {
            if (first) autoChooser.setDefaultOption(entry.getKey(), entry.getValue().get());
            else       autoChooser.addOption(entry.getKey(), entry.getValue().get());
            first = false;
        }

        // PathPlanner autos: every deploy/pathplanner/autos/*.auto, shown as "PP: <name>"
        if (AutoBuilder.isConfigured()) {
            for (String name : AutoBuilder.getAllAutoNames()) {
                try {
                    autoChooser.addOption("PP: " + name, new PathPlannerAuto(name));
                } catch (Exception e) {
                    DriverStation.reportError("Could not load PathPlanner auto '" + name + "': " + e.getMessage(), false);
                }
            }
        }

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

        // Default drive: -LeftY = forward (VelocityX), -LeftX = left (VelocityY),
        // -RightX = counter-clockwise. See driverVelocityX/Y/Rotation below.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(driverVelocityX())
                    .withVelocityY(driverVelocityY())
                    .withRotationalRate(driverRotationRate())
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

        // Y = Shoot (hold: spin up, feeds automatically once at speed; release to stop)
        // Guarded from the SysId combos (Back/Start + Y).
        joystick.y()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(Commands.runEnd(
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

        // Right stick click = toggle individual climber direction.
        // BUG FIX (2026-10): this was D-pad center + X, but the D-pad is
        // "center" whenever it isn't pressed, so every intake press (X)
        // also flipped the climber direction.
        joystick.rightStick().onTrue(
            Commands.runOnce(this::toggleClimberDirection));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // -- Driver stick mapping (one place, used by default drive AND auto-aim) --
    // BUG FIX (2026-10): the default drive used X = LeftX, Y = -LeftY and an
    // un-negated RightX, while AutoAim used X = -LeftY, Y = -LeftX at full
    // speed — so the sticks changed meaning while B was held. Both now use
    // the standard WPILib/CTRE mapping below.
    // If the REAL robot drives sideways with this mapping, the swerve encoder
    // offsets are 90 degrees off — re-zero them in Tuner X instead of
    // swapping axes here.
    private double driverVelocityX()    { return -joystick.getLeftY()  * MaxSpeed; }
    private double driverVelocityY()    { return -joystick.getLeftX()  * MaxSpeed; }
    private double driverRotationRate() { return -joystick.getRightX() * MaxAngularRate; }

    // -- Helpers --
    private void toggleIntake() {
        setIntake(!m_intakeRunning);
    }

    public void setIntake(boolean running) {
        m_intakeRunning = running;
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
        return m_autoOverride != null ? m_autoOverride : autoChooser.getSelected();
    }

    /** Simulator only: run this command in the next autonomous instead of the chooser selection. */
    public void setAutonomousOverride(Command command) { m_autoOverride = command; }

    /** Simulator only: whether an override is queued (the sim checks it never goes stale). */
    public boolean hasAutonomousOverride() { return m_autoOverride != null; }

    public Map<String, Supplier<Command>> getHandCodedAutos() { return Collections.unmodifiableMap(m_handCodedAutos); }

    public List<String> getNamedCommandNames() { return Collections.unmodifiableList(m_namedCommandNames); }

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
