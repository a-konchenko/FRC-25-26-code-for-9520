package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase {
    public static final int kShooterMotorId = 13;
    public static final int kFeedMotorId    = 14;

    private final TalonFX m_shooter = new TalonFX(kShooterMotorId, TunerConstants.kCANBus);
    private final TalonFX m_shooterFeed = new TalonFX(kFeedMotorId, TunerConstants.kCANBus);

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_neutralRequest = new NeutralOut();

    // Tune these RPS (Rotations Per Second) values / side note miiight be too fast for our gears idk :p
    private double m_shooterRPS = 55.0;  // ~3300 RPM, adjust as needed
    private double m_feedRPS = -46.0;     // ~2760 RPM, adjust as needed

    /** Flywheel counts as "at speed" within this fraction of the setpoint. */
    private static final double kAtSpeedTolerance = 0.05;

    public Shooter() {
        // Configure PID gains for shooter motor /just more precice cfgs
        var shooterConfig = new TalonFXConfiguration();
        Slot0Configs shooterSlot0 = shooterConfig.Slot0;
        shooterSlot0.kP = 0.11;   // start here, tune if needed
        shooterSlot0.kI = 0.0;
        shooterSlot0.kD = 0.0;
        shooterSlot0.kV = 0.12;   // feedforward most important for velocity control
        shooterSlot0.kS = 0.05;   // static friction compensation
        m_shooter.getConfigurator().apply(shooterConfig);

        // Configure PID gains for feed motor /more tuning
        var feedConfig = new TalonFXConfiguration();
        Slot0Configs feedSlot0 = feedConfig.Slot0;
        feedSlot0.kP = 0.11;
        feedSlot0.kI = 0.0;
        feedSlot0.kD = 0.0;
        feedSlot0.kV = 0.12;
        feedSlot0.kS = 0.05;
        m_shooterFeed.getConfigurator().apply(feedConfig);

        // Push initial values to SmartDashboard /rps to dashboard
        SmartDashboard.putNumber("Shooter/ShooterRPS", m_shooterRPS);
        SmartDashboard.putNumber("Shooter/FeedRPS", m_feedRPS);
    }

    /** Spin the flywheel only. The feeder stays stopped so no ball leaves early. */
    public void spinUp() {
        m_shooter.setControl(m_velocityRequest.withVelocity(m_shooterRPS));
        m_shooterFeed.setControl(m_neutralRequest);
    }

    /** Flywheel at speed + feeder running = balls go out. */
    public void feed() {
        m_shooter.setControl(m_velocityRequest.withVelocity(m_shooterRPS));
        m_shooterFeed.setControl(m_velocityRequest.withVelocity(m_feedRPS));
    }

    /**
     * Teleop helper: spin up, and only feed once the flywheel is at speed.
     * BUG FIX (2026-10): this used to start the feeder at the same moment as
     * the flywheel, so balls left before the wheel was up to speed.
     */
    public void runShooter() {
        if (isAtSpeed()) feed(); else spinUp();
    }

    public void stop() {
        m_shooter.setControl(m_neutralRequest);
        m_shooterFeed.setControl(m_neutralRequest);
    }

    public boolean isAtSpeed() {
        double actual = m_shooter.getVelocity().getValueAsDouble();
        return Math.abs(m_shooterRPS) > 1e-3
            && Math.abs(actual - m_shooterRPS) <= Math.abs(m_shooterRPS) * kAtSpeedTolerance;
    }

    public boolean isFeeding() {
        return Math.abs(m_shooterFeed.getVelocity().getValueAsDouble()) > Math.abs(m_feedRPS) * 0.5;
    }

    public double getShooterSetpointRPS() { return m_shooterRPS; }
    public double getFeedSetpointRPS()    { return m_feedRPS; }

    public void setShooterRPS(double rps) { m_shooterRPS = rps; }
    public void setFeedRPS(double rps) { m_feedRPS = rps; }

    @Override
    public void periodic() {
        // Read live updates from SmartDashboard /may comment this out later depending on latency
        m_shooterRPS = SmartDashboard.getNumber("Shooter/ShooterRPS", m_shooterRPS);
        m_feedRPS = SmartDashboard.getNumber("Shooter/FeedRPS", m_feedRPS);

        // Monitor actual vs target velocity for tuning / again same here buuut might be unnessecary
        SmartDashboard.putNumber("Shooter/ShooterActualRPS", m_shooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/FeedActualRPS", m_shooterFeed.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterCurrent", m_shooter.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/FeedCurrent", m_shooterFeed.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
    }
}
