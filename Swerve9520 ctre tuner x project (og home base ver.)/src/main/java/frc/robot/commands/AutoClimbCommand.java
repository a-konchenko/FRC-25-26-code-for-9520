package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs both climber motors upward for a fixed duration in autonomous.
 * Does not require a subsystem because the climbers are raw TalonFX motors
 * managed directly in RobotContainer — we just call the setClimbers lambda.
 *
 * Tuning:
 *   CLIMB_SECONDS – how long to run the climbers. Start short (1-2 s) and
 *                   increase until the robot is reliably latched.
 */
public class AutoClimbCommand extends Command {

    // ── Tune this on the robot ────────────────────────────────────────────────
    private static final double CLIMB_SECONDS = 2.0;   // seconds to run climbers up
    // ─────────────────────────────────────────────────────────────────────────

    private final Runnable m_climbUp;
    private final Runnable m_climbStop;
    private final Timer    m_timer = new Timer();

    /**
     * @param climbUp   Lambda that drives both climbers upward
     * @param climbStop Lambda that stops both climbers
     */
    public AutoClimbCommand(Runnable climbUp, Runnable climbStop) {
        m_climbUp   = climbUp;
        m_climbStop = climbStop;
        // No subsystem requirements — motors accessed via lambdas
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_climbUp.run();
    }

    @Override
    public void execute() {
        // Keep running upward — climbers are continuous
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= CLIMB_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        m_climbStop.run();
    }
}
