package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Spins up the shooter, waits for it to reach speed, then feeds the ball
 * for a set duration. Used exclusively in autonomous.
 *
 * Tuning constants at the top of this file:
 *   SPINUP_SECONDS  – how long to wait for the shooter to reach full speed
 *   SHOOT_SECONDS   – how long to run the intake to feed the ball
 */
public class AutoShootCommand extends Command {

    // ── Tune these on the robot ───────────────────────────────────────────────
    private static final double SPINUP_SECONDS = 1.0;  // wait for shooter to rev up
    private static final double SHOOT_SECONDS  = 1.5;  // time to feed ball through
    // ─────────────────────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final Runnable m_startIntake;
    private final Runnable m_stopIntake;

    private final Timer m_timer = new Timer();
    private boolean m_intakeStarted = false;

    /**
     * @param shooter      The Shooter subsystem
     * @param startIntake  Lambda that starts the intake motor (from RobotContainer)
     * @param stopIntake   Lambda that stops the intake motor (from RobotContainer)
     */
    public AutoShootCommand(Shooter shooter, Runnable startIntake, Runnable stopIntake) {
        m_shooter    = shooter;
        m_startIntake = startIntake;
        m_stopIntake  = stopIntake;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_intakeStarted = false;
        m_shooter.runShooter();   // start spinning up immediately
    }

    @Override
    public void execute() {
        double elapsed = m_timer.get();

        // After spinup period, start feeding the ball
        if (elapsed >= SPINUP_SECONDS && !m_intakeStarted) {
            m_startIntake.run();
            m_intakeStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Done once we've fed for the full shoot duration after spinup
        return m_timer.get() >= SPINUP_SECONDS + SHOOT_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_stopIntake.run();
    }
}
