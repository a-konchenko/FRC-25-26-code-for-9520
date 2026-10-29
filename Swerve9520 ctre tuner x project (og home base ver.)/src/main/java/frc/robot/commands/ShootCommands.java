package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Timed shooting used by the PathPlanner named commands (Shoot, ShootAll,
 * AimAndShoot, ...). Takes the shooter's actions as functions, so the timing
 * can be unit-tested without motors (ShootCommandsTest).
 */
public final class ShootCommands {
    private ShootCommands() {}

    /** Longest wait for the flywheel before feeding anyway. */
    public static final double SPIN_UP_TIMEOUT_S = 1.5;

    /**
     * Spin up (max SPIN_UP_TIMEOUT_S, until at speed), then feed for
     * feedSeconds, then stop. If the flywheel never reaches speed it feeds
     * anyway after the timeout, so the robot still empties in auto (team
     * decision, 2026-10; pinned by ShootCommandsTest).
     */
    public static Command shootFor(Subsystem shooter, Runnable spinUp, BooleanSupplier atSpeed,
                                   Runnable feed, Runnable stop, double feedSeconds) {
        return Commands.sequence(
            Commands.run(spinUp, shooter).until(atSpeed).withTimeout(SPIN_UP_TIMEOUT_S),
            Commands.run(feed, shooter).withTimeout(feedSeconds)
        ).finallyDo(stop);
    }
}
