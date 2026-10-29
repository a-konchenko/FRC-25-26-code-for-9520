package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Pins the timing of the named shooting commands (Shoot, ShootAll, AimAndShoot).
 * Team decision (2026-10): if the flywheel never reaches speed, feed anyway
 * after SPIN_UP_TIMEOUT_S so the robot still empties in auto.
 */
class ShootCommandsTest {
    private static final double LOOP = 0.02;

    private final Subsystem shooter = new Subsystem() {};
    private boolean atSpeed;
    private int spinUps, stops;
    private double feedStart, feedEnd, t0;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        SimHooks.pauseTiming(); // time only moves when the test steps it
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        atSpeed = false;
        spinUps = stops = 0;
        feedStart = feedEnd = -1;
        t0 = Timer.getFPGATimestamp();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run();
        SimHooks.resumeTiming();
    }

    private Command shoot(double feedSeconds) {
        return ShootCommands.shootFor(shooter, () -> spinUps++, () -> atSpeed,
            () -> { double t = now(); if (feedStart < 0) feedStart = t; feedEnd = t; }, () -> stops++, feedSeconds);
    }

    private double now() { return Timer.getFPGATimestamp() - t0; }

    /** Run robot loops for `seconds`, optionally flipping at-speed on at `atSpeedAt`. */
    private void loops(double seconds, double atSpeedAt) {
        for (int i = 0; i < Math.round(seconds / LOOP); i++) {
            if (atSpeedAt >= 0 && now() >= atSpeedAt) atSpeed = true;
            CommandScheduler.getInstance().run();
            SimHooks.stepTiming(LOOP);
        }
    }

    @Test
    void feedsAnywayWhenTheFlywheelNeverReachesSpeed() {
        CommandScheduler.getInstance().schedule(shoot(1.0));
        loops(3.0, -1);
        assertEquals(ShootCommands.SPIN_UP_TIMEOUT_S, feedStart, 2 * LOOP, "feeds after the spin-up timeout");
        assertEquals(ShootCommands.SPIN_UP_TIMEOUT_S + 1.0, feedEnd, 2 * LOOP, "for the full feed time");
        assertTrue(spinUps > 60, "spun up the whole time before feeding (" + spinUps + " loops)");
        assertEquals(1, stops, "stopped once at the end");
    }

    @Test
    void feedsAsSoonAsTheFlywheelIsAtSpeed() {
        CommandScheduler.getInstance().schedule(shoot(1.0));
        loops(2.0, 0.4);
        assertEquals(0.4, feedStart, 2 * LOOP, "no waiting once at speed");
        assertEquals(1, stops);
    }

    @Test
    void interruptingStillStopsTheShooter() {
        Command c = shoot(4.0);
        CommandScheduler.getInstance().schedule(c);
        loops(0.5, 0.1);
        c.cancel();
        assertEquals(1, stops, "stop runs when interrupted (e.g. auto ends)");
    }
}
