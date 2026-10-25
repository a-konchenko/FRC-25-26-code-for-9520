package frc.robot.sim;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * ★ NEW (sim only) — Pause / Resume / Step for the whole simulation.
 *
 * Uses WPILib SimHooks: while paused, robot time stops — the TimedRobot
 * loop, the CTRE swerve sim Notifier and every Timer freeze together.
 * Because the robot loop itself is frozen, the UI's pause/step/resume
 * actions are read on this small wall-clock thread instead.
 *
 * Actions (UI → robot): /Sim/Control/clock = [seq, op, arg]
 *   op 0 = resume, 1 = pause, 2 = step arg seconds (only while paused)
 * State (robot → UI):   /Sim/Clock/paused
 */
public class SimClock {
    private final DoubleArraySubscriber clockAction = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("/Sim/Control/clock").subscribe(new double[0]);
    private final BooleanPublisher pausedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("/Sim/Clock/paused").publish();

    private volatile boolean paused = false;
    private double lastSeq = Double.NaN;

    public SimClock() {
        pausedPub.set(false);
        Thread t = new Thread(this::run, "SimClock");
        t.setDaemon(true);
        t.start();
    }

    public boolean isPaused() { return paused; }

    public synchronized void pause() {
        if (!paused) {
            SimHooks.pauseTiming();
            paused = true;
            pausedPub.set(true);
        }
    }

    public synchronized void resume() {
        if (paused) {
            SimHooks.resumeTiming();
            paused = false;
            pausedPub.set(false);
        }
    }

    /** Advance robot time by `seconds` while paused (e.g. one 20 ms loop). */
    public synchronized void step(double seconds) {
        if (paused) SimHooks.stepTiming(seconds);
    }

    private void run() {
        while (true) {
            double[] v = clockAction.get();
            if (v.length >= 2 && v[0] != lastSeq) {
                boolean first = Double.isNaN(lastSeq);
                lastSeq = v[0];
                if (!first || v[0] != 0) {
                    switch ((int) v[1]) {
                        case 0 -> resume();
                        case 1 -> pause();
                        case 2 -> step(v.length >= 3 ? v[2] : 0.02);
                        default -> { }
                    }
                }
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return;
            }
        }
    }
}
