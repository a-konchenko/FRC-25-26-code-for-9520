package frc.robot.sim;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

/**
 * ★ NEW (sim only) — match control: Play / Reset an autonomous routine.
 *
 * Play (UI → /Sim/Control/play = [seq, kind, red, errX, errY, errTheta],
 * name in /Sim/Control/run/name):
 *   1. stop whatever is running, disable
 *   2. build the command FRESH from the files (edits in the Plan tab or the
 *      PathPlanner app are picked up without restarting the sim)
 *   3. put the robot code's estimate on the starting pose, and the TRUE robot
 *      on starting pose + placement error (how far off the students set it down)
 *   4. hand the command to RobotContainer and enable autonomous — the robot
 *      code's own Robot.autonomousInit() schedules it, exactly like a match
 *   5. disable after 20 s (2026 AUTO period)
 *
 * kind: 0 = PathPlanner auto (.auto), 1 = single path (.path), 2 = hand-coded
 * auto from RobotContainer.
 *
 * The robot code checks its mode once per loop, at the start of the loop, so a
 * disable and re-enable within one loop is never seen (Robot.autonomousInit
 * would not run again). DriverStationSim updates DriverStation.isEnabled()
 * immediately, so that can't tell us what the robot code saw. Play while the
 * robot is enabled therefore DISARMS first: it disables, waits at least one
 * full robot loop (the robot code then runs disabledInit), and only then enables.
 */
public class SimControl {
    public static final double kAutoDurationS = 20.0; // 2026 game manual 7.4.1
    /** Give up if the robot code doesn't disable / start autonomous within this long. */
    static final double kArmTimeoutS = 3.0;

    private final SimManager     sim;
    private final RobotContainer container;
    private final SimNT          nt;

    private enum State { IDLE, DISARMING, ARMED, RUNNING, FINISHED }
    private State   state = State.IDLE;
    private double  stateSince = 0.0;
    /** update() calls (= robot loops) so far, and the loop the current state started in. */
    private long    loop = 0, stateLoop = 0;
    private Command runCommand = null;
    private double  runStart = 0.0;
    private double  commandEnd = -1.0;
    private double  lastElapsed = 0.0;
    private int     runId = 0;
    private String  runName = "";
    private int     runKind = 0;
    private String  lastError = "";
    private double  lastListPublish = -10.0;

    public SimControl(SimManager sim, RobotContainer container, SimNT nt) {
        this.sim = sim;
        this.container = container;
        this.nt = nt;
    }

    public void update() {
        loop++;
        double now = Timer.getFPGATimestamp();
        if (now - lastListPublish > 1.0) {
            publishLists();
            lastListPublish = now;
        }

        double[] play = nt.action("play");
        if (play != null) play(nt.settingString("run/name", ""), play);

        double[] reset = nt.action("reset");
        if (reset != null) reset(nt.settingString("run/name", ""), reset);

        switch (state) {
            case DISARMING -> {
                // A later loop than the one Play disabled in: the robot code has seen "disabled"
                if (loop > stateLoop && DriverStation.isDisabled()) {
                    sim.setEnabled(true, true);
                    state = State.ARMED;
                    stateSince = now;
                } else if (now - stateSince > kArmTimeoutS) {
                    giveUp("the robot code did not disable");
                }
            }
            case ARMED -> {
                if (DriverStation.isAutonomousEnabled()) {
                    state = State.RUNNING;
                    runStart = now;
                    commandEnd = -1.0;
                    lastElapsed = 0.0;
                } else if (now - stateSince > kArmTimeoutS) {
                    giveUp("the robot code never started autonomous");
                }
            }
            case RUNNING -> {
                double elapsed = now - runStart;
                lastElapsed = elapsed;
                DriverStationSim.setMatchTime(Math.max(0.0, kAutoDurationS - elapsed));
                DriverStationSim.notifyNewData(); // robot code reads match time from cached DS data
                if (commandEnd < 0 && runCommand != null && !runCommand.isScheduled() && elapsed > 0.1) {
                    commandEnd = elapsed;
                }
                if (elapsed >= kAutoDurationS || !DriverStation.isAutonomousEnabled()) {
                    finish();
                }
            }
            default -> { }
        }
        publishState(now);
    }

    /** The UI's Stop button / test drive also end a run. */
    public void stop() {
        if (active()) finish();
    }

    private boolean active() {
        return state == State.DISARMING || state == State.ARMED || state == State.RUNNING;
    }

    private void finish() {
        state = State.FINISHED;
        container.setAutonomousOverride(null);
        sim.setEnabled(false, false);
    }

    /** A run that never got going: never leave its command queued for the next autonomous enable. */
    private void giveUp(String why) {
        lastError = kindName(runKind) + " '" + runName + "': " + why;
        System.out.println("[Sim] Play stopped: " + lastError);
        container.setAutonomousOverride(null);
        sim.setEnabled(false, false);
        state = State.IDLE;
    }

    // ── Play / Reset ────────────────────────────────────────────────────────

    private void play(String name, double[] args) {
        int kind = (int) arg(args, 0);
        boolean red = arg(args, 1) > 0.5;
        // What the robot code saw this loop (it reads the DS once per loop)
        boolean wasEnabled = DriverStation.isEnabled();
        sim.cancelTestDrive();
        if (runCommand != null && runCommand.isScheduled()) runCommand.cancel();
        container.setAutonomousOverride(null); // a failed build must not leave the old run queued
        sim.setEnabled(false, false);
        setAlliance(red);

        Built built = build(kind, name, red);
        if (built == null) { state = State.IDLE; return; }

        placeAtStart(built.start, args);
        sim.resetFuel();
        runCommand = built.command;
        runName = name;
        runKind = kind;
        runId++;
        container.setAutonomousOverride(runCommand);
        if (wasEnabled) {
            state = State.DISARMING; // enable once the robot code has seen the disable (see update)
        } else {
            sim.setEnabled(true, true);
            state = State.ARMED;
        }
        stateSince = Timer.getFPGATimestamp();
        stateLoop = loop;
        System.out.println("[Sim] Play" + kindName(kind) + " '" + name + "' (" + (red ? "red" : "blue") + ")");
    }

    private void reset(String name, double[] args) {
        int kind = (int) arg(args, 0);
        boolean red = arg(args, 1) > 0.5;
        sim.cancelTestDrive();
        if (active()) finish();
        container.setAutonomousOverride(null);
        sim.setEnabled(false, false);
        setAlliance(red);
        Built built = build(kind, name, red);
        if (built != null) placeAtStart(built.start, args);
        sim.resetFuel();
        state = State.IDLE;
        runId++;
    }

    private record Built(Command command, Pose2d start) {}

    private Built build(int kind, String name, boolean red) {
        lastError = "";
        try {
            PathPlannerPath.clearCache(); // pick up edited files
            switch (kind) {
                case 0 -> {
                    var auto = new PathPlannerAuto(name);
                    Pose2d start = auto.getStartingPose();
                    if (start != null && red) start = FlippingUtil.flipFieldPose(start);
                    return new Built(auto, start);
                }
                case 1 -> {
                    var path = PathPlannerPath.fromPathFile(name);
                    Pose2d start = path.getStartingHolonomicPose().orElse(null);
                    if (start != null && red) start = FlippingUtil.flipFieldPose(start);
                    return new Built(AutoBuilder.followPath(path), start);
                }
                case 2 -> {
                    var supplier = container.getHandCodedAutos().get(name);
                    if (supplier == null) throw new IllegalArgumentException("No hand-coded auto named '" + name + "'");
                    return new Built(supplier.get(), null); // hand-coded autos start wherever the robot is
                }
                default -> throw new IllegalArgumentException("Unknown run kind " + kind);
            }
        } catch (Exception e) {
            lastError = kindName(kind) + " '" + name + "': " + e.getClass().getSimpleName() + ": " + e.getMessage();
            System.out.println("[Sim] Could not build " + lastError);
            return null;
        }
    }

    /** Estimate on the start pose; truth on start + placement error (field frame). */
    private void placeAtStart(Pose2d start, double[] args) {
        Pose2d estimate = start != null ? start : container.drivetrain.getState().Pose;
        Pose2d truth = new Pose2d(
            estimate.getX() + arg(args, 2),
            estimate.getY() + arg(args, 3),
            estimate.getRotation().plus(Rotation2d.fromRadians(arg(args, 4))));
        sim.placeRobot(estimate, truth);
    }

    private void setAlliance(boolean red) {
        DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    // ── Publishing ──────────────────────────────────────────────────────────

    private void publishLists() {
        File pp = new File(Filesystem.getDeployDirectory(), "pathplanner");
        nt.strings("Autos/pathplanner", list(new File(pp, "autos"), ".auto"));
        nt.strings("Autos/paths",       list(new File(pp, "paths"), ".path"));
        nt.strings("Autos/handCoded",   container.getHandCodedAutos().keySet().toArray(new String[0]));
        nt.strings("NamedCommands",     container.getNamedCommandNames().toArray(new String[0]));
    }

    private static String[] list(File dir, String ext) {
        File[] files = dir.listFiles((d, n) -> n.endsWith(ext) && !n.startsWith("."));
        if (files == null) return new String[0];
        List<String> names = new ArrayList<>();
        for (File f : files) names.add(f.getName().substring(0, f.getName().length() - ext.length()));
        String[] out = names.toArray(new String[0]);
        Arrays.sort(out, String.CASE_INSENSITIVE_ORDER);
        return out;
    }

    private void publishState(double now) {
        nt.string("Run/state", state.name().toLowerCase());
        nt.number("Run/id", runId);
        nt.string("Run/name", runName);
        nt.number("Run/kind", runKind);
        nt.number("Run/elapsed", state == State.RUNNING ? now - runStart
                               : state == State.FINISHED ? lastElapsed : 0.0);
        nt.number("Run/commandEnd", commandEnd);
        nt.number("Run/duration", kAutoDurationS);
        nt.string("Run/error", lastError);
        nt.bool("Run/overrideActive", container.hasAutonomousOverride());
    }

    private static double arg(double[] a, int i) { return a != null && i < a.length ? a[i] : 0.0; }

    private static String kindName(int kind) {
        return switch (kind) { case 0 -> "auto"; case 1 -> "path"; default -> "hand-coded auto"; };
    }
}
