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
 */
public class SimControl {
    public static final double kAutoDurationS = 20.0; // 2026 game manual 7.4.1

    private final SimManager     sim;
    private final RobotContainer container;
    private final SimNT          nt;

    private enum State { IDLE, ARMED, RUNNING, FINISHED }
    private State   state = State.IDLE;
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
            case ARMED -> {
                if (DriverStation.isAutonomousEnabled()) {
                    state = State.RUNNING;
                    runStart = now;
                    commandEnd = -1.0;
                    lastElapsed = 0.0;
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
        if (state == State.ARMED || state == State.RUNNING) finish();
    }

    private void finish() {
        state = State.FINISHED;
        container.setAutonomousOverride(null);
        sim.setEnabled(false, false);
    }

    // ── Play / Reset ────────────────────────────────────────────────────────

    private void play(String name, double[] args) {
        int kind = (int) arg(args, 0);
        boolean red = arg(args, 1) > 0.5;
        sim.cancelTestDrive();
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
        sim.setEnabled(true, true);
        state = State.ARMED;
        System.out.println("[Sim] Play " + kindName(kind) + " '" + name + "' (" + (red ? "red" : "blue") + ")");
    }

    private void reset(String name, double[] args) {
        int kind = (int) arg(args, 0);
        boolean red = arg(args, 1) > 0.5;
        sim.cancelTestDrive();
        if (state == State.ARMED || state == State.RUNNING) finish();
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
    }

    private static double arg(double[] a, int i) { return a != null && i < a.length ? a[i] : 0.0; }

    private static String kindName(int kind) {
        return switch (kind) { case 0 -> "auto"; case 1 -> "path"; default -> "hand-coded auto"; };
    }
}
