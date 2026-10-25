package frc.robot.sim;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

/**
 * ★ NEW (sim only) — NetworkTables contract between the robot sim and the
 * web UI. Everything lives under /Sim. See docs/simulator.md for the table.
 *
 * Robot → UI: plain topics, poses as double[] {x, y, thetaRad}.
 * UI → robot: /Sim/Control/...
 *   - settings are plain doubles (latest value wins)
 *   - one-shot actions are double[] whose first element is a sequence
 *     number; the robot acts once each time the number changes.
 */
public class SimNT {
    private final NetworkTable root = NetworkTableInstance.getDefault().getTable("Sim");
    private final NetworkTable control = root.getSubTable("Control");

    private final Map<String, DoubleArrayPublisher> arrays  = new HashMap<>();
    private final Map<String, DoublePublisher>      doubles = new HashMap<>();
    private final Map<String, StringPublisher>      strings = new HashMap<>();
    private final Map<String, BooleanPublisher>     bools   = new HashMap<>();
    private final Map<String, StringArrayPublisher> stringArrays = new HashMap<>();
    private final Map<String, StringSubscriber>     controlStrings = new HashMap<>();

    private final Map<String, DoubleSubscriber>      controlDoubles = new HashMap<>();
    private final Map<String, DoubleArraySubscriber> actions        = new HashMap<>();
    private final Map<String, Double>                lastActionSeq  = new HashMap<>();

    // ── Robot → UI ──────────────────────────────────────────────────────────
    public void pose(String path, Pose2d p) {
        array(path, new double[] { p.getX(), p.getY(), p.getRotation().getRadians() });
    }

    public void array(String path, double[] value) {
        arrays.computeIfAbsent(path, k -> topic(k).getDoubleArrayTopic(leaf(k))
            .publish(PubSubOption.sendAll(false))).set(value);
    }

    public void number(String path, double value) {
        doubles.computeIfAbsent(path, k -> topic(k).getDoubleTopic(leaf(k)).publish()).set(value);
    }

    public void string(String path, String value) {
        strings.computeIfAbsent(path, k -> topic(k).getStringTopic(leaf(k)).publish()).set(value);
    }

    public void strings(String path, String[] value) {
        stringArrays.computeIfAbsent(path, k -> topic(k).getStringArrayTopic(leaf(k)).publish()).set(value);
    }

    public void bool(String path, boolean value) {
        bools.computeIfAbsent(path, k -> topic(k).getBooleanTopic(leaf(k)).publish()).set(value);
    }

    // ── UI → robot ──────────────────────────────────────────────────────────
    /** A setting under /Sim/Control (e.g. "drift/wheelScalePct"). */
    public double setting(String path, double defaultValue) {
        return controlDoubles.computeIfAbsent(path,
            k -> sub(control, k).getDoubleTopic(leaf(k)).subscribe(defaultValue)).get();
    }

    /** A string setting under /Sim/Control (e.g. "run/name"). */
    public String settingString(String path, String defaultValue) {
        return controlStrings.computeIfAbsent(path,
            k -> sub(control, k).getStringTopic(leaf(k)).subscribe(defaultValue)).get();
    }

    /**
     * A one-shot action under /Sim/Control. Returns the arguments (without the
     * sequence number) the first time a new sequence number arrives, else null.
     */
    public double[] action(String path) {
        var s = actions.computeIfAbsent(path,
            k -> sub(control, k).getDoubleArrayTopic(leaf(k)).subscribe(new double[0]));
        double[] v = s.get();
        if (v.length == 0) return null;
        Double last = lastActionSeq.get(path);
        if (last != null && last == v[0]) return null;
        lastActionSeq.put(path, v[0]);
        if (last == null && v[0] == 0) return null; // never sent yet
        double[] args = new double[v.length - 1];
        System.arraycopy(v, 1, args, 0, args.length);
        return args;
    }

    // ── helpers: "a/b/c" → table a/b, key c ─────────────────────────────────
    private NetworkTable topic(String path) { return sub(root, path); }

    private static NetworkTable sub(NetworkTable base, String path) {
        int i = path.lastIndexOf('/');
        return i < 0 ? base : base.getSubTable(path.substring(0, i));
    }

    private static String leaf(String path) {
        int i = path.lastIndexOf('/');
        return i < 0 ? path : path.substring(i + 1);
    }
}
