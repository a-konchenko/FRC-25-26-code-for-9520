package frc.robot.sim;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * ★ NEW (sim only) — publishes what PathPlannerLib is doing while it follows
 * a path, so the UI can draw "plan vs fact":
 *   /Sim/ActivePath      double[] {x, y, θ, x, y, θ, …} — the path being followed
 *                        (already flipped for red), empty when none
 *   /Sim/ActivePathName  string
 *   /Sim/Setpoint        double[3] — where the path follower wants the robot NOW
 *   /Sim/SetpointActive  boolean — false when no path is being followed
 */
public class PathLogging {
    private final SimNT nt;
    private volatile double lastTargetTime = -1;
    private volatile Pose2d lastTarget = Pose2d.kZero;

    public PathLogging(SimNT nt) {
        this.nt = nt;
        PathPlannerLogging.setLogActivePathCallback(this::onActivePath);
        PathPlannerLogging.setLogTargetPoseCallback(this::onTarget);
        nt.array("ActivePath", new double[0]);
        nt.string("ActivePathName", "");
    }

    private void onActivePath(List<Pose2d> poses) {
        double[] flat = new double[poses.size() * 3];
        for (int i = 0; i < poses.size(); i++) {
            Pose2d p = poses.get(i);
            flat[3 * i]     = p.getX();
            flat[3 * i + 1] = p.getY();
            flat[3 * i + 2] = p.getRotation().getRadians();
        }
        nt.array("ActivePath", flat);
        String name = PathPlannerAuto.currentPathName;
        nt.string("ActivePathName", poses.isEmpty() || name == null ? "" : name);
    }

    private void onTarget(Pose2d target) {
        nt.pose("Setpoint", target);
        lastTarget = target;
        lastTargetTime = Timer.getFPGATimestamp();
    }

    /** Call every loop: the setpoint goes stale when no path is running. */
    public void update() {
        nt.bool("SetpointActive", isActive());
    }

    public boolean isActive() { return lastTargetTime >= 0 && Timer.getFPGATimestamp() - lastTargetTime < 0.1; }
    public Pose2d getSetpoint() { return lastTarget; }
}
