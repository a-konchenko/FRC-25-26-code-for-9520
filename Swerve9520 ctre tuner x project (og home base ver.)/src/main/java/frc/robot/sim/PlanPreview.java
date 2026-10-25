package frc.robot.sim;

import java.lang.reflect.Method;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;

/**
 * ★ NEW (sim only) — "exact preview" for the Plan tab.
 *
 * The UI sends a path (the same JSON a .path file holds, even unsaved) to
 * /Sim/Plan/previewRequest as {"id": n, "path": {...}}. This class turns it
 * into a timed trajectory with the SAME PathPlannerLib code the robot runs
 * (PathPlannerPath JSON loader + generateTrajectory with settings.json),
 * and publishes:
 *   /Sim/Plan/previewResult  double[] {id, totalTime, t, x, y, θ, v, t, x, y, θ, v, …}
 *   /Sim/Plan/previewError   string  "id: message" (empty when OK)
 */
public class PlanPreview {
    private static final int kMaxStates = 600;

    private final SimNT nt;
    private final StringSubscriber request = NetworkTableInstance.getDefault()
        .getStringTopic("/Sim/Plan/previewRequest").subscribe("");
    private String lastRequest = "";
    private Method fromJson;

    public PlanPreview(SimNT nt) {
        this.nt = nt;
        try {
            fromJson = PathPlannerPath.class.getDeclaredMethod("fromJson", JSONObject.class);
            fromJson.setAccessible(true);
        } catch (Exception e) {
            System.out.println("[Sim] PlanPreview unavailable: " + e);
        }
    }

    public void update() {
        String req = request.get();
        if (req.isEmpty() || req.equals(lastRequest)) return;
        lastRequest = req;

        double id = -1;
        try {
            JSONObject msg = (JSONObject) new JSONParser().parse(req);
            id = ((Number) msg.get("id")).doubleValue();
            JSONObject pathJson = (JSONObject) msg.get("path");
            if (fromJson == null) throw new IllegalStateException("PathPlanner JSON loader not found");

            PathPlannerPath path = (PathPlannerPath) fromJson.invoke(null, pathJson);
            RobotConfig config = RobotConfig.fromGUISettings(); // re-read: Robot settings may have changed
            Rotation2d startRot = path.getIdealStartingState() != null
                ? path.getIdealStartingState().rotation() : Rotation2d.kZero;
            PathPlannerTrajectory traj = path.generateTrajectory(new ChassisSpeeds(), startRot, config);

            var states = traj.getStates();
            int stride = Math.max(1, (int) Math.ceil(states.size() / (double) kMaxStates));
            int n = (states.size() + stride - 1) / stride;
            double[] out = new double[2 + n * 5 + 5];
            out[0] = id;
            out[1] = traj.getTotalTimeSeconds();
            int k = 2;
            for (int i = 0; i < states.size(); i += stride) k = put(out, k, states.get(i));
            k = put(out, k, states.get(states.size() - 1)); // always include the end
            nt.array("Plan/previewResult", java.util.Arrays.copyOf(out, k));
            nt.string("Plan/previewError", "");
        } catch (Exception e) {
            Throwable cause = e.getCause() != null ? e.getCause() : e;
            nt.string("Plan/previewError", (long) id + ": " + cause.getClass().getSimpleName() + ": " + cause.getMessage());
        }
    }

    private static int put(double[] out, int k, com.pathplanner.lib.trajectory.PathPlannerTrajectoryState s) {
        out[k++] = s.timeSeconds;
        out[k++] = s.pose.getX();
        out[k++] = s.pose.getY();
        out[k++] = s.pose.getRotation().getRadians();
        out[k++] = s.linearVelocity;
        return k;
    }
}
