package frc.robot.sim;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * ★ NEW (sim only) — the FUEL lying on the field, as 2D physics (top-down).
 *
 * Each ball is a disc (5.91 in) that rolls, slows down on the carpet, bumps
 * into other balls, the guardrails, the HUBs, TOWERs and OUTPOSTs, and is
 * pushed by the robot's bumpers. BUMPs and TRENCHes don't stop FUEL (it
 * rolls over / under them). The robot is moved by the TRUE pose and is not
 * slowed down by the balls (robot collisions are phase 5).
 *
 * A ball that touches the intake zone in front of the bumper while the
 * intake runs is picked up (the caller says how many it may take this step).
 *
 * Also holds FUEL that is not on the floor: inside a HUB (comes back out of
 * one of its 4 exits into the NEUTRAL ZONE) and in each OUTPOST CHUTE (rolls
 * out onto the field when the human player releases it).
 *
 * Pure Java (no NetworkTables / HAL), so it is unit-tested directly.
 */
public class FieldFuel {
    public static final double R = SimRobotConstants.kFuelRadiusM;

    // ── Physics (foam ball on carpet — educated guesses) ────────────────────
    /** Bounce: 0 = dead stop, 1 = perfectly elastic. High-density foam is soft. */
    static final double kRestitution = 0.3;
    /** Rolling resistance on carpet, m/s². A ball pushed at 2 m/s rolls ~1.7 m. */
    static final double kRollingDecel = 1.2;
    static final double kSubstepS = 0.005;
    /** Seconds a scored ball spends inside the HUB before it comes out. */
    static final double kHubDelayS = 1.0;
    /** One CHUTE ball every this many seconds once released. */
    static final double kChuteIntervalS = 0.15;
    /** A robot "move" longer than this in one step is a teleport, not driving. */
    static final double kTeleportM = 0.5;

    private static final int MAX = 1024;
    private static final double CELL = 2 * R;

    final double[] x = new double[MAX], y = new double[MAX], vx = new double[MAX], vy = new double[MAX];
    int n = 0;

    private final double length, width;
    private final List<double[]> solids;
    private final double[] hubBlue, hubRed, outpostBlue, outpostRed, depotBlue, depotRed;
    private final int gx, gy;
    private final int[] head, next;

    /** Scored FUEL inside a HUB: {release time, red ? 1 : 0}. */
    private final ArrayDeque<double[]> inHub = new ArrayDeque<>();
    private final int[] chute = new int[2];          // 0 = blue, 1 = red
    private final boolean[] chuteOpen = new boolean[2];
    private final double[] chuteNext = new double[2];
    private double time = 0;
    private Random rng = new Random(9520);

    /** The robot during one step: true pose at the start and end, bumper size, intake. */
    public record RobotBody(double x0, double y0, double th0, double x1, double y1, double th1,
                            double lengthM, double widthM, double intakeDepthM, double intakeWidthM) {}

    public FieldFuel(FieldModel field) {
        length = field.length;
        width = field.width;
        solids = field.fuelSolids;
        hubBlue = boxOr(field.box("blue-hub"), 4.0146, 3.4246, 5.2084, 4.6184);
        hubRed = boxOr(field.box("red-hub"), 11.3096, 3.4246, 12.5034, 4.6184);
        outpostBlue = boxOr(field.box("blue-outpost"), 0, 0.4123, 0.15, 1.3217);
        outpostRed = boxOr(field.box("red-outpost"), length - 0.15, width - 1.3217, length, width - 0.4123);
        depotBlue = boxOr(field.box("blue-depot"), 0, 5.4266, 0.6858, 6.4934);
        depotRed = boxOr(field.box("red-depot"), length - 0.6858, width - 6.4934, length, width - 5.4266);
        gx = (int) Math.ceil(length / CELL) + 1;
        gy = (int) Math.ceil(width / CELL) + 1;
        head = new int[gx * gy];
        next = new int[MAX];
    }

    private static double[] boxOr(double[] b, double x0, double y0, double x1, double y1) {
        return b != null ? b : new double[] { x0, y0, x1, y1 };
    }

    // ── Staging (start of a match) ───────────────────────────────────────────

    /**
     * Put FUEL where the field crew stages it: `neutral` in the NEUTRAL ZONE
     * box (206 in × 72 in, centered on the CENTER LINE, split by a 2 in
     * divider), `perDepot` in each DEPOT, `perChute` in each CHUTE.
     */
    public void restage(long seed, int neutral, int perDepot, int perChute) {
        rng = new Random(seed);
        n = 0;
        inHub.clear();
        chute[0] = chute[1] = perChute;
        chuteOpen[0] = chuteOpen[1] = false;

        // NEUTRAL ZONE box (GM 6.3.4.1). If the balls don't fit in one flat
        // layer, the box grows a row at a time (the real crew piles them).
        double halfDepth = 72.0 * 0.0254 / 2, halfWidth = 206.0 * 0.0254 / 2, gap = 2.0 * 0.0254;
        List<double[]> spots;
        do {
            spots = new ArrayList<>();
            hexFill(length / 2 + gap / 2, width / 2 - halfWidth, length / 2 + halfDepth, width / 2 + halfWidth, spots);
            hexFill(length / 2 - halfDepth, width / 2 - halfWidth, length / 2 - gap / 2, width / 2 + halfWidth, spots);
            halfDepth += 2 * R * 0.866;
        } while (spots.size() < neutral);
        place(spots, neutral);

        for (double[] d : new double[][] { depotBlue, depotRed }) {
            List<double[]> ds = new ArrayList<>();
            hexFill(d[0], d[1], d[2], d[3], ds);
            place(ds, Math.min(perDepot, ds.size()));
        }
    }

    /** Hex-packed ball centers inside a box (1 mm between neighbours). */
    private void hexFill(double x0, double y0, double x1, double y1, List<double[]> out) {
        double s = 2 * R + 0.001, row = s * Math.sqrt(3) / 2;
        int k = 0;
        for (double cx = x0 + R; cx <= x1 - R + 1e-9; cx += row, k++) {
            for (double cy = y0 + R + (k % 2) * s / 2; cy <= y1 - R + 1e-9; cy += s) out.add(new double[] { cx, cy });
        }
    }

    /** Use `count` of the spots, picked at random (so it isn't a perfect grid). */
    private void place(List<double[]> spots, int count) {
        Collections.shuffle(spots, rng);
        for (int i = 0; i < count && i < spots.size(); i++) add(spots.get(i)[0], spots.get(i)[1], 0, 0);
    }

    // ── FUEL entering / leaving the floor ────────────────────────────────────

    /** A ball lands on the field (a missed shot, a CHUTE ball). */
    public void add(double px, double py, double pvx, double pvy) {
        if (n >= MAX) return;
        x[n] = Math.max(R, Math.min(length - R, px));
        y[n] = Math.max(R, Math.min(width - R, py));
        vx[n] = pvx;
        vy[n] = pvy;
        n++;
    }

    /** A scored ball goes into a HUB; it comes back out into the NEUTRAL ZONE after kHubDelayS. */
    public void toHub(boolean red) { inHub.addLast(new double[] { time + kHubDelayS, red ? 1 : 0 }); }

    /** The human player opens the CHUTE door: its FUEL rolls out onto the field, one ball at a time. */
    public void releaseChute(boolean red) {
        int a = red ? 1 : 0;
        if (chute[a] > 0 && !chuteOpen[a]) { chuteOpen[a] = true; chuteNext[a] = time; }
    }

    private void remove(int i) {
        n--;
        x[i] = x[n]; y[i] = y[n]; vx[i] = vx[n]; vy[i] = vy[n];
    }

    // ── Physics ──────────────────────────────────────────────────────────────

    /**
     * Advance by dt. `robot` may be null. Returns how many balls the intake
     * picked up (never more than maxPickups; 0 = intake off or robot full,
     * then the bumper just pushes the balls).
     */
    public int step(double dt, RobotBody robot, int maxPickups) {
        if (dt <= 0) return 0;
        time += dt;
        releaseHubAndChutes();

        // Robot velocity over this step (zero across a teleport: it didn't really drive there)
        double rvx = 0, rvy = 0, w = 0;
        boolean teleport = false;
        if (robot != null) {
            teleport = Math.hypot(robot.x1() - robot.x0(), robot.y1() - robot.y0()) > kTeleportM;
            if (!teleport) {
                rvx = (robot.x1() - robot.x0()) / dt;
                rvy = (robot.y1() - robot.y0()) / dt;
                w = Math.IEEEremainder(robot.th1() - robot.th0(), 2 * Math.PI) / dt;
            }
        }
        int steps = Math.max(1, (int) Math.ceil(dt / kSubstepS));
        double h = dt / steps;
        int picked = 0;
        for (int s = 1; s <= steps; s++) {
            integrate(h);
            for (int it = 0; it < 2; it++) collideBalls();
            if (robot != null) picked += collideRobot(robot, teleport ? 1.0 : (double) s / steps, rvx, rvy, w, maxPickups - picked);
            for (int i = 0; i < n; i++) collideStatic(i);
        }
        return picked;
    }

    private void releaseHubAndChutes() {
        while (!inHub.isEmpty() && inHub.peekFirst()[0] <= time) {
            boolean red = inHub.removeFirst()[1] > 0.5;
            // GM 5.4: 4 exits at the base of the HUB, facing the NEUTRAL ZONE (positions approximate)
            double[] hub = red ? hubRed : hubBlue;
            double cy = (hub[1] + hub[3]) / 2, ex = red ? hub[0] - R - 0.01 : hub[2] + R + 0.01;
            double ey = cy + (rng.nextInt(4) - 1.5) * 0.30;
            double out = 1.2 + rng.nextDouble() * 0.8;
            add(ex, ey, red ? -out : out, (rng.nextDouble() - 0.5) * 0.8);
        }
        for (int a = 0; a < 2; a++) {
            while (chuteOpen[a] && chute[a] > 0 && chuteNext[a] <= time) {
                // GM 5.9.2: FUEL leaves the OUTPOST through a 31.8 in wide opening, rolling into the field
                double[] o = a == 1 ? outpostRed : outpostBlue;
                double cy = (o[1] + o[3]) / 2, ex = a == 1 ? o[0] - R - 0.02 : o[2] + R + 0.02;
                double out = 1.5 + rng.nextDouble() * 0.6;
                add(ex, cy + (rng.nextDouble() - 0.5) * 0.6, a == 1 ? -out : out, (rng.nextDouble() - 0.5) * 0.6);
                chute[a]--;
                chuteNext[a] += kChuteIntervalS;
            }
            if (chute[a] == 0) chuteOpen[a] = false;
        }
    }

    private void integrate(double h) {
        for (int i = 0; i < n; i++) {
            double v = Math.hypot(vx[i], vy[i]);
            if (v < 1e-4) { vx[i] = vy[i] = 0; continue; }
            double k = Math.max(0, v - kRollingDecel * h) / v;
            vx[i] *= k;
            vy[i] *= k;
            x[i] += vx[i] * h;
            y[i] += vy[i] * h;
        }
    }

    /** Ball–ball contacts, found with a uniform grid (each ball only checks its 3×3 neighbour cells). */
    private void collideBalls() {
        java.util.Arrays.fill(head, -1);
        for (int i = 0; i < n; i++) {
            int c = cell(x[i], y[i]);
            next[i] = head[c];
            head[c] = i;
        }
        double d2min = 4 * R * R;
        for (int i = 0; i < n; i++) {
            int cx = cellX(x[i]), cy = cellY(y[i]);
            for (int ox = -1; ox <= 1; ox++) for (int oy = -1; oy <= 1; oy++) {
                int qx = cx + ox, qy = cy + oy;
                if (qx < 0 || qy < 0 || qx >= gx || qy >= gy) continue;
                for (int j = head[qx * gy + qy]; j >= 0; j = next[j]) {
                    if (j <= i) continue;
                    double dx = x[j] - x[i], dy = y[j] - y[i], d2 = dx * dx + dy * dy;
                    if (d2 >= d2min) continue;
                    double d = Math.sqrt(d2), nx, ny;
                    if (d < 1e-9) { nx = 1; ny = 0; d = 0; } else { nx = dx / d; ny = dy / d; }
                    double push = (2 * R - d) / 2;
                    x[i] -= nx * push; y[i] -= ny * push;
                    x[j] += nx * push; y[j] += ny * push;
                    double rel = (vx[j] - vx[i]) * nx + (vy[j] - vy[i]) * ny;
                    if (rel < 0) {
                        double imp = -(1 + kRestitution) * rel / 2; // equal masses
                        vx[i] -= imp * nx; vy[i] -= imp * ny;
                        vx[j] += imp * nx; vy[j] += imp * ny;
                    }
                }
            }
        }
    }

    private int cellX(double px) { return Math.max(0, Math.min(gx - 1, (int) (px / CELL))); }
    private int cellY(double py) { return Math.max(0, Math.min(gy - 1, (int) (py / CELL))); }
    private int cell(double px, double py) { return cellX(px) * gy + cellY(py); }

    /** Guardrails / alliance walls and the solid field elements. */
    private void collideStatic(int i) {
        if (x[i] < R)          { x[i] = R;          if (vx[i] < 0) vx[i] = -vx[i] * kRestitution; }
        if (x[i] > length - R) { x[i] = length - R; if (vx[i] > 0) vx[i] = -vx[i] * kRestitution; }
        if (y[i] < R)          { y[i] = R;          if (vy[i] < 0) vy[i] = -vy[i] * kRestitution; }
        if (y[i] > width - R)  { y[i] = width - R;  if (vy[i] > 0) vy[i] = -vy[i] * kRestitution; }
        for (double[] b : solids) {
            double qx = Math.max(b[0], Math.min(b[2], x[i])), qy = Math.max(b[1], Math.min(b[3], y[i]));
            double dx = x[i] - qx, dy = y[i] - qy, d2 = dx * dx + dy * dy;
            if (d2 >= R * R) continue;
            double nx, ny, pen;
            if (d2 > 1e-12) {               // center outside the box, edge inside the ball
                double d = Math.sqrt(d2);
                nx = dx / d; ny = dy / d; pen = R - d;
            } else {                        // center inside the box: leave by the nearest face
                double l = x[i] - b[0], r = b[2] - x[i], dn = y[i] - b[1], up = b[3] - y[i];
                double m = Math.min(Math.min(l, r), Math.min(dn, up));
                if (m == l) { nx = -1; ny = 0; } else if (m == r) { nx = 1; ny = 0; } else if (m == dn) { nx = 0; ny = -1; } else { nx = 0; ny = 1; }
                pen = m + R;
            }
            x[i] += nx * pen;
            y[i] += ny * pen;
            double vn = vx[i] * nx + vy[i] * ny;
            if (vn < 0) { vx[i] -= (1 + kRestitution) * vn * nx; vy[i] -= (1 + kRestitution) * vn * ny; }
        }
    }

    /**
     * The robot at fraction f of this step: the intake takes balls touching
     * its zone (up to `canTake`), and the bumpers push every other ball out.
     */
    private int collideRobot(RobotBody rb, double f, double rvx, double rvy, double w, int canTake) {
        double dth = Math.IEEEremainder(rb.th1() - rb.th0(), 2 * Math.PI);
        double rx = rb.x0() + (rb.x1() - rb.x0()) * f, ry = rb.y0() + (rb.y1() - rb.y0()) * f, th = rb.th0() + dth * f;
        double c = Math.cos(th), s = Math.sin(th);
        double hl = rb.lengthM() / 2, hw = rb.widthM() / 2, iw = rb.intakeWidthM() / 2;
        double reach = Math.hypot(hl, hw) + rb.intakeDepthM() + R;
        int picked = 0;
        for (int i = 0; i < n; i++) {
            double dx = x[i] - rx, dy = y[i] - ry;
            if (Math.abs(dx) > reach || Math.abs(dy) > reach) continue;
            double lx = dx * c + dy * s, ly = -dx * s + dy * c;   // robot frame: +x forward, +y left
            // Intake: a ball touching the zone in front of the bumper
            if (picked < canTake && Math.abs(ly) <= iw && lx >= hl - R && lx <= hl + rb.intakeDepthM() + R) {
                remove(i--);
                picked++;
                continue;
            }
            // Bumpers: push the ball out of the rectangle
            double qx = Math.max(-hl, Math.min(hl, lx)), qy = Math.max(-hw, Math.min(hw, ly));
            double ex = lx - qx, ey = ly - qy, d2 = ex * ex + ey * ey;
            if (d2 >= R * R) continue;
            double nlx, nly, pen;
            if (d2 > 1e-12) {
                double d = Math.sqrt(d2);
                nlx = ex / d; nly = ey / d; pen = R - d;
            } else {
                double front = hl - lx, back = lx + hl, left = hw - ly, right = ly + hw;
                double m = Math.min(Math.min(front, back), Math.min(left, right));
                if (m == front) { nlx = 1; nly = 0; } else if (m == back) { nlx = -1; nly = 0; } else if (m == left) { nlx = 0; nly = 1; } else { nlx = 0; nly = -1; }
                pen = m + R;
            }
            double nx = nlx * c - nly * s, ny = nlx * s + nly * c;    // back to the field frame
            x[i] += nx * pen;
            y[i] += ny * pen;
            // Bumper surface velocity at the contact: v + ω × r
            double px = rvx - w * (y[i] - ry), py = rvy + w * (x[i] - rx);
            double rel = (vx[i] - px) * nx + (vy[i] - py) * ny;
            if (rel < 0) { vx[i] -= (1 + kRestitution) * rel * nx; vy[i] -= (1 + kRestitution) * rel * ny; }
        }
        return picked;
    }

    // ── Read-only views ──────────────────────────────────────────────────────

    public int onField() { return n; }
    public int inHub() { return inHub.size(); }
    public int inChute(boolean red) { return chute[red ? 1 : 0]; }
    /** Everything this class holds (floor + HUBs + CHUTEs). Plus the robot's FUEL, this is the match total. */
    public int total() { return n + inHub.size() + chute[0] + chute[1]; }

    /** {x, y} of every ball on the floor, flat. */
    public double[] positions() {
        double[] out = new double[2 * n];
        for (int i = 0; i < n; i++) { out[2 * i] = x[i]; out[2 * i + 1] = y[i]; }
        return out;
    }
}
