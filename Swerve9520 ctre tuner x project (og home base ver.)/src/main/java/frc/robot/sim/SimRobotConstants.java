package frc.robot.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * Physical robot numbers the simulator needs that are NOT in the robot code.
 *
 * Every value marked TO MEASURE is a placeholder guess. Measure the real
 * robot and replace it here (and in pathplanner/settings.json, which the
 * Plan tab's Robot settings panel edits).
 *
 * Rules referenced (2026 Game Manual, section 8):
 *   R104 ROBOT PERIMETER <= 110.0 in, height <= 30 in
 *   R402 BUMPER foam >= 2.25 in, R403 BUMPERS <= 4.0 in from the perimeter
 */
public final class SimRobotConstants {
    private SimRobotConstants() {}

    // ── Frame & bumpers ─────────────────────────────────────────────────────
    // Wheelbase is known: TunerConstants module positions = ±11.5 in (X), ±10 in (Y).
    /** Frame length front-to-back, without bumpers. TO MEASURE */
    public static final double kFrameLengthM   = Inches.of(27.0).in(Meters);
    /** Frame width side-to-side, without bumpers. TO MEASURE */
    public static final double kFrameWidthM    = Inches.of(24.0).in(Meters);
    /** Bumper thickness (foam + backing + fabric). TO MEASURE */
    public static final double kBumperThicknessM = Inches.of(3.25).in(Meters);

    public static final double kLengthWithBumpersM = kFrameLengthM + 2 * kBumperThicknessM;
    public static final double kWidthWithBumpersM  = kFrameWidthM  + 2 * kBumperThicknessM;

    // ── Mass properties (also in pathplanner/settings.json) ────────────────
    /** Robot mass with battery and bumpers. TO MEASURE */
    public static final double kMassKg = 52.0;
    /** Rotational moment of inertia about the center. TO MEASURE (or estimate from CAD) */
    public static final double kMoiKgM2 = 5.5;
    /** Wheel coefficient of friction on carpet. TO MEASURE */
    public static final double kWheelCof = 1.2;

    // ── Cameras (robot frame: +X forward, +Y left, +Z up; angles in degrees) ─
    // LL3 mount is configured in the Limelight web UI, not in robot code. TO MEASURE
    public static final double[] kLL3Mount = { 0.30, 0.00, 0.25, /*roll*/ 0, /*pitch up*/ 20, /*yaw*/ 0 };
    // LL4 height 0.55 m and pitch 25° are from VisionSubsystem; X/Y are TO MEASURE.
    public static final double[] kLL4Mount = { 0.20, 0.00, 0.55, 0, 25, 0 };

    // ── Shooter (used by the flywheel sim in phase 4) ───────────────────────
    /** Flywheel motor:wheel gearing. TO MEASURE */
    public static final double kShooterGearing = 1.0;
    /** Flywheel moment of inertia. TO MEASURE */
    public static final double kShooterMoiKgM2 = 0.004;
    /** Direction the shooter fires, relative to the robot's front (0 = forward). TO MEASURE */
    public static final double kShooterYawDeg = 0.0;
    /** Preloaded FUEL (game manual allows up to 8). */
    public static final int    kFuelPreload = 8;

    // ── Default start pose for the sim (blue alliance zone, facing red) ────
    public static final double[] kDefaultStartPose = { 3.40, 5.50, 0.0 };
}
