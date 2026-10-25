package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;

/**
 * ★ NEW (sim only) — physics for the robot's non-drive motors, connected to
 * the simulated TalonFX devices (the robot code's own control requests —
 * VelocityVoltage, DutyCycleOut — run on the simulated controllers; this
 * class turns their output voltage into motion and feeds it back).
 *
 *   Shooter flywheel (CAN 13) and feeder (14): flywheels.
 *   Intake roller (15): flywheel.
 *   Intake arm (19) and climbers (16, 17): position mechanisms with soft limits
 *   (the real robot has none — this shows where they would end).
 *
 * Gearings, inertias and travel are placeholders — TO MEASURE
 * (SimRobotConstants holds the shooter ones).
 */
public class MechanismSims {
    // CAN IDs — must match RobotContainer / Shooter
    private static final int kIntakeId = 15, kArmId = 19, kClimbLeftId = 16, kClimbRightId = 17;

    final Wheel shooter = new Wheel(new TalonFX(Shooter.kShooterMotorId, TunerConstants.kCANBus),
        SimRobotConstants.kShooterMoiKgM2, SimRobotConstants.kShooterGearing);
    final Wheel feeder  = new Wheel(new TalonFX(Shooter.kFeedMotorId, TunerConstants.kCANBus), 0.0008, 1.0);
    final Wheel intake  = new Wheel(new TalonFX(kIntakeId), 0.0008, 1.0);
    final Arm   arm        = new Arm(new TalonFX(kArmId), 60.0, 0.05, 0.0, 0.30);       // output rotations
    final Arm   climbLeft  = new Arm(new TalonFX(kClimbLeftId), 25.0, 0.01, 0.0, 8.0);
    final Arm   climbRight = new Arm(new TalonFX(kClimbRightId), 25.0, 0.01, 0.0, 8.0);

    public void update(double dt) {
        if (dt <= 0) return;
        double vbat = RobotController.getBatteryVoltage();
        shooter.update(dt, vbat);
        feeder.update(dt, vbat);
        intake.update(dt, vbat);
        arm.update(dt, vbat);
        climbLeft.update(dt, vbat);
        climbRight.update(dt, vbat);
    }

    /** A ball leaving takes energy out of the flywheel (≈ 8 % speed dip). */
    public void onShot() {
        shooter.sim.setAngularVelocity(shooter.sim.getAngularVelocityRadPerSec() * 0.92);
    }

    /** Spinning mechanism (flywheel / roller). Velocity reported in mechanism rotations/s. */
    static final class Wheel {
        final TalonFXSimState state;
        final FlywheelSim sim;
        final double gearing;

        Wheel(TalonFX motor, double moi, double gearing) {
            this.state = motor.getSimState();
            this.gearing = gearing;
            DCMotor kraken = DCMotor.getKrakenX60(1);
            this.sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(kraken, moi, gearing), kraken);
        }

        void update(double dt, double vbat) {
            state.setSupplyVoltage(vbat);
            sim.setInputVoltage(state.getMotorVoltage());
            sim.update(dt);
            double mechRps = sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
            state.setRotorVelocity(mechRps * gearing);
            state.addRotorPosition(mechRps * gearing * dt);
        }

        /** Mechanism (wheel) rotations per second. */
        double rps() { return sim.getAngularVelocityRadPerSec() / (2 * Math.PI); }

        /** Motor (rotor) rotations per second — the units the TalonFX reports. */
        double rotorRps() { return rps() * gearing; }
    }

    /** Positional mechanism with hard stops at [min, max] output rotations. */
    static final class Arm {
        final TalonFXSimState state;
        final DCMotorSim sim;
        final double gearing, min, max;

        Arm(TalonFX motor, double gearing, double moi, double min, double max) {
            this.state = motor.getSimState();
            this.gearing = gearing;
            this.min = min;
            this.max = max;
            DCMotor kraken = DCMotor.getKrakenX60(1);
            this.sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(kraken, moi, gearing), kraken);
        }

        void update(double dt, double vbat) {
            state.setSupplyVoltage(vbat);
            sim.setInputVoltage(state.getMotorVoltage());
            sim.update(dt);
            double pos = sim.getAngularPositionRotations();
            if (pos < min || pos > max) { // hit the end of travel
                sim.setState(Math.max(min, Math.min(max, pos)) * 2 * Math.PI, 0.0);
            }
            state.setRawRotorPosition(sim.getAngularPositionRotations() * gearing);
            state.setRotorVelocity(sim.getAngularVelocityRadPerSec() / (2 * Math.PI) * gearing);
        }

        /** 0 = at min, 1 = at max. */
        double fraction() { return (sim.getAngularPositionRotations() - min) / (max - min); }
    }
}
