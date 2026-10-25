package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim implements ShooterIO {

    // Sim-only physics. The real robot closes the loop with TalonFX VelocityVoltage
    // (Slot0 kP), so the flywheel reaches target regardless of feedforward. The sim has
    // no such controller, so it needs a model that actually settles at the target: a
    // Kraken X60's free speed is ~6000 RPM at 12 V, giving a feedforward of
    // 12 V * targetRPM / 6000, plus a small proportional term to mop up error.
    // Do NOT use ShooterConstants.kV (0.39) here: as a SimpleMotorFeedforward in
    // rad/s it demands 100+ V and saturates at 12 V, so the wheel runs away to free
    // speed and the 200 RPM at-speed gate never opens in sim. Likewise kMOI = 12 is
    // not a physical flywheel inertia (a real shooter wheel is ~0.001-0.01 kg m^2);
    // with 12 the wheel would take minutes to spin up.
    private static final double FREE_SPEED_RPM = 6000.0;
    private static final double SIM_MOI_KG_M2 = 0.002;
    private static final double KP_VOLTS_PER_RAD_PER_SEC = 0.05;

    private double targetRPM = 0.0;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(1),
            1.0,
            SIM_MOI_KG_M2
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    public ShooterSim() {
    }

    @Override
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    @Override
    public double getRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(
            sim.getAngularVelocityRadPerSec()
        );
    }

    @Override
    public void stop() {
        this.targetRPM = 0.0;
    }

    public void periodic() {
        // Recomputed every loop (the old code only set voltage when the target
        // changed, so there was no feedback at all). With target 0 this actively
        // brakes the wheel, which is what stop() wants.
        double targetRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);
        double ffVolts = 12.0 * targetRPM / FREE_SPEED_RPM;
        double fbVolts = KP_VOLTS_PER_RAD_PER_SEC
            * (targetRadPerSec - sim.getAngularVelocityRadPerSec());

        sim.setInputVoltage(MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0));
        sim.update(0.02);
    }
}
