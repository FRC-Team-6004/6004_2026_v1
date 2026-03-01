package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

import java.util.EnumMap;

public class ShooterSim implements ShooterIO {

    private static class ShooterSimUnit {
        double targetRPM = 0.0;
        BangBangController bangBang = new BangBangController();

        FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                1.0,
                ShooterConstants.kMOI),
            DCMotor.getKrakenX60Foc(1)
        );
    }

    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.kS,
            ShooterConstants.kV,
            ShooterConstants.kA
        );

    private final EnumMap<ShooterSide, ShooterSimUnit> shooters =
        new EnumMap<>(ShooterSide.class);

    public ShooterSim() {
        shooters.put(ShooterSide.LEFT, new ShooterSimUnit());
        shooters.put(ShooterSide.RIGHT, new ShooterSimUnit());
    }

    @Override
    public void setTargetRPM(ShooterSide side, double rpm) {
        shooters.get(side).targetRPM = rpm;
    }

    @Override
    public double getRPM(ShooterSide side) {
        return Units.radiansPerSecondToRotationsPerMinute(
            shooters.get(side).sim.getAngularVelocityRadPerSec());
    }

    @Override
    public void stop(ShooterSide side) {
        shooters.get(side).targetRPM = 0.0;
        shooters.get(side).sim.setInputVoltage(0.0);
    }

    @Override
    public void setServoAngle(ShooterSide side, double percent) {} //later

    public void periodic() {
        for (ShooterSimUnit unit : shooters.values()) {
            double currentRPM =
                Units.radiansPerSecondToRotationsPerMinute(
                    unit.sim.getAngularVelocityRadPerSec());

            if (unit.targetRPM <= 0.0) {
                unit.sim.setInputVoltage(0.0);
            } else {
                double bangVolts =
                    unit.bangBang.calculate(currentRPM, unit.targetRPM) * 12.0;

                double ffVolts =
                    feedforward.calculate(unit.targetRPM);

                unit.sim.setInputVoltage(
                    Math.max(-12.0, Math.min(12.0, bangVolts + 0.9 * ffVolts))
                );
            }

            unit.sim.update(0.02);
        }
    }
}
