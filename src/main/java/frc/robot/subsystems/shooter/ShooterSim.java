package frc.robot.subsystems.shooter;

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
        double oldTRPM = 0.0;

        double servoPercent = 0.0;
        double oldSP = 0.0;

        FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                1.0,
                ShooterConstants.kMOI
            ),
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
        shooters.put(ShooterSide.MAIN, new ShooterSimUnit());
    }

    @Override
    public void setTargetRPM(ShooterSide side, double rpm) {
        shooters.get(side).targetRPM = rpm;
    }

    @Override
    public double getRPM(ShooterSide side) {
        return Units.radiansPerSecondToRotationsPerMinute(
            shooters.get(side).sim.getAngularVelocityRadPerSec()
        );
    }

    @Override
    public void setServoAngle(ShooterSide side, double percent) {
        shooters.get(side).servoPercent = percent;
    }

    @Override
    public void stop(ShooterSide side) {
        ShooterSimUnit unit = shooters.get(side);
        unit.targetRPM = 0.0;
        unit.sim.setInputVoltage(0.0);
    }

    /** Call from Shooter subsystem periodic() */
    public void periodic() {
        for (ShooterSimUnit unit : shooters.values()) {

            // Mirror servo "update only on change"
            if (unit.oldSP != unit.servoPercent) {
                unit.oldSP = unit.servoPercent;
                // No physical servo in sim, just store value
            }

            // Mirror real "only update motor when target changes"
            if (unit.oldTRPM != unit.targetRPM) {
                unit.oldTRPM = unit.targetRPM;

                if (unit.targetRPM <= 0.0) {
                    unit.sim.setInputVoltage(0.0);
                } else {
                    double targetRadPerSec =
                        Units.rotationsPerMinuteToRadiansPerSecond(unit.targetRPM);

                    double ffVolts = feedforward.calculate(targetRadPerSec);

                    unit.sim.setInputVoltage(
                        Math.max(-12.0, Math.min(12.0, ffVolts))
                    );
                }
            }

            unit.sim.update(0.02);
        }
    }
}