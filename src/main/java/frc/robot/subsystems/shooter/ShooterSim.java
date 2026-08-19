package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterSim implements ShooterIO {

    private double targetRPM = 0.0;
    private double oldTRPM = 0.0;

    // private double servoPercent = 0.0;
    // private double oldSP = 0.0;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(1),
            1.0,
            ShooterConstants.kMOI
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.kS,
            ShooterConstants.kV,
            ShooterConstants.kA
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

    // @Override
    // public void setServoAngle(double percent) {
    //     this.servoPercent = percent;
    // }

    @Override
    public void stop() {
        this.targetRPM = 0.0;
        sim.setInputVoltage(0.0);
    }

    public void periodic() {
        // if (this.oldSP != this.servoPercent) {
        //     this.oldSP = this.servoPercent;
        // }

        if (this.oldTRPM != this.targetRPM) {
            this.oldTRPM = this.targetRPM;

            if (this.targetRPM <= 0.0) {
                sim.setInputVoltage(0.0);
            } else {
                double targetRadPerSec =
                    Units.rotationsPerMinuteToRadiansPerSecond(this.targetRPM);

                double ffVolts = feedforward.calculate(targetRadPerSec);

                sim.setInputVoltage(
                    Math.max(-12.0, Math.min(12.0, ffVolts))
                );
            }
        }

        sim.update(0.02);
    }
}