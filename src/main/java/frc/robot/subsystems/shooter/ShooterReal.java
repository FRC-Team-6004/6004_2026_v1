package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

import java.util.EnumMap;

public class ShooterReal implements ShooterIO {

    private static class ShooterUnit {
        final TalonFX motor;
        final BangBangController bangBang = new BangBangController();
        final Servo hood;
        double servoPercent = 0.0;
        double targetRPM = 0.0;

        ShooterUnit(int motorID, int servoChannel) {
            motor = new TalonFX(motorID);
            hood = new Servo(servoChannel);
        }
    }

    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.kS,
            ShooterConstants.kV,
            ShooterConstants.kA
        );

    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final EnumMap<ShooterSide, ShooterUnit> shooters =
        new EnumMap<>(ShooterSide.class);

    public ShooterReal() {
        shooters.put(
            ShooterSide.LEFT,
            new ShooterUnit(ShooterConstants.kLeftMotorID, ShooterConstants.kLeftServoPort)
        );

        shooters.put(
            ShooterSide.RIGHT,
            new ShooterUnit(ShooterConstants.kRightMotorID, ShooterConstants.kRightServoPort)
        );
    }

    @Override
    public void setTargetRPM(ShooterSide side, double rpm) {
        shooters.get(side).targetRPM = rpm;
    }

    @Override
    public double getRPM(ShooterSide side) {
        return shooters.get(side).motor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void setServoAngle(ShooterSide side, double percent) {
        shooters.get(side).servoPercent = percent;
    }

    @Override
    public void stop(ShooterSide side) {
        ShooterUnit unit = shooters.get(side);
        unit.targetRPM = 0.0;
        //unit.motor.stopMotor();
    }

    /** Call from Shooter subsystem periodic() */
    public void periodic() {
        for (ShooterUnit unit : shooters.values()) {
            if (unit.targetRPM <= 0.0) {
                unit.motor.stopMotor();
                continue;
            }

            double currentRPM =
                unit.motor.getVelocity().getValueAsDouble() * 60.0;

            double bangVolts =
                unit.bangBang.calculate(currentRPM, unit.targetRPM) * 12.0;

            double ffVolts =
                feedforward.calculate(unit.targetRPM);

            unit.motor.setControl(
                voltageRequest.withOutput(
                    Math.max(-12.0, Math.min(12.0, bangVolts + 0.9 * ffVolts))
                )
            );

            unit.hood.setPosition(unit.servoPercent);
        }
    }
}
