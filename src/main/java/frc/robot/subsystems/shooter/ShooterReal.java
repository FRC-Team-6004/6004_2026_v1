package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ShooterConstants;


import java.util.EnumMap;

public class ShooterReal implements ShooterIO {

    private static class ShooterUnit {
        final TalonFX motor;
        final VelocityTorqueCurrentFOC VTC = new VelocityTorqueCurrentFOC(0)
            .withAcceleration(10)
            .withFeedForward(2)
            .withSlot(0);
        final BangBangController bangBang = new BangBangController();
        final Servo hood;
        double oldSP = 0.0;
        double oldTRPM = 0.0;
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

            if (unit.oldSP != unit.servoPercent) {
                unit.hood.setPulseTimeMicroseconds((int) (unit.servoPercent * ShooterConstants.servoRange) + ShooterConstants.servoIn);
            }

            if (unit.oldTRPM != unit.targetRPM) {
                unit.oldTRPM = unit.targetRPM;
                if (unit.targetRPM <= 0.0) {
                    unit.motor.stopMotor();
                    continue;
                } 

                unit.motor.setControl(
                    unit.VTC.withVelocity(unit.targetRPM / 60)
                );
            }


        }
    }
}
