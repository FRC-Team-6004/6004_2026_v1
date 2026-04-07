package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.controls.RainbowAnimation;


import java.util.EnumMap;

public class ShooterReal implements ShooterIO {

    private static class ShooterUnit {
        final TalonFX motor;
        final TalonFX followerMotor;
        final VelocityTorqueCurrentFOC VTC = new VelocityTorqueCurrentFOC(0)
            .withAcceleration(10)
            .withFeedForward(2)
            .withSlot(0);

        
        final Servo hood;
        final Servo hood2;

        double oldSP = 0.0;
        double oldTRPM = 0.0;
        double servoPercent = 0.0;
        double targetRPM = 0.0;

        ShooterUnit(int motorID, int followerID, int servoChannel, int servoTwo) {

            var motorConfig = new TalonFXConfiguration();

            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfig.CurrentLimits.SupplyCurrentLimit = 70;
            motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfig.CurrentLimits.StatorCurrentLimit = 120;
            motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            motorConfig.Slot0.kP = 10;
            motorConfig.Audio.AllowMusicDurDisable = true;

            motor = new TalonFX(motorID);

            followerMotor = new TalonFX(followerID);

            motor.getConfigurator().apply(motorConfig);
            followerMotor.getConfigurator().apply(motorConfig);

            followerMotor.setControl(new Follower(motorID, MotorAlignmentValue.Opposed));
            
            hood = new Servo(servoChannel);
            hood2 = new Servo(servoTwo);
            

        }
    }

    private final EnumMap<ShooterSide, ShooterUnit> shooters =
        new EnumMap<>(ShooterSide.class);

    public ShooterReal() {
        shooters.put(
            ShooterSide.MAIN,
            new ShooterUnit(ShooterConstants.kLeftMotorID, ShooterConstants.kRightMotorID, ShooterConstants.kLeftServoPort, ShooterConstants.kRightServoPort)
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
    }

    public void periodic() {
        for (ShooterUnit unit : shooters.values()) {

            if (unit.oldSP != unit.servoPercent) {
                unit.oldSP = unit.servoPercent;
                int servo = (int) (unit.servoPercent * ShooterConstants.servoRange) + ShooterConstants.servoIn;
                servo = Math.max(Math.min(servo, ShooterConstants.servoOut), ShooterConstants.servoIn);
                unit.hood.setPulseTimeMicroseconds(servo);
                unit.hood2.setPulseTimeMicroseconds(servo);
            }

            if (unit.oldTRPM != unit.targetRPM) {
                unit.oldTRPM = unit.targetRPM;
                if (unit.targetRPM == 0.0) {
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
