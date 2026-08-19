package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

// import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

public class ShooterReal implements ShooterIO {

    private final TalonFX motor;
    private final TalonFX followerMotor;
    private final VelocityTorqueCurrentFOC VTC = new VelocityTorqueCurrentFOC(0)
        .withAcceleration(10)
        .withFeedForward(2)
        .withSlot(0);

    // private final Servo hood;
    // private final Servo hood2;

    // private double oldSP = 0.0;
    private double oldTRPM = 0.0;
    // private double servoPercent = 0.0;
    private double targetRPM = 0.0;

    public ShooterReal() {
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 120;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Slot0.kP = 10;
        motorConfig.Audio.AllowMusicDurDisable = true;

        motor = new TalonFX(ShooterConstants.kLeftMotorID);
        followerMotor = new TalonFX(ShooterConstants.kRightMotorID);

        motor.getConfigurator().apply(motorConfig);
        followerMotor.getConfigurator().apply(motorConfig);

        followerMotor.setControl(new Follower(ShooterConstants.kLeftMotorID, MotorAlignmentValue.Opposed));

        // hood = new Servo(ShooterConstants.kLeftServoPort);
        // hood2 = new Servo(ShooterConstants.kRightServoPort);
    }

    @Override
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    @Override
    public double getRPM() {
        return motor.getVelocity().getValueAsDouble() * 60.0;
    }

    // @Override
    // public void setServoAngle(double percent) {
    //     this.servoPercent = percent;
    // }

    @Override
    public void stop() {
        this.targetRPM = 0.0;
    }

    public void periodic() {
        // if (this.oldSP != this.servoPercent) {
        //     this.oldSP = this.servoPercent;
        //     int servo = (int) (this.servoPercent * ShooterConstants.servoRange) + ShooterConstants.servoIn;
        //     servo = Math.max(Math.min(servo, ShooterConstants.servoOut), ShooterConstants.servoIn);
        //     this.hood.setPulseTimeMicroseconds(servo);
        //     this.hood2.setPulseTimeMicroseconds(servo);
        // }

        if (this.oldTRPM != this.targetRPM) {
            this.oldTRPM = this.targetRPM;
            if (this.targetRPM == 0.0) {
                motor.stopMotor();
                return;
            } 

            motor.setControl(
                VTC.withVelocity(this.targetRPM / 60.0)
            );
        }
    }
}