package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;;

public class ClimberReal implements ClimberIO {

    private final SparkMax motor;
    
    private final RelativeEncoder encoder;  
    private final PIDController pid;  

    public ClimberReal () {
        motor = new SparkMax(ClimberConstants.kMotorID, MotorType.kBrushless);

        motor.setCANTimeout(250);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.voltageCompensation(10);
        motorConfig.smartCurrentLimit(ClimberConstants.kCurrentLimit);
        motorConfig.idleMode(IdleMode.kBrake);
        // motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        pid = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
    }

    @Override
    public void run(double speed){
        motor.set(speed);
    }
    @Override
    public Trigger atHeight(double height, double tolerance)    {
        return new Trigger(() -> MathUtil.isNear(height,
                                                getHeightMeters(),
                                                tolerance));
    }
    @Override
    // Returns elevator height in inches
    public double getHeight() {
        return encoder.getPosition();
    }
    @Override
    public double getHeightMeters(){

        return (encoder.getPosition() * ClimberConstants.kEncoderDistPerPulse);
    }
    @Override
    public void setPosition(double targetHeight) {
        double pidOutput = pid.calculate(getHeight(), targetHeight);
        
        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        
        // Clamp the output to valid range
        double motorOutput = pidOutput; // Add g comp once we know

        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
        
        motor.set(motorOutput);  
    }

    @Override 
    public void periodic() {}

    @Override
    public double getVelocityMetersPerSec() {
        return 0.0;
    }

}