package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSub extends SubsystemBase {
    TalonFX groundMotor;
    SparkFlex topMotor;

    /**
     * This subsytem that controls the arm.
     */
    public StorageSub() {
        groundMotor = new TalonFX(StorageConstants.GroundMotorID);
        topMotor = new SparkFlex(StorageConstants.RollerMotorID, MotorType.kBrushless);
        
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        groundMotor.getConfigurator().apply(motorConfig);

        var neoConfig = new SparkFlexConfig();
        neoConfig.idleMode(IdleMode.kBrake);
        neoConfig.smartCurrentLimit(55);
        topMotor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    @Override
    public void periodic() {}
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runMotor(double speed){
        groundMotor.set(speed);
        topMotor.set(-speed * .7);
    }
}