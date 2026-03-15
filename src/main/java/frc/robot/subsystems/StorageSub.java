package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
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

import frc.robot.Constants.StorageConstants;
import frc.robot.util.LoggedTunableNumber;

public class StorageSub extends SubsystemBase {
    TalonFX groundMotor;
    SparkFlex topMotor;

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
        neoConfig.smartCurrentLimit(65);

        //  topMotor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    @Override
    public void periodic() {

    }

    public void runGround(double speed){
        groundMotor.set(-speed);
    }

    public void runNeo(double speed){
        topMotor.set(speed);
    }


}