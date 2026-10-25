package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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

public class StorageSub extends SubsystemBase {
    TalonFX topRoller = new TalonFX(StorageConstants.GroundMotorID);
    SparkFlex floor;

    public StorageSub() {
        // topRoller = new TalonFX(StorageConstants.GroundMotorID);
        floor = new SparkFlex(StorageConstants.RollerMotorID, MotorType.kBrushless);
        
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        motorConfig.Audio.AllowMusicDurDisable = true;

        topRoller.getConfigurator().apply(motorConfig);

        var neoConfig = new SparkFlexConfig();
        neoConfig.idleMode(IdleMode.kBrake);
        neoConfig.smartCurrentLimit(20);

        // Was commented out: the floor motor ran on factory defaults (coast, no current limit).
        floor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    /** Duty cycle in [-1, 1]. Callers used to pass volt-scale values (-12..12); clamp so "2" and "-10" stop pretending to be different speeds. */
    public void runTop(double speed){
        topRoller.set(-MathUtil.clamp(speed, -1.0, 1.0));
    }

    /** Duty cycle in [-1, 1]; see runTop. */
    public void runFloor(double speed){
        floor.set(MathUtil.clamp(speed, -1.0, 1.0));
    }


}