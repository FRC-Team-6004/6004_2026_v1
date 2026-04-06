package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import org.littletonrobotics.junction.Logger;

/**
 * Intake pivot subsystem with two mechanically mirrored arm motors.
 * One leader, one inverted follower, both using CTRE position control
 * with gravity compensation.
 */
public class IntakeReal implements IntakeIO {

    /* ---------------- Motors ---------------- */
    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;
    private final TalonFX rollerMotor;
    private final TalonFX rollerMotorOpposed;


    /* ---------------- Controls ---------------- */
    private final NeutralOut m_brake = new NeutralOut();
    private final PositionVoltage m_positionVoltage =
        new PositionVoltage(0).withSlot(0);

    private double targetPos = 0.0;

    private Timer t = new Timer();

    private final MotionMagicExpoVoltage m_motionMagic = new MotionMagicExpoVoltage(0).withSlot(0);


    public IntakeReal() {
        pivotLeader        = new TalonFX(IntakeConstants.kLeftMotorID);
        pivotFollower      = new TalonFX(IntakeConstants.kRightMotorID);
        rollerMotor        = new TalonFX(IntakeConstants.kRollerMotorID);
        rollerMotorOpposed = new TalonFX(IntakeConstants.kRollerMotor2ID);

        t.start();

        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        config.MotionMagic.MotionMagicCruiseVelocity = 10;   // rotations/sec
        config.MotionMagic.MotionMagicAcceleration = 30;    // rotations/sec^2
        config.MotionMagic.MotionMagicJerk = 1000;          // controls curve aggressiveness
        config.MotionMagic.MotionMagicExpo_kV = 0.12;       // velocity feedforward
        config.MotionMagic.MotionMagicExpo_kA = 0.01;       // accel feedforward


        // Brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        // Current limiting
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfig.CurrentLimits.SupplyCurrentLimit = 10;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerConfig.CurrentLimits.StatorCurrentLimit = 45;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;


        // Voltage limiting
        config.Voltage
            .withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));

        // Slot 0 PID + gravity compensation
        config.Slot0.kP = 4;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kG = .12;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        applyConfigWithRetry(pivotLeader, config);
        applyConfigWithRetry(pivotFollower, config);
        applyConfigWithRetry(rollerMotor, rollerConfig);
        applyConfigWithRetry(rollerMotorOpposed, rollerConfig);

        rollerMotorOpposed.setControl(new Follower(rollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        
        pivotFollower.setControl(
            new Follower(
                pivotLeader.getDeviceID(),
                MotorAlignmentValue.Opposed   // invert follower
            )
        );
         

        pivotLeader.setPosition(0.0);
    }

    private void setArmControl(double pos) {
        // pivotLeader.setControl(
        //     m_positionVoltage.withPosition(pos)
        // );

        pivotLeader.setControl(
            m_motionMagic.withPosition(pos)
        );

    }

    @Override
    public void setBrake() {
        pivotLeader.setControl(m_brake);
    }

    private void moveArm(double speed) {
        pivotLeader.set(speed *.1);
    }

    @Override
    public void periodic() {
        double actualPos = targetPos;
        if (RobotContainer.bounceIntake) {
            actualPos = (targetPos + Math.sin(t.get() * 7.5) * 1) + 0.5;
        }
        // setArmControl(Math.min(actualPos, 0));
        Logger.recordOutput("/intake/arm position", getAngle());
    }

    private static void applyConfigWithRetry(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) return;
        }
        System.out.println(
            "Failed to apply TalonFX config to ID "
                + motor.getDeviceID()
                + " : "
                + status
        );
    }

    @Override
    public void runRollers(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void runArm(double speed) {
        moveArm(speed);
    }

    @Override
    public void setArmPosition(double targetAngle) {
        targetPos = targetAngle;
    }


    @Override
    public double getAngle() {
        return pivotLeader.getPosition().getValueAsDouble();
    }
    
    @Override
    public Trigger atAngle(double angle, double tolerance)    {
        return new Trigger(() -> MathUtil.isNear(angle,
                                                getAngle(),
                                                tolerance));
    }
}