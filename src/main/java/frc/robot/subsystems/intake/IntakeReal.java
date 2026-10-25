package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;

/**
 * Intake pivot subsystem with two mechanically mirrored arm motors.
 * One leader, one inverted follower, both using CTRE position control
 * with gravity compensation.
 */
public class IntakeReal implements IntakeIO {

    //Motors
    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;
    private final TalonFX rollerMotor;
    private final TalonFX rollerMotorFollower;


    //Controls
    private final NeutralOut m_brake = new NeutralOut();

    private double targetPos = 0.0;

    /** While true, the POV jog owns the arm and periodic() must not fight it with MotionMagic. */
    private boolean jogActive = false;

    private Timer t = new Timer();

    private final MotionMagicExpoVoltage m_motionMagic = new MotionMagicExpoVoltage(0).withSlot(0);


    public IntakeReal() {
        pivotLeader        = new TalonFX(IntakeConstants.kLeftMotorID);
        pivotFollower      = new TalonFX(IntakeConstants.kRightMotorID);
        rollerMotor        = new TalonFX(IntakeConstants.kRollerMotorID);
        rollerMotorFollower        = new TalonFX(IntakeConstants.kRollerMotorFollowerID);


        t.start();

        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        TorqueCurrentFOC TCF = new TorqueCurrentFOC(0);

        config.Audio.AllowMusicDurDisable = true;
        rollerConfig.Audio.AllowMusicDurDisable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = 10;   // rotations/sec
        config.MotionMagic.MotionMagicAcceleration = 30;    // rotations/sec^2
        config.MotionMagic.MotionMagicJerk = 1000;          // controls curve aggressiveness
        config.MotionMagic.MotionMagicExpo_kV = 0.12;       // velocity feedforward
        config.MotionMagic.MotionMagicExpo_kA = 0.01;       // accel feedforward


        // Brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        // Current limiting
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfig.CurrentLimits.SupplyCurrentLimit = 25;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;


        // Voltage limiting
        config.Voltage
            .withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));

        // Slot 0 PID + gravity compensation
        config.Slot0.kP = 3;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kG = .12;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        applyConfigWithRetry(pivotLeader, config);
        applyConfigWithRetry(pivotFollower, config);
        applyConfigWithRetry(rollerMotor, rollerConfig);
        // Was missing: the roller follower ran on factory defaults (no current limit, coast).
        applyConfigWithRetry(rollerMotorFollower, rollerConfig);
        
        pivotFollower.setControl(
            new Follower(
                pivotLeader.getDeviceID(),
                MotorAlignmentValue.Opposed   // invert follower
            )
        );

        rollerMotorFollower.setControl(
            new Follower(
                rollerMotor.getDeviceID(),
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
        if (speed == 0) {
            // Jog released: park the position target where the arm actually is, so the
            // position loop doesn't drag the arm back to the old target afterwards.
            jogActive = false;
            targetPos = getAngle();
            pivotLeader.set(0);
        } else {
            jogActive = true;
            pivotLeader.set(speed *.1);
        }
    }

    @Override
    public void periodic() {
        double actualPos = targetPos;
        // Skip the position loop while jogging: it used to re-send MotionMagic every loop and
        // override the open-loop jog, so POV left/right barely moved the arm.
        if (!jogActive && Math.abs(getAngle() - targetPos) > 0.05) {
            setArmControl(Math.min(actualPos, 0));
        }
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
        // Duty cycle in [-1, 1]; callers pass volt-scale values, clamp like the sim does.
        rollerMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
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