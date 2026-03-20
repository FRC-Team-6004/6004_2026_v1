package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

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
import frc.robot.Constants.IntakeConstants;

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

    /* ---------------- Controls ---------------- */
    private final NeutralOut m_brake = new NeutralOut();
    private final PositionVoltage m_positionVoltage =
        new PositionVoltage(0).withSlot(0);

    private double targetPos = 0.0;

    public IntakeReal() {
        pivotLeader   = new TalonFX(IntakeConstants.kLeftMotorID);
        pivotFollower = new TalonFX(IntakeConstants.kRightMotorID);
        rollerMotor   = new TalonFX(IntakeConstants.kRollerMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limiting
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Voltage limiting
        config.Voltage
            .withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));

        // Slot 0 PID + gravity compensation
        config.Slot0.kP = 3.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kG = 1.0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        applyConfigWithRetry(pivotLeader, config);
        applyConfigWithRetry(pivotFollower, config);
        
        pivotFollower.setControl(
            new Follower(
                pivotLeader.getDeviceID(),
                MotorAlignmentValue.Opposed   // invert follower
            )
        );
         

        pivotLeader.setPosition(0.0);
    }

    private void setArmControl(double pos) {
        pivotLeader.setControl(
            m_positionVoltage.withPosition(pos)
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
        //setArmControl(targetPos);
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