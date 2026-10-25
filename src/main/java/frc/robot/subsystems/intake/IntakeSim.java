package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

public class IntakeSim implements IntakeIO {

    private final SingleJointedArmSim armSim;
    private final TalonFX rollerMotor;
    private final TalonFXSimState rollerSim;

    private double armInputVolts = 0.0;

    // Position-control target (arm radians). Set by setArmPosition; cleared to direct
    // voltage control whenever runArm is used, so the two modes never fight.
    private double targetArmAngleRad = 0.0;
    private boolean positionControlActive = false;

    public IntakeSim() {
        rollerMotor = new TalonFX(IntakeConstants.kRightMotorID);
        rollerSim = rollerMotor.getSimState();

        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2),      // motors
            IntakeConstants.kGearing,       // gearing
            0.153,                          // moment of inertia (kg m^2)
            0.3556,                         // arm length (m)
            0.0,                            // min angle (rad)
            Units.degreesToRadians(90),     // max angle (rad)
            false,                          // simulate gravity
            0.0,                            // starting angle
            0.1,                            // measurement noise
            0.0
        );
    }

    @Override
    public void runRollers(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        rollerSim.setSupplyVoltage(speed * 12.0);
    }

    @Override
    public void runArm(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        armInputVolts = speed * 12.0;
        positionControlActive = false;
    }

    @Override
    public void setArmPosition(double targetMotorRotations) {
        // The real IO takes motor rotations (MotionMagic target). Convert to arm
        // radians via the gearing and clamp to this model's range. NOTE: if the sim
        // arm moves the wrong way versus the real arm, the sign convention differs
        // and this conversion needs a leading minus. (The old version divided by the
        // current angle, which is a divide-by-zero at the 0 starting angle.)
        targetArmAngleRad = MathUtil.clamp(
            targetMotorRotations / IntakeConstants.kGearing * 2.0 * Math.PI,
            0.0,
            Units.degreesToRadians(90)
        );
        positionControlActive = true;
    }

    @Override
    public double getAngle() {
        return armSim.getAngleRads();
    }

    @Override
    public Trigger atAngle(double targetAngle, double toleranceRad) {
        // Both in arm radians (the old version squared the angle, which was nonsense).
        return new Trigger(() -> MathUtil.isNear(targetAngle, armSim.getAngleRads(), toleranceRad));
    }

    @Override
    public void periodic() {
        if (positionControlActive) {
            double kP = 2.0; // simple sim-only proportional gain, V per rad of error
            armInputVolts = MathUtil.clamp(
                kP * (targetArmAngleRad - armSim.getAngleRads()), -12.0, 12.0);
        }
        armSim.setInputVoltage(armInputVolts);
        armSim.update(0.02);
    }

    @Override
    public void setBrake() {} //todo

}