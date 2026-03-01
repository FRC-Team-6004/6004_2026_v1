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
    }

    @Override
    public void setArmPosition(double targetAngle) {
        // convert linear height to angle assuming arc of arm length
        double targetAngleRad =
            MathUtil.clamp(
                targetAngle / armSim.getAngleRads(),
                0.0,
                1.0
            ) * (Units.degreesToRadians(90) - 0);

        double error = targetAngleRad - armSim.getAngleRads();
        double kP = 2.0; // simple sim-only proportional gain
        armInputVolts = MathUtil.clamp(kP * error, -12.0, 12.0);
    }

    @Override
    public double getAngle() {
        return armSim.getAngleRads();
    }

    @Override
    public Trigger atAngle(double targetAngle, double toleranceRad) {
        return new Trigger(() -> {
            double currentHeight =
                armSim.getAngleRads() * armSim.getAngleRads();
            return Math.abs(currentHeight - targetAngle) <= toleranceRad;
        });
    }

    @Override
    public void periodic() {
        armSim.setInputVoltage(armInputVolts);
        armSim.update(0.02);
    }

    @Override
    public void setBrake() {} //todo

}