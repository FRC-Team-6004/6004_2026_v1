package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final IntakeIO io;

    // Visualization
    private final Mechanism2d mech;
    private final MechanismLigament2d arm;

    public Intake() {
        io = RobotBase.isReal() ? new IntakeReal() : new IntakeSim();

        mech = new Mechanism2d(2.0, 2.0);

        MechanismRoot2d root =
            mech.getRoot("IntakeRoot", 1.0, 0.25);

        arm = root.append(
            new MechanismLigament2d(
                "IntakeArm",
                0.35,          // arm length (m)
                0.0,           // starting angle (deg)
                6.0,
                new Color8Bit(0, 255, 255)
            )
        );

        SmartDashboard.putData("Intake", mech);
    }

    /* -------------------- Control -------------------- */

    public void runRollers(double speed) {
        io.runRollers(speed);
    }

    public void runArm(double speed) {
        io.runArm(speed);
    }

    public void setBrake() {
        io.setBrake();
    }

    /**
     * Set the arm angle in mechanism radians. The real IO works in motor rotations (MotionMagic),
     * so convert: motor rotations = angle (rev) * gearing. The old code stored radians as if they
     * were rotations, and positive targets were then clamped to 0 by the real periodic loop.
     */
    public void setArmPositionRad(double angleRad) {
        io.setArmPosition(angleRad / (2.0 * Math.PI) * IntakeConstants.kGearing);
    }

    public void setArmPositionDeg(double angleDeg) {
        setArmPositionRad(Units.degreesToRadians(angleDeg));
    }

    public void setArmControl(double control) {
        io.setArmPosition(control);
    }

    /* -------------------- State -------------------- */

    public double getAngleRad() {
        return io.getAngle();
    }

    /* -------------------- Periodic -------------------- */

    @Override
    public void periodic() {
        io.periodic();

        // Update Mechanism2d (convert radians → degrees)
        double angleDeg = Units.radiansToDegrees(getAngleRad());
        arm.setAngle(angleDeg);
    }
}