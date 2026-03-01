package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

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

    public void setArmPositionRad(double angleRad) {
        io.setArmPosition(angleRad);
    }

    public void setArmPositionDeg(double angleDeg) {
        io.setArmPosition(Units.degreesToRadians(angleDeg));
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