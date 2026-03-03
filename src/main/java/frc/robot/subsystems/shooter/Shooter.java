package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;

    private final Mechanism2d mech;
    private final MechanismLigament2d leftShooter;
    private final MechanismLigament2d rightShooter;
    private boolean isSim = false;

    private LoggedTunableNumber rpm = new LoggedTunableNumber("Shooter/rpm", 0);
    private LoggedTunableNumber servoAngle = new LoggedTunableNumber("Shooter/servoAngle", 0);
    
    public Shooter() {
        io = RobotBase.isReal() ? new ShooterReal() : new ShooterSim();

        mech = new Mechanism2d(2.0, 2.0);
        if (RobotBase.isReal()) {
            isSim = true;
            MechanismRoot2d root = mech.getRoot("ShooterRoot", 1.0, 1.0);

            leftShooter = root.append(
                    new MechanismLigament2d(
                            "LeftShooter",
                            0.2, // initial length
                            180.0, // left
                            6.0,
                            new Color8Bit(255, 0, 0)));

            rightShooter = root.append(
                    new MechanismLigament2d(
                            "RightShooter",
                            0.2, // initial length
                            0.0, // right
                            6.0,
                            new Color8Bit(0, 0, 255)));

            SmartDashboard.putData("Shooter", mech);
        } else {
            leftShooter = null;
            rightShooter = null;
        }

    }

    public void setRPM(ShooterSide side, double rpm) {
        io.setTargetRPM(side, rpm);
    }

    public void stop(ShooterSide side) {
        io.stop(side);
    }

    public void stopAll() {
        io.stopAll();
    }

    public double getRPM(ShooterSide side) {
        return io.getRPM(side);
    }

    public void setServoAngle(ShooterSide side, double percent) {
        io.setServoAngle(side, percent);
    }

    int cycler = 0;

    @Override
    public void periodic() {
        if (io instanceof ShooterReal real) {
            real.periodic();
        } else if (io instanceof ShooterSim sim) {
            sim.periodic();
        }
        
        if (cycler == 10) {
            cycler = 0;
            setServoAngle(ShooterSide.LEFT, servoAngle.get());
            setServoAngle(ShooterSide.RIGHT, servoAngle.get());
        
            double leftRPM0 = rpm.get();
            double rightRPM0 = rpm.get();
            setRPM(ShooterSide.LEFT, leftRPM0);
            setRPM(ShooterSide.RIGHT, rightRPM0);
        }

        cycler++;

        

        double leftRPM = getRPM(ShooterSide.LEFT);
        double rightRPM = getRPM(ShooterSide.RIGHT);

        Logger.recordOutput("/Shooter/Left RPM", -leftRPM);
        Logger.recordOutput("/Shooter/Right RPM", -rightRPM);

        if (isSim) {
            leftShooter.setLength(0.1 + leftRPM / 8000.0);
            rightShooter.setLength(0.1 + rightRPM / 8000.0);
        }
    }
}
