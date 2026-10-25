package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;

    public static double distance = 0;

    public Shooter() {
        io = RobotBase.isReal() ? new ShooterReal() : new ShooterSim();
    }

    public void setRPM(double rpm) {
        io.setTargetRPM(rpm);
    }

    public void stop() {
        io.stop();
    }

    public double getRPM() {
        return io.getRPM();
    }

    @Override
    public void periodic() {
        if (io instanceof ShooterReal real) {
            real.periodic();
        } else if (io instanceof ShooterSim sim) {
            sim.periodic();
        }

        Logger.recordOutput("/Shooter/RPM", -getRPM());
        Logger.recordOutput("/Shooter/Distance", distance);
    }
}
