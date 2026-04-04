package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;

    public static double distance = 0;

    private LoggedTunableNumber rpm = new LoggedTunableNumber("Shooter/rpm", 0);
    private LoggedTunableNumber servoAngle = new LoggedTunableNumber("Shooter/servoAngle", 0);
    
    public Shooter() {
        io = RobotBase.isReal() ? new ShooterReal() : new ShooterSim();

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
            setServoAngle(ShooterSide.MAIN, servoAngle.get());
        
            double mainRPM0 = rpm.get();
            setRPM(ShooterSide.MAIN, mainRPM0);
        }


        cycler++;

        
        double mainRPM = getRPM(ShooterSide.MAIN);

        Logger.recordOutput("/Shooter/RPM", -mainRPM);
        Logger.recordOutput("/Shooter/Distance", distance);

    }
}
