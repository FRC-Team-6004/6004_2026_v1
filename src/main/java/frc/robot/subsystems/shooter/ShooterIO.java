package frc.robot.subsystems.shooter;

public interface ShooterIO {

    /** Set target speed for one shooter (RPM) */
    void setTargetRPM(double rpm);


    /** Get current speed for one shooter (RPM) */
    double getRPM();

    /** Stop one shooter */
    void stop();

}
