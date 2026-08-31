package frc.robot.subsystems.shooter;

public interface ShooterIO {

    /** Set target speed for one shooter (RPM) */
    void setTargetRPM(double rpm);


    /** Get current speed for one shooter (RPM) */
    double getRPM();

    /** Get current speed for one shooter (RPM) */
    // void setServoAngle(double percent);


    /** Stop one shooter */
    void stop();

}
