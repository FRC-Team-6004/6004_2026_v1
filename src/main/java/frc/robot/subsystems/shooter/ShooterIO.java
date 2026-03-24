package frc.robot.subsystems.shooter;

/**
 * Hardware abstraction for a dual-shooter system.
 * Each side is controlled independently.
 */
public interface ShooterIO {

    /** Set target speed for one shooter (RPM) */
    void setTargetRPM(ShooterSide side, double rpm);


    /** Get current speed for one shooter (RPM) */
    double getRPM(ShooterSide side);

    /** Get current speed for one shooter (RPM) */
    void setServoAngle(ShooterSide side, double percent);


    /** Stop one shooter */
    void stop(ShooterSide side);

    /** Stop both shooters */
    default void stopAll() {
        stop(ShooterSide.LEFT);
        stop(ShooterSide.RIGHT);
    }
}
