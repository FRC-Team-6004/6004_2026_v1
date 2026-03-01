package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IntakeIO {
    void runRollers(double speed);

    void runArm(double speed);

    void setArmPosition(double targetAngle);

    default void stopRollers() {
        runRollers(0.0);
    }
    
    default void stopArm() {
        runArm(0.0);
    }

    double getAngle();

    Trigger atAngle (double targetAngle, double toleranceRad);

    void periodic();

    void setBrake();

}
