package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

public class DoubleShooterLR extends Command {

  private final Shooter shooter;
  private final DoubleSupplier leftInput;
  private final DoubleSupplier rightInput;
  private final DoubleSupplier leftServo;
  private final DoubleSupplier rightServo;
  private final BooleanSupplier enableServo;



  public DoubleShooterLR(Shooter shooter, DoubleSupplier L_Input, DoubleSupplier R_Input, DoubleSupplier L_Servo, DoubleSupplier R_Servo, BooleanSupplier servoOn) {
    this.shooter = shooter;
    this.leftInput = L_Input;
    this.rightInput = R_Input;
    this.leftServo = L_Servo;
    this.rightServo = R_Servo;
    this.enableServo = servoOn;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftRPM = 4500.0 * leftInput.getAsDouble();
    double rightRPM = 4500.0 * rightInput.getAsDouble();

    shooter.setRPM(ShooterSide.LEFT, leftRPM);
    shooter.setRPM(ShooterSide.RIGHT, rightRPM);
    if (enableServo.getAsBoolean()) {
      shooter.setServoAngle(ShooterSide.LEFT, leftServo.getAsDouble());
      shooter.setServoAngle(ShooterSide.RIGHT, rightServo.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(ShooterSide.LEFT, 0);
    shooter.setRPM(ShooterSide.RIGHT, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
