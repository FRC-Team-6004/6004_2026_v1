package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

public class DoubleShooterLR extends Command {

  private final Shooter shooter;

  public DoubleShooterLR(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftRPM = 3500;
    double rightRPM = 3500;

    shooter.setRPM(ShooterSide.LEFT, leftRPM);
    shooter.setRPM(ShooterSide.RIGHT, rightRPM);

    shooter.setServoAngle(ShooterSide.LEFT, 0.5);
    shooter.setServoAngle(ShooterSide.RIGHT, 0.5);

  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(ShooterSide.LEFT, 0);
    shooter.setRPM(ShooterSide.RIGHT, 0);
    shooter.setServoAngle(ShooterSide.LEFT, 0);
    shooter.setServoAngle(ShooterSide.RIGHT, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
