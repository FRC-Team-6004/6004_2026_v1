package frc.robot.commands;

import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDown extends Command {
  private final Climber m_climber;

  public ClimberDown(Climber climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climber.run(-12);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.run(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}