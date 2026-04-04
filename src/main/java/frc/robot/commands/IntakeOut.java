package frc.robot.commands;

import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOut extends Command {
  private final Intake m_intake;

  public IntakeOut(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.runArm(-4);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.runArm(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}