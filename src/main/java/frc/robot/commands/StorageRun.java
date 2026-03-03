package frc.robot.commands;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class StorageRun extends Command {
  private final StorageSub storage;
  private double Magnitude;

  public StorageRun(StorageSub m_sub, double magnitude) {
    storage = m_sub;
    this.Magnitude = magnitude;
    addRequirements(m_sub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    storage.runMotor(Magnitude);
  }

  @Override
  public void end(boolean interrupted) {
    storage.runMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}