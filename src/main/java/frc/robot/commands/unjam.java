package frc.robot.commands;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class unjam extends Command {
    private final StorageSub m_StorageSub;

    public unjam(StorageSub storage) {
        this.m_StorageSub = storage;
        addRequirements(storage);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_StorageSub.runFloor(-12);
        m_StorageSub.runTop(-12);
    }

    @Override
    public void end(boolean interrupted) {
        m_StorageSub.runTop(0);
        m_StorageSub.runFloor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}