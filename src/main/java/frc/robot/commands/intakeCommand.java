package frc.robot.commands;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class intakeCommand extends Command {
    private final Intake m_intake;
    private final StorageSub m_StorageSub;

    public intakeCommand(Intake intake, StorageSub storage) {
        this.m_intake = intake;
        this.m_StorageSub = storage;
        addRequirements(intake, storage);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_intake.runRollers(-6);
        m_StorageSub.runGround(-8);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.runRollers(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}