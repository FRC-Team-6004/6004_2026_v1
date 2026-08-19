package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.Shooter;

public class shooterUnjam extends Command {

    private final Shooter shooter;


    public shooterUnjam(Shooter shooterSub) {
        this.shooter = shooterSub;
        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setRPM(-2000);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
