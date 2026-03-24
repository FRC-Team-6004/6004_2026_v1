package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

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
        shooter.setRPM(ShooterSide.MAIN, -2000);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(ShooterSide.MAIN, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
