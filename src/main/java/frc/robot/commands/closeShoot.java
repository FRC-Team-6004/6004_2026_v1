package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

public class closeShoot extends Command {

    private final Shooter shooter;
    private final StorageSub storage;

    Timer t = new Timer();

    public closeShoot(Shooter shooterSub, StorageSub ssub) {
        t.start();
        this.storage = ssub;
        this.shooter = shooterSub;
        addRequirements(shooterSub, ssub);
    }

    @Override
    public void initialize() {
        t.reset();
    }

    @Override
    public void execute() {

        shooter.setRPM(ShooterSide.MAIN, 3000);

        shooter.setServoAngle(ShooterSide.MAIN, 0);

        if (t.get() > 1) {
            storage.runFloor(4);
            storage.runTop(4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(ShooterSide.MAIN, 0);

        shooter.setServoAngle(ShooterSide.MAIN, 0);

        storage.runFloor(0);
        storage.runTop(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
