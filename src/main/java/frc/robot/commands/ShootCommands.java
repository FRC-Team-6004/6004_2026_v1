package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.IntakeConstants;

public class ShootCommands {

    private static final double feedDelay = 0.4;

    public static Command createShootCommand(
        Shooter shooter,
        StorageSub storage,
        Intake intake,
        double rpm,
        double servoAngle
    ) {
        return new Command() {
            private final Timer timer = new Timer();
            { addRequirements(shooter, storage, intake); }

            @Override
            public void initialize() {
                timer.reset();
                timer.start();

                shooter.setRPM(ShooterSide.MAIN, rpm);
                shooter.setServoAngle(ShooterSide.MAIN, rpm);
            }

            @Override
            public void execute() {
                if (timer.get() > feedDelay) {
                    storage.runFloor(0.5);
                    storage.runTop(0.5);
                    intake.runRollers(-2);
                    intake.setArmControl(-3);
                } else {
                    intake.setArmControl(IntakeConstants.PIDout);
                }
            }

            @Override
            public void end(boolean interrupted) {
                shooter.setRPM(ShooterSide.MAIN, 0);
                shooter.setServoAngle(ShooterSide.MAIN, 0);

                storage.runFloor(0);
                storage.runTop(0);
                intake.runRollers(0);

                intake.setArmControl(IntakeConstants.PIDout);
            }

            @Override
            public boolean isFinished() {
                return false; // use withTimeout externally
            }


        };
    }
    
}