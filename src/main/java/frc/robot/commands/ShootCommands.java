package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.IntakeConstants;

public class ShootCommands {

    private static final double feedDelay = 0.4;

    /** Flywheel is considered "at speed" within this many RPM of target. Tune if feeding starts too early/late. */
    private static final double AT_SPEED_TOLERANCE_RPM = 200;

    /**
     * Builds a shoot command. The RPM supplier is evaluated live (on initialize and every execute),
     * so callers must NOT precompute the RPM at command-construction time: the robot's distance to
     * the hub at construction time (e.g. robot startup) is not its distance when shooting.
     * Feeding only starts once the spin-up delay has passed AND the flywheel is actually at speed.
     */
    public static Command createShootCommand(
        Shooter shooter,
        StorageSub storage,
        Intake intake,
        DoubleSupplier rpmSupplier
    ) {
        return new Command() {
            private final Timer timer = new Timer();
            private double targetRpm = 0;
            { addRequirements(shooter, storage, intake); }

            @Override
            public void initialize() {
                timer.reset();
                timer.start();

                targetRpm = rpmSupplier.getAsDouble();
                shooter.setRPM(targetRpm);
            }

            @Override
            public void execute() {
                // Keep tracking a live target (e.g. distance-based) while shooting.
                targetRpm = rpmSupplier.getAsDouble();
                shooter.setRPM(targetRpm);

                boolean atSpeed =
                    Math.abs(Math.abs(shooter.getRPM()) - Math.abs(targetRpm)) <= AT_SPEED_TOLERANCE_RPM;

                if (timer.get() > feedDelay && atSpeed) {
                    storage.runFloor(0.5);
                    storage.runTop(0.5);
                    intake.runRollers(-2);
                    intake.setArmControl(-3);
                } else {
                    // Never feed into a slow or stopped flywheel.
                    storage.runFloor(0);
                    storage.runTop(0);
                    intake.runRollers(0);
                    intake.setArmControl(IntakeConstants.PIDout);
                }
            }

            @Override
            public void end(boolean interrupted) {
                shooter.setRPM(0);

                storage.runFloor(0);
                storage.runTop(0);
                intake.runRollers(0);

                intake.setArmControl(IntakeConstants.PIDout);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
}
