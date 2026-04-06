package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ShooterLookup;
import frc.robot.Constants.visionConstants;
import frc.robot.Constants.IntakeConstants;

public class AutoCommands {

    /** Shoot at hub using ShooterLookup, time-limited externally */
    public static Command shootAuto(CommandSwerveDrivetrain drivetrain, Shooter shooter, StorageSub storage, Intake intake) {
        ShooterLookup shootTable = new ShooterLookup();
        return ShootCommands.createShootCommand(
                shooter,
                storage,
                intake,
                shootTable.getRPM(getDistance(drivetrain)),
                shootTable.getServo(getDistance(drivetrain))
        ).withTimeout(5);
    }

    /** Move intake arm out for a short duration */
    public static Command extendAuto(Intake intake) {
        return new InstantCommand(() -> {
            intake.setArmControl(IntakeConstants.PIDout);
        }, intake);
    }

    /** Start intake rollers and storage feed */
    public static Command startIntake(Intake intake, StorageSub storage) {
        return new InstantCommand(() -> {
            intake.runRollers(-10);
            storage.runFloor(0);
        }, intake, storage);
    }

    /** Stop intake and storage */
    public static Command stopIntake(Intake intake, StorageSub storage) {
        return new InstantCommand(() -> {
            intake.runRollers(0);
            storage.runFloor(0);
        }, intake, storage);
    }

    /** Helper to calculate distance to hub */
    private static double getDistance(CommandSwerveDrivetrain drivetrain) {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d targetPose = isRed(drivetrain) ? visionConstants.redHubPos : visionConstants.hubPos;
        return currentPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    /** Determine alliance side based on position */
    private static boolean isRed(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.getState().Pose.getX() > 8.25;
    }
}