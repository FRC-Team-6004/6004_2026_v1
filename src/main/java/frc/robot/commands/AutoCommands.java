package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.util.ShooterLookup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.visionConstants;

public class AutoCommands {

    public static Command shootAuto(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub) {
        return new Command() {
            private final CommandSwerveDrivetrain swerve;
            private final Shooter shooter;
            private final ShooterLookup shootTable;
            private final StorageSub storage;
            Timer t = new Timer();

            {
                t.start();
                this.storage = ssub;
                this.shootTable = new ShooterLookup();
                this.swerve = drivetrain;
                this.shooter = shooterSub;
                addRequirements(drivetrain, shooterSub, ssub);
            }

            @Override 
            public void initialize() {
                t.reset();
            }

            @Override
            public void execute() {
                Pose2d currentPose = swerve.getState().Pose;
                
                Pose2d targetPose = isRed() ? visionConstants.redHubPos : visionConstants.hubPos;
                Translation2d robotTranslation = currentPose.getTranslation();
                Translation2d targetTranslation = targetPose.getTranslation();

                double distance = robotTranslation.getDistance(targetTranslation);

                double RPM = shootTable.getRPM(distance);
                double servo = shootTable.getServo(distance);

                shooter.setRPM(ShooterSide.MAIN, RPM);

                shooter.setServoAngle(ShooterSide.MAIN, servo);


                storage.runFloor(8);
                if (t.get() > 0.5) {
                    storage.runTop(12);
                }
            }

            @Override
            public void end(boolean interrupted) {
                shooter.setRPM(ShooterSide.MAIN, 0);
                shooter.setServoAngle(ShooterSide.MAIN, .2);
                storage.runFloor(0);
                storage.runTop(0);
            }

            @Override
            public boolean isFinished() {
                return t.hasElapsed(7);
            }

            private boolean isRed() {
                return (swerve.getState().Pose.getX() > 8.25);
            }
        };
    }

    public static Command extendAuto(Intake intake) {
        return new Command() {
            private final Intake m_intake;
            Timer t = new Timer();
            { t.start(); this.m_intake = intake;}
            @Override public void initialize() { t.reset(); }
            @Override public void execute() { m_intake.runArm(-1); }
            @Override public void end(boolean i) { m_intake.runArm(0); }
            @Override public boolean isFinished() { return t.hasElapsed(1.5); }
        };
    }
    

    public static Command startIntake(Intake intake, StorageSub storage) {
        return new Command() {
            private final Intake m_intake;
            private final StorageSub m_StorageSub;

            {
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
                m_StorageSub.runFloor(8);
            }

            @Override
            public void end(boolean interrupted) {
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public static Command stopIntake(Intake intake, StorageSub storage) {
        return new Command() {
            private final Intake m_intake;
            private final StorageSub m_StorageSub;

            {
                this.m_intake = intake;
                this.m_StorageSub = storage;
                addRequirements(intake, storage);
            }

            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                m_intake.runRollers(0);
                m_StorageSub.runFloor(0);
            }

            @Override
            public void end(boolean interrupted) {
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

}