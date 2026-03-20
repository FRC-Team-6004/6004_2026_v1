package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.util.ShooterLookup;
import frc.robot.subsystems.intake.Intake;

public class AutoCommands {

    public static Command shootAuto(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub) {
        return new Command () {
            Timer t = new Timer();
            private final Shooter shooter;
            private final ShooterLookup shootTable;
            private final StorageSub storage;
            private final CommandSwerveDrivetrain swerve;
            private static Pose2d hubPos = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d());
            private static Pose2d redHubPos = FlippingUtil.flipFieldPose(hubPos);

            {
                System.out.println("AutoShoot");
                t.start();
                t.reset();
                this.storage = ssub;
                this.shootTable = new ShooterLookup();
                this.swerve = drivetrain;
                this.shooter = shooterSub;
                addRequirements(drivetrain, shooterSub, ssub);
            }

            @Override public void initialize() {
                t.reset();
            }

            @Override public void execute() { 
                Pose2d currentPose = swerve.getState().Pose;

                double distance = 0;
                if (Robot.isRed()) {
                    distance = currentPose.getTranslation().getDistance(redHubPos.getTranslation());
                } else {
                    distance = currentPose.getTranslation().getDistance(hubPos.getTranslation());
                }

                double RPM = shootTable.getRPM(distance);
                double servo = shootTable.getServo(distance);

                shooter.setRPM(ShooterSide.MAIN, RPM);
        
                shooter.setServoAngle(ShooterSide.MAIN, servo);

                storage.runFloor(12);
                storage.runTop(8);
            }

            @Override
            public void end(boolean interrupted) {
                shooter.setRPM(ShooterSide.LEFT, 0);
                shooter.setRPM(ShooterSide.RIGHT, 0);

                shooter.setServoAngle(ShooterSide.LEFT, .2);
                shooter.setServoAngle(ShooterSide.RIGHT, .2);

                storage.runTop(0);
                storage.runFloor(0);
            }

            @Override
            public boolean isFinished() {
                 return t.hasElapsed(5); 
            }
        };
    }

    public static Command extendAuto(Intake intake) {
        return new Command() {
            private final Intake m_intake;
            Timer t = new Timer();
            { t.start(); this.m_intake = intake;}
            @Override public void initialize() { t.reset(); }
            @Override public void execute() { m_intake.runArm(-2); }
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