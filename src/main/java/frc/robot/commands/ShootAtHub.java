package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.io.IOException;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Robot;
import frc.robot.Constants.PathingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.util.ShooterLookup;

import com.pathplanner.lib.util.FlippingUtil;

public class ShootAtHub extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final Shooter shooter;

    private final ShooterLookup shootTable;

    private final StorageSub storage;

    Timer t = new Timer();

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double ROT_KP = 3.5;
    private static Pose2d hubPos = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d());
    private static Pose2d redHubPos = FlippingUtil.flipFieldPose(hubPos);


    public ShootAtHub(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub) {
        t.start();
        this.storage = ssub;
        this.shootTable = new ShooterLookup();
        this.swerve = drivetrain;
        this.shooter = shooterSub;
        addRequirements(drivetrain, shooterSub, ssub);
    }

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getState().Pose;
        Rotation2d headingToTarget = new Rotation2d();

        if (Robot.isRed()) {
            headingToTarget = redHubPos.minus(currentPose).getTranslation().getAngle();
        } else {
            headingToTarget = hubPos.minus(currentPose).getTranslation().getAngle();
        }
        
        double headingError =
            headingToTarget.minus(currentPose.getRotation()).getRadians();

        // double omega = headingError * ROT_KP;
        // swerve.setControl(
        //     drive.withVelocityX(0)
        //          .withVelocityY(0)
        //          .withRotationalRate(omega)
        // );

        double distance = currentPose.getTranslation().getDistance(hubPos.getTranslation());

        double RPM = shootTable.getRPM(distance);
        double servo = shootTable.getServo(distance);

        shooter.setRPM(ShooterSide.LEFT, RPM);
        shooter.setRPM(ShooterSide.RIGHT, RPM);

        shooter.setServoAngle(ShooterSide.LEFT, servo);
        shooter.setServoAngle(ShooterSide.RIGHT, servo);


        storage.runNeo(12);
        if (t.get() > 0.5) {
            storage.runGround(8);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
        shooter.setRPM(ShooterSide.LEFT, 0);
        shooter.setRPM(ShooterSide.RIGHT, 0);

        shooter.setServoAngle(ShooterSide.LEFT, .2);
        shooter.setServoAngle(ShooterSide.RIGHT, .2);

        storage.runNeo(0);
        storage.runGround(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
