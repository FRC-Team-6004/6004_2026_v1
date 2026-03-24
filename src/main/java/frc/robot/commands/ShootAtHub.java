package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;


import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.subsystems.VisionSub;

import frc.robot.util.ShooterLookup;

import frc.robot.Constants.visionConstants;

public class ShootAtHub extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final Shooter shooter;

    private final ShooterLookup shootTable;

    private final StorageSub storage;

    private final VisionSub vision;

    Timer t = new Timer();

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double ROT_KP = 3.5;

    public ShootAtHub(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub, VisionSub vsub) {
        t.start();
        this.storage = ssub;
        this.vision = vsub;
        this.shootTable = new ShooterLookup();
        this.swerve = drivetrain;
        this.shooter = shooterSub;
        addRequirements(drivetrain, shooterSub, ssub);
    }

    @Override
    public void execute() {

        vision.addVisionMeasurement(swerve);


        Pose2d currentPose = swerve.getState().Pose;

        Pose2d targetPose = isRed() ? visionConstants.redHubPos : visionConstants.hubPos;

        Translation2d robotTranslation = currentPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();

        // Vector from robot → target
        Translation2d toTarget = targetTranslation.minus(robotTranslation);

        // Field-relative angle to target
        Rotation2d headingToTarget = toTarget.getAngle();

        // Error (robot-relative)
        double headingError =
            headingToTarget.minus(currentPose.getRotation()).getRadians();

        double omega = headingError * ROT_KP;
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(omega)
        );

        double distance = robotTranslation.getDistance(targetTranslation);

        System.out.println("Distance: " + distance);
        System.out.println("error " + headingError);

        double RPM = Math.abs(shootTable.getRPM(Math.abs(distance)));
        double servo = shootTable.getServo(Math.abs(distance));

        System.out.println("RPM: " + RPM);
        System.out.println("servo: " + servo);


        shooter.setRPM(ShooterSide.MAIN, RPM);

        shooter.setServoAngle(ShooterSide.MAIN, servo);

        storage.runFloor(8);
        if (t.get() > 0.5) {
            storage.runTop(12);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
        shooter.setRPM(ShooterSide.MAIN, 0);

        shooter.setServoAngle(ShooterSide.MAIN, .2);

        storage.runFloor(0);
        storage.runTop(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        t.reset();
    }

    private boolean isRed() {
        return (swerve.getState().Pose.getX() > 8.25);
    }
}
