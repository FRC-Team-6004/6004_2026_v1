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
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ShooterLookup;

import frc.robot.Constants.visionConstants;

import org.littletonrobotics.junction.Logger;

public class ShootAtHub extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final Shooter shooter;

    private final ShooterLookup shootTable;

    private final StorageSub storage;

    private final VisionSub vision;

    private final Intake m_Intake;

    Timer t = new Timer();

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double ROT_KP = 2.5;

    public ShootAtHub(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub, VisionSub vsub, Intake intake) {
        t.start();
        this.storage = ssub;
        this.vision = vsub;
        this.shootTable = new ShooterLookup();
        this.swerve = drivetrain;
        this.shooter = shooterSub;
        this.m_Intake = intake;
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


        double RPM = Math.abs(shootTable.getRPM(Math.abs(distance)));
        double servo = shootTable.getServo(Math.abs(distance));

        System.out.println(distance);

        shooter.setRPM(ShooterSide.MAIN, RPM);

        shooter.setServoAngle(ShooterSide.MAIN, servo);

        if (t.get() > 1) {
            storage.runFloor(4);
            storage.runTop(4);
        }
        
        if (t.get() > 2 && t.get() < 2.6) {
            m_Intake.runArm(2);
        } else {
            m_Intake.runArm(0);
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
        shooter.setServoAngle(ShooterSide.MAIN, 0);
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
