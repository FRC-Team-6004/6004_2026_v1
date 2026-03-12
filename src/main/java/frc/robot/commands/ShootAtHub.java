package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.PathingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.util.ShooterLookup;

public class ShootAtHub extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final Shooter shooter;

    private final ShooterLookup shootTable;

    private final StorageSub storage;


    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double ROT_KP = 3.5;
    private static final Pose2d hubPos = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d());


    public ShootAtHub(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub) {
        this.storage = ssub;
        this.shootTable = new ShooterLookup();
        this.swerve = drivetrain;
        this.shooter = shooterSub;
        addRequirements(drivetrain, shooterSub, ssub);
    }

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getState().Pose;
    
        Rotation2d headingToTarget = hubPos.minus(currentPose).getTranslation().getAngle();
        
        double headingError =
            headingToTarget.minus(currentPose.getRotation()).getRadians();

        double omega = headingError * ROT_KP;
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(omega)
        );

        double distance = currentPose.getTranslation().getDistance(hubPos.getTranslation());

        double RPM = shootTable.getRPM(distance);
        double servo = shootTable.getServo(distance);

        shooter.setRPM(ShooterSide.LEFT, RPM);
        shooter.setRPM(ShooterSide.RIGHT, RPM);

        shooter.setServoAngle(ShooterSide.LEFT, servo);
        shooter.setServoAngle(ShooterSide.RIGHT, servo);

        storage.runNeo(12);
        storage.runGround(8);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
