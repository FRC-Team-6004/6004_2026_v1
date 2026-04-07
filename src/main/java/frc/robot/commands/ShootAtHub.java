package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ShooterLookup;
import frc.robot.Constants.visionConstants;

public class ShootAtHub extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final double ROT_KP = 2.5;

    private final Command shooterCommand;
    private final ShooterLookup shootTable;
    private final Shooter shooter;

    public ShootAtHub(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooterSub,
        StorageSub storage,
        Intake intake
    ) {
        this.shooter = shooterSub;
        this.swerve = drivetrain;
        this.shootTable = new ShooterLookup();

        double RPM = Math.abs(shootTable.getRPM(Math.abs(Shooter.distance)));
        double servo = shootTable.getServo(Math.abs(Shooter.distance));

        this.shooterCommand = ShootCommands.createShootCommand(
            shooterSub,
            storage,
            intake,
            RPM,
            servo
        );

        addRequirements(drivetrain, shooterSub, storage, intake);
    }

    @Override
    public void initialize() {
        shooterCommand.initialize();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getState().Pose;
        Pose2d targetPose = isRed(currentPose) ? visionConstants.redHubPos : visionConstants.hubPos;

        Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double headingError = toTarget.getAngle().minus(currentPose.getRotation()).getRadians();
        double omega = headingError * ROT_KP;

        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(omega)
        );
        shooter.setRPM(ShooterSide.MAIN, Math.abs(shootTable.getRPM(Math.abs(Shooter.distance))));
        shooter.setServoAngle(ShooterSide.MAIN, Math.abs(shootTable.getServo(Math.abs(Shooter.distance))));
        shooterCommand.execute();
        
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
        shooterCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isRed(Pose2d pose) {
        return pose.getX() > 8.25;
    }
}