package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.VisionSub;
import frc.robot.util.ShooterLookup;
import frc.robot.util.AllianceUtil;

public class ShootAtHub extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final double ROT_KP = 2.5;

    private final Command shooterCommand;
    private final ShooterLookup shootTable;

    public ShootAtHub(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooterSub,
        StorageSub storage,
        Intake intake,
        VisionSub vision
    ) {
        this.swerve = drivetrain;
        this.shootTable = new ShooterLookup();

        // RPM is supplied live (see ShootCommands): it must NOT be snapshotted here, because this
        // command object is constructed once when bindings are created, long before it runs.
        // Shooter.distance is refreshed every robot loop from the live pose (RobotContainer.periodic).
        this.shooterCommand = ShootCommands.createShootCommand(
            shooterSub,
            storage,
            intake,
            () -> Math.abs(shootTable.getRPM(Math.abs(Shooter.distance)))
        );

        addRequirements(drivetrain, shooterSub, storage, intake, vision);
    }

    @Override
    public void initialize() {
        vision.addVisionMeasurement(drivetrain);
        shooterCommand.initialize();

    }

    @Override
    public void execute() {
        vision.addVisionMeasurement(drivetrain);

        Pose2d currentPose = swerve.getState().Pose;
        Pose2d targetPose = AllianceUtil.getHubPose();

        Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double headingError = toTarget.getAngle().minus(currentPose.getRotation()).getRadians();
        double omega = headingError * ROT_KP;

        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(omega)
        );
        // Flywheel RPM is set live by shooterCommand.execute() from the same live distance.
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
}