package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ShotCalc;
import frc.robot.util.ShotLUT;
import frc.robot.Constants.visionConstants;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class shootOnTheMove extends Command {

    private final CommandXboxController joystick = new CommandXboxController(0);


    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final double ROT_KP = 2.5;

    private final Shooter shooterSub;
    private final StorageSub storageSub;
    private final Intake intakeSub;

    private final ShotCalc shotCalc;
    private final ShotLUT shotLUT;

    public shootOnTheMove(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooterSub,
        StorageSub storage,
        Intake intake,
        ShotLUT lut
    ) {
        this.swerve = drivetrain;
        this.shooterSub = shooterSub;
        this.storageSub = storage;
        this.intakeSub = intake;
        this.shotLUT = lut;

        // Initialize SOTM calculator with default config
        this.shotCalc = new ShotCalc();
        this.shotCalc.loadShotLUT(this.shotLUT);

        addRequirements(drivetrain, shooterSub, storage, intake);
    }

    @Override
    public void initialize() {
        shotCalc.resetWarmStart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getState().Pose;
        // Hub center position
        Translation2d hubCenter = isRed(currentPose)
                ? visionConstants.redHubPos.getTranslation()
                : visionConstants.hubPos.getTranslation();

        // Unit vector pointing from robot to hub
        Translation2d toHub = hubCenter.minus(currentPose.getTranslation());
        double mag = Math.sqrt(toHub.getX() * toHub.getX() + toHub.getY() * toHub.getY());
        Translation2d hubForward = mag > 1e-6
                ? new Translation2d(toHub.getX() / mag, toHub.getY() / mag)
                : new Translation2d(0, 0);

        Translation2d robotChassisSpeeds = new Translation2d(
           swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates).vxMetersPerSecond,
           swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates).vyMetersPerSecond 
        );

        Translation2d fieldChassisSpeeds = new Translation2d(
           swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates).vxMetersPerSecond,
           swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates).vyMetersPerSecond 
        ).rotateBy(swerve.getState().RawHeading);

        ShotCalc.ShotInputs inputs = new ShotCalc.ShotInputs(
            currentPose,
            robotChassisSpeeds,
            fieldChassisSpeeds,
            hubCenter,
            hubForward,
            1.0 // assume full vision confidence
        );

        ShotCalc.LaunchParameters params = shotCalc.calculate(inputs);

        if (params.isValid() && params.confidence() > 50) {
            shooterSub.setRPM(ShooterSide.MAIN, params.rpm());
            // Optionally set hood servo using LUT angle
            double hoodAngle = shotLUT.getAngle(params.solvedDistanceM());
            shooterSub.setServoAngle(ShooterSide.MAIN, hoodAngle);

            // Field-centric drive to maintain SOTM heading
            double headingError = params.driveAngle().getRadians() - currentPose.getRotation().getRadians();
            double omega = headingError * ROT_KP;

            swerve.setControl(
                drive.withVelocityX(joystick.getLeftY())
                     .withVelocityY(joystick.getLeftX())
                     .withRotationalRate(omega)
            );
        } else {
            // Stop rotation if shot invalid
            swerve.setControl(
                drive.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            );
        }

        // Run storage/intake as needed
        storageSub.runFloor(0.5);
        storageSub.runTop(0.5);
        intakeSub.runRollers(-2);
        intakeSub.setArmControl(-3);

    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );

        storageSub.runFloor(0);
        storageSub.runTop(0);
        intakeSub.runRollers(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isRed(Pose2d pose) {
        return pose.getX() > 8.25;
    }
}