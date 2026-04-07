package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.visionConstants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.Timer;

public class shootOnTheMove extends Command {

    private final Timer t = new Timer();

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double ROT_KP = 5;

    private static final double TOTAL_LATENCY_SEC = -0.5;

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
        t.start();

        this.swerve = drivetrain;
        this.shooterSub = shooterSub;
        this.storageSub = storage;
        this.intakeSub = intake;
        this.shotLUT = lut;

        this.shotCalc = new ShotCalc();
        this.shotCalc.loadShotLUT(this.shotLUT);

        addRequirements(drivetrain, shooterSub, storage, intake);
    }

    @Override
    public void initialize() {
        shotCalc.resetWarmStart();
        t.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getState().Pose;

        // Select correct hub
        Translation2d hubCenter = isRed(currentPose)
                ? visionConstants.redHubPos.getTranslation()
                : visionConstants.hubPos.getTranslation();

        // Direction to hub
        Translation2d toHub = hubCenter.minus(currentPose.getTranslation());
        double mag = toHub.getNorm();
        Translation2d hubForward = mag > 1e-6
                ? toHub.div(mag)
                : new Translation2d();

        var chassisSpeeds = swerve.getKinematics()
                .toChassisSpeeds(swerve.getState().ModuleStates);

        Translation2d robotChassisSpeeds = new Translation2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond
        );

        // Convert to field-relative
        Translation2d fieldChassisSpeeds = robotChassisSpeeds
                .rotateBy(swerve.getState().RawHeading);

        // Shot calculation
        ShotCalc.ShotInputs inputs = new ShotCalc.ShotInputs(
            currentPose,
            robotChassisSpeeds,
            fieldChassisSpeeds,
            hubCenter,
            hubForward,
            1.0
        );

        ShotCalc.LaunchParameters params = shotCalc.calculate(inputs);

        if (params.isValid() && params.confidence() > 50) {

            shooterSub.setRPM(ShooterSide.MAIN, params.rpm());

            double hoodAngle = shotLUT.getAngle(params.solvedDistanceM());
            shooterSub.setServoAngle(ShooterSide.MAIN, hoodAngle);


            // Predict robot motion during latency
            Translation2d motionOffset = fieldChassisSpeeds.times(TOTAL_LATENCY_SEC);

            // Adjusted aim point
            Translation2d adjustedToHub = hubCenter.minus(
                currentPose.getTranslation().plus(motionOffset)
            );

            Logger.recordOutput("/shootOnTheMove/AdjustedToHub", currentPose.plus(new Transform2d(motionOffset, currentPose.getRotation())));

            double targetAngle = Math.atan2(
                adjustedToHub.getY(),
                adjustedToHub.getX()
            );

            double currentAngle = currentPose.getRotation().getRadians();

            double headingError = targetAngle - currentAngle;

            headingError = Math.atan2(
                Math.sin(headingError),
                Math.cos(headingError)
            );

            double omega = headingError * ROT_KP;

            swerve.setControl(
                drive.withVelocityX(joystick.getLeftY())
                     .withVelocityY(joystick.getLeftX())
                     .withRotationalRate(omega)
            );

        } else {
            swerve.setControl(
                drive.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            );
        }
        if (t.get() > 0.5) {
            // Run mechanisms
            storageSub.runFloor(2);
            storageSub.runTop(2);
            intakeSub.runRollers(-2);
            // intakeSub.setArmControl(-3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
        shooterSub.setRPM(ShooterSide.MAIN, 0);
        storageSub.runFloor(0);
        storageSub.runTop(0);
        intakeSub.runRollers(0);
        intakeSub.setArmControl(IntakeConstants.PIDout);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isRed(Pose2d pose) {
        return pose.getX() > 8.25;
    }
}