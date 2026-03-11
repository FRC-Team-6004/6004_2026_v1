package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.util.GridDistanceProcessing;
import frc.robot.Constants.PathingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CustomPathing extends Command {

    private final GridDistanceProcessing gdp =
        new GridDistanceProcessing(
            PathingConstants.map,
            PathingConstants.flowX,
            PathingConstants.flowY,
            PathingConstants.MAP_LENGTH,
            PathingConstants.MAP_WIDTH
        );

    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double MAX_SPEED = 4.5;
    private static final double MIN_SPEED = 1.6;
    private static final double ROT_KP    = 3.5;
    private double vx = 0;
    private double vy = 0;
    double speedDecay = .7;
    double maxVectorMagnitude = speedDecay / (1 - speedDecay);
    double speedModifier = (1 / maxVectorMagnitude);

    public CustomPathing(CommandSwerveDrivetrain drivetrain) {
        this.swerve = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getState().Pose;

        int heat = gdp.heatAt(currentPose);
        
        /* ---------- Flow vector ---------- */

        double[] flow = gdp.flowVector(currentPose);
        double dirX = flow[0];
        double dirY = flow[1];

        /* ---------- Speed from heat ---------- */

        double speed = Math.min(MAX_SPEED,
                        Math.max(MIN_SPEED, heat * 0.08));

        vx += dirX * -speed;
        vy += dirY * -speed;
        vx *= speedDecay;
        vy *= speedDecay;

        /* ---------- Face direction of travel ---------- */

        Rotation2d desiredHeading = new Rotation2d(dirX, dirY);
        double headingError =
            desiredHeading.minus(currentPose.getRotation()).getRadians();

        double omega = headingError * ROT_KP;
        swerve.setControl(
            drive.withVelocityX(vx * -speedModifier)
                 .withVelocityY(vy * -speedModifier)
                 .withRotationalRate(omega)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
        vx = 0;
        vy = 0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
