package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.util.ShooterLookup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

public class ShootToHub extends Command {

    private final ShooterLookup trajHelper;
    private final CommandSwerveDrivetrain swerve;
    private final Shooter shooter;

    // Replace with real field position of hub center
    private final Translation2d HUB_LOCATION = new Translation2d(8.23, 4.11);

    // Shooter constants (CHANGE THESE)
    private static final double WHEEL_RADIUS_METERS = 0.0508; // 2in wheel
    private static final double SHOOTER_LATENCY = 0.15; // tuned
    private static final double MAX_EXIT_VELOCITY = 20.0; // m/s mechanical limit

    private final FieldCentricFacingAngle drive =
        new FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ShootToHub(CommandSwerveDrivetrain drivetrain, Shooter shooterSub) {
        this.trajHelper = new ShooterLookup();
        this.swerve = drivetrain;
        this.shooter = shooterSub;

        addRequirements(drivetrain, shooterSub);
    }

    @Override
    public void execute() {

        /* ---------------------------------------------------------- */
        /* 1. LATENCY COMPENSATION                                   */
        /* ---------------------------------------------------------- */

        Translation2d robotPos = swerve.getState().Pose.getTranslation();
        Translation2d robotVel = new Translation2d(
            swerve.getState().Speeds.vxMetersPerSecond,
            swerve.getState().Speeds.vyMetersPerSecond
        );

        Translation2d futurePos = robotPos.plus(robotVel.times(SHOOTER_LATENCY));

        /* ---------------------------------------------------------- */
        /* 2. TARGET VECTOR                                           */
        /* ---------------------------------------------------------- */

        Translation2d targetVec = HUB_LOCATION.minus(futurePos);
        double distance = targetVec.getNorm();

        if (distance < 0.01) return;

        Translation2d unitToTarget = targetVec.div(distance);

        /* ---------------------------------------------------------- */
        /* 3. GET BASE TRAJECTORY (STATIONARY ROBOT)                 */
        /* ---------------------------------------------------------- */

        double hoodAngleDeg = 0; //trajHelper.getHoodAngleFor(distance);
        double hoodAngleRad = Math.toRadians(hoodAngleDeg);

        // This must return EXIT VELOCITY in m/s
        double baseExitVelocity = 0;
            //trajHelper.getExitVelocityFor(distance, hoodAngleDeg);

        /* ---------------------------------------------------------- */
        /* 4. ROBOT MOTION COMPENSATION                               */
        /* ---------------------------------------------------------- */

        // Required horizontal component of shot
        double horizontalComponent =
            baseExitVelocity * Math.cos(hoodAngleRad);

        Translation2d desiredHorizontalVec =
            unitToTarget.times(horizontalComponent);

        // Subtract robot velocity (field-relative)
        Translation2d compensatedVec =
            desiredHorizontalVec.minus(robotVel);

        double compensatedHorizontalSpeed =
            compensatedVec.getNorm();

        /* ---------------------------------------------------------- */
        /* 5. RECOMPUTE TOTAL EXIT VELOCITY                           */
        /* ---------------------------------------------------------- */

        double newExitVelocity =
            compensatedHorizontalSpeed / Math.cos(hoodAngleRad);

        newExitVelocity = Math.min(newExitVelocity, MAX_EXIT_VELOCITY);

        /* ---------------------------------------------------------- */
        /* 6. AIM DRIVETRAIN                                          */
        /* ---------------------------------------------------------- */

        Rotation2d targetAngle =
            compensatedVec.getAngle();

        swerve.setControl(
            drive.withTargetDirection(targetAngle)
        );

        /* ---------------------------------------------------------- */
        /* 7. CONVERT TO RPM AND FIRE                                 */
        /* ---------------------------------------------------------- */

        double newRPM = 0; //trajHelper.getRPMFor(distance);


        shooter.setRPM(ShooterSide.LEFT, newRPM);
        shooter.setRPM(ShooterSide.RIGHT, newRPM);

        // If using separate hood subsystem:
        // hood.setAngle(hoodAngleDeg);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}