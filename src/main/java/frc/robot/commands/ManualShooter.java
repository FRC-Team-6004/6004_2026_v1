package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.PathingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;
import frc.robot.util.ShooterLookup;

public class ManualShooter extends Command {

    private final Shooter shooter;
    private final StorageSub storage;
    private DoubleSupplier RPM;


    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ManualShooter(Shooter shooterSub, StorageSub ssub, DoubleSupplier rpm) {
        this.storage = ssub;
        this.shooter = shooterSub;
        this.RPM = rpm;
        addRequirements(shooterSub, ssub);
    }

    @Override
    public void execute() {

        shooter.setRPM(ShooterSide.LEFT, RPM.getAsDouble());
        shooter.setRPM(ShooterSide.RIGHT, RPM.getAsDouble());

        shooter.setServoAngle(ShooterSide.LEFT, .2);
        shooter.setServoAngle(ShooterSide.RIGHT, .2);

        storage.runNeo(12);
        storage.runGround(8);
    }

    @Override
    public void end(boolean interrupted) {
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
