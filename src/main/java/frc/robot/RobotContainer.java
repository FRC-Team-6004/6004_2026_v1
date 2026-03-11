// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PathingConstants;
import frc.robot.commands.ClimberSetPos1;
import frc.robot.commands.ClimberSetPos0;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.CustomPathing;
import frc.robot.commands.DoubleShooterLR;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeRollers;
import frc.robot.commands.ShooterLeftRun;
import frc.robot.commands.ShooterRightRun;
import frc.robot.commands.StorageRun;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.GridDistanceProcessing;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    private double MaxSpeed = .70 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController op = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Climber climber = new Climber();

    private final Shooter shooter = new Shooter();

    private final Intake intake = new Intake();

    private final StorageSub storageSub = new StorageSub();

    private final VisionSub vision = new VisionSub();

    private final GridDistanceProcessing gdp = new GridDistanceProcessing(
        PathingConstants.map,
        PathingConstants.flowX,
        PathingConstants.flowY,
        PathingConstants.MAP_LENGTH,
        PathingConstants.MAP_WIDTH);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        joystick.x().onTrue(Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getState().Pose)));

        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.rightBumper().whileTrue(new CustomPathing(drivetrain));

        joystick.povUp().onTrue(new ClimberUp(climber));
        joystick.povDown().onTrue(new ClimberDown(climber));

        joystick.povLeft().whileTrue(new IntakeIn(intake));
        joystick.povRight().whileTrue(new IntakeOut(intake));


        op.leftBumper().whileTrue(new IntakeRollers(intake));
        op.leftBumper().onTrue(new StorageRun(storageSub, 12));
        op.leftBumper().onFalse(new StorageRun(storageSub, 0));

        op.rightBumper().onTrue(new StorageRun(storageSub, -12));
        op.rightBumper().onFalse(new StorageRun(storageSub, 0));


        drivetrain.registerTelemetry(logger::telemeterize);

            
    }

    public void periodic() {
        vision.addVisionMeasurement(drivetrain);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
