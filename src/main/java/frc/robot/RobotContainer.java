// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathingConstants;
import frc.robot.Constants.visionConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CustomPathing;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShootAtHub;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.unjam;
import frc.robot.commands.shooterUnjam;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.GridDistanceProcessing;
import frc.robot.subsystems.shooter.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import frc.robot.util.NamedCommandManager;

import frc.robot.commands.ShootCommands;

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

    // private final Joystick manualJoystick = new Joystick(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final Climber climber = new Climber();

    private final Shooter shooter = new Shooter();

    private final Intake intake = new Intake();

    private final StorageSub storageSub = new StorageSub();

    public final VisionSub vision = new VisionSub();


    private final SendableChooser<Command> autoChooser;       

    private final GridDistanceProcessing gdp = new GridDistanceProcessing(
        PathingConstants.map,
        PathingConstants.flowX,
        PathingConstants.flowY,
        PathingConstants.MAP_LENGTH,
        PathingConstants.MAP_WIDTH);

    double speedDecay = .8;
    double maxN = speedDecay / (1 - speedDecay);

    public RobotContainer() {

        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(storageSub);
        CommandScheduler.getInstance().registerSubsystem(intake);

        NamedCommands.registerCommand("AutoShoot", AutoCommands.shootAuto(drivetrain, shooter, storageSub, intake));
        NamedCommands.registerCommand("intakeOut", AutoCommands.extendAuto(intake));
        NamedCommands.registerCommand("startIntake", AutoCommands.startIntake(intake, storageSub));
        NamedCommands.registerCommand("stopIntake", AutoCommands.stopIntake(intake, storageSub));

        NamedCommandManager.registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser("mid - shoot");  

        SmartDashboard.putData("Auto Mode", autoChooser);

        Logger.recordOutput("/Field/Blue hub", visionConstants.hubPos);
        Logger.recordOutput("/Field/Red hub", visionConstants.redHubPos);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xs * (1 / maxN) * 1 * MaxSpeed)
                    .withVelocityY(-ys * (1 / maxN) * 1 * MaxSpeed)
                    .withRotationalRate(-rs * .8 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        joystick.x().onTrue(Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getState().Pose)));

        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // joystick.rightBumper().whileTrue(new CustomPathing(drivetrain));


        joystick.povLeft().whileTrue(new IntakeIn(intake));
        joystick.povRight().whileTrue(new IntakeOut(intake));

        joystick.povUp().onTrue(Commands.runOnce(() -> intake.setArmControl(IntakeConstants.PIDin)));
        joystick.povDown().onTrue(Commands.runOnce(() -> intake.setArmControl(IntakeConstants.PIDout)));


        op.leftBumper().whileTrue(new intakeCommand(intake, storageSub));

        op.rightBumper().whileTrue(new shooterUnjam(shooter));
        op.rightBumper().whileTrue(new unjam(storageSub));

        op.rightTrigger(0.05).whileTrue(new ShootAtHub(drivetrain, shooter, storageSub, intake));
        op.leftTrigger(0.05).whileTrue(ShootCommands.createShootCommand(shooter, storageSub, intake, 4500, 0.5));

        op.b().whileTrue(ShootCommands.createShootCommand(shooter, storageSub, intake, 3000, 0.0));
        op.a().whileTrue(ShootCommands.createShootCommand(shooter, storageSub, intake, 3500, 0.0));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    double xs = 0;
    double ys = 0;
    double rs = 0;

    public void periodic() { 

        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d targetPose = isRed() ? visionConstants.redHubPos : visionConstants.hubPos;
        Translation2d robotTranslation = currentPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();
        // Vector from robot → target
        Translation2d toTarget = targetTranslation.minus(robotTranslation);
        double distance = robotTranslation.getDistance(targetTranslation);
        Shooter.distance = distance;

        int mode = 1;

        //mode 1: slippery profile
        //mode default: reg

        switch(mode) {
            case 1 : 
                xs += Math.pow(joystick.getLeftY(), 3);
                ys += Math.pow(joystick.getLeftX(), 3);
                xs *= speedDecay;
                ys *= speedDecay;
                rs = Math.pow(joystick.getRightX(), 3);
                break;
            default : 
                xs = Math.pow(joystick.getLeftY(), 3) * maxN;
                ys = Math.pow(joystick.getLeftX(), 3) * maxN;
                rs = Math.pow(joystick.getRightX(), 3);
                break;
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return null;
    }

    private boolean isRed() {
        return (drivetrain.getState().Pose.getX() > 8.25);
    }

}
