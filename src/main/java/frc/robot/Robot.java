// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.drivesims.COTS;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public static Optional<Alliance> alliance = DriverStation.getAlliance();

    Orchestra orchestra = new Orchestra();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "mk5n test");
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new WPILOGWriter("logs"));
            Logger.addDataReceiver(new NT4Publisher());

        }

        Logger.start();

        orchestra.addInstrument(new TalonFX(20));
        orchestra.addInstrument(new TalonFX(21));
        orchestra.addInstrument(new TalonFX(22));
        orchestra.addInstrument(new TalonFX(23));
        orchestra.addInstrument(new TalonFX(24));
        orchestra.addInstrument(new TalonFX(25));
        orchestra.addInstrument(new TalonFX(27));

        orchestra.loadMusic("pacman.chrp");

        m_robotContainer = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {
        Logger.recordOutput("Test/RobotPeriodic", true);

        m_robotContainer.periodic();
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        
        if (RobotController.getUserButton()) {
            if (orchestra.isPlaying()) {
                orchestra.stop();
            } else {
                orchestra.loadMusic("pacman.chrp");
                orchestra.play();
            }
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand instanceof PathPlannerAuto ppAuto) {
            if (ppAuto.getStartingPose() != null) {
                m_robotContainer.drivetrain.resetPose(ppAuto.getStartingPose());
            }
        }

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        m_robotContainer.vision.addVisionMeasurement(m_robotContainer.drivetrain);
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        orchestra.stop();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.vision.addVisionMeasurement(m_robotContainer.drivetrain);
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance();
     
    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    public static boolean Real() {
        return isReal();
    }

}
