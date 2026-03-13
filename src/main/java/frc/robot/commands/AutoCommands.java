package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;


public class AutoCommands {

    public static Command shootAuto(CommandSwerveDrivetrain drivetrain, Shooter shooterSub, StorageSub ssub) {
        return new Command() {
            Timer t = new Timer();
            { t.start(); }
            @Override public void initialize() { t.reset(); }
            @Override public void execute() { new ShootAtHub(drivetrain, shooterSub, ssub); }
            @Override public void end(boolean i) { }
            @Override public boolean isFinished() { return t.hasElapsed(5); }
        };
    }

    public static Command extendAuto(Intake intake) {
        return new Command() {
            Timer t = new Timer();
            { t.start(); }
            @Override public void initialize() { t.reset(); }
            @Override public void execute() { new IntakeOut(intake); }
            @Override public void end(boolean i) { }
            @Override public boolean isFinished() { return t.hasElapsed(2); }
        };
    }

    public static Command startIntake(Intake intake, StorageSub storage) {
        return new Command() {
            private final Intake m_intake;
            private final StorageSub m_StorageSub;

            {this.m_intake = intake; this.m_StorageSub = storage; addRequirements(intake, storage);}
            @Override public void initialize() {}
            @Override public void execute() {m_intake.runRollers(-6); m_StorageSub.runGround(-8);}
            @Override public void end(boolean interrupted) {}
            @Override public boolean isFinished() {return true;}
        };    
    }

    public static Command stopIntake(Intake intake, StorageSub storage) {
        return new Command() {
            private final Intake m_intake;
            private final StorageSub m_StorageSub;

            {this.m_intake = intake; this.m_StorageSub = storage; addRequirements(intake, storage);}
            @Override public void initialize() {}
            @Override public void execute() {m_intake.runRollers(0); m_StorageSub.runGround(0);}
            @Override public void end(boolean interrupted) {}
            @Override public boolean isFinished() {return true;}
        };    
    }



}