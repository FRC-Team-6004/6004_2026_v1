package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final ClimberIO io;

    private final Mechanism2d mech;
    private final MechanismLigament2d elevator;

    public Climber() {
        io = RobotBase.isReal() ? new ClimberReal() : new ClimberSim();

        mech = new Mechanism2d(2.0, 2.0);

        MechanismRoot2d root =
            mech.getRoot("ClimberRoot", 1.0, 0.1);

        elevator = root.append(
            new MechanismLigament2d(
                "Elevator",
                0.1,           // initial length
                90.0,          // vertical
                6.0,
                new Color8Bit(255, 255, 0)
            )
        );

        SmartDashboard.putData("Climber", mech);
    }

    public void run(double speed) {
        io.run(speed);
    }

    public void setPosition(double heightMeters) {
        io.setPosition(heightMeters);
    }

    public double getHeightMeters() {
        return io.getHeightMeters();
    }

    public double getVelocityMetersPerSec() {
        return io.getVelocityMetersPerSec();
    }


    @Override
    public void periodic() {
        io.periodic();

        // Clamp to prevent Mechanism2d inversion
        //double height = Math.max(0.0, getHeightMeters());
        
        //elevator.setLength(height);
    }
}
