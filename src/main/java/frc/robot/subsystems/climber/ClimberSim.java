package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;


public class ClimberSim implements ClimberIO {

    private final ElevatorSim elevatorSim;
    private final PIDController pid;

    // Spark MAX sim objects
    private final SparkMaxSim sparkSim;
    private final SparkRelativeEncoderSim encoderSim;
    private final SparkMax motor = new SparkMax(ClimberConstants.kMotorID, MotorType.kBrushless);



    public ClimberSim() {
        // Physics-based elevator sim
        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(1),
            ClimberConstants.kGearing,
            ClimberConstants.kCarriageMass,
            ClimberConstants.kDrumRadius,
            ClimberConstants.kMinHeight,
            ClimberConstants.kMaxHeight,
            true,
            0,
            0.01,
            0.0
        );

        // PID for position control
        pid = new PIDController(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD
        );
        pid.setTolerance(0.005);

        // Spark MAX simulation
        sparkSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
        encoderSim = new SparkRelativeEncoderSim(motor);
    }

    @Override
    public void run(double speed) {
        // Direct percent output for teleop control
        double appliedOutput = MathUtil.clamp(speed, -1.0, 1.0);
        sparkSim.setBusVoltage(RoboRioSim.getVInVoltage());
        sparkSim.setAppliedOutput(appliedOutput);
    }

    @Override
    public Trigger atHeight(double heightMeters, double toleranceMeters) {
        return new Trigger(() ->
            MathUtil.isNear(heightMeters, getHeightMeters(), toleranceMeters)
        );
    }

    @Override
    public double getHeight() {
        return getHeightMeters();
    }

    @Override
    public double getHeightMeters() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public double getVelocityMetersPerSec() {
        return elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    public void setPosition(double targetHeightMeters) {
        // PID output in volts
        double pidVolts = pid.calculate(getHeightMeters(), targetHeightMeters) * 12.0;

        double outputVolts = pidVolts;

        // Prevent motion past bounds, but allow reversing
        if (getHeightMeters() <= ClimberConstants.kMinHeight && outputVolts < 0) {
            outputVolts = 0;
        }
        if (getHeightMeters() >= ClimberConstants.kMaxHeight && outputVolts > 0) {
            outputVolts = 0;
        }

        // Apply voltage to SparkSim
        sparkSim.setBusVoltage(RoboRioSim.getVInVoltage());
        sparkSim.setAppliedOutput(outputVolts / RoboRioSim.getVInVoltage());
    }

    @Override
    public void periodic() {
        // Step the elevator sim using actual voltage from SparkSim
        double voltage = sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
        elevatorSim.setInputVoltage(voltage);

        // Adjust physics for current load
        elevatorSim.update(0.02);

        // Update encoder sim
        double rotations = elevatorSim.getPositionMeters() / (2 * Math.PI * ClimberConstants.kDrumRadius);
        encoderSim.setPosition(rotations);
        encoderSim.setVelocity(elevatorSim.getVelocityMetersPerSecond() / (2 * Math.PI * ClimberConstants.kDrumRadius));

        // Update battery simulation
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

}
