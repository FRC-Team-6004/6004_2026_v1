package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;


/**
 * Shoot-On-The-Move calculator.
 * Solves for RPM, TOF, and drive angle while robot is moving.
 */
public class ShotCalc {

    private ShotLUT lut;

    /** Load a LUT for angle/RPM lookup. */
    public void loadShotLUT(ShotLUT lut) {
        this.lut = lut;
    }

    /** Reset warm start for iterative solver (optional). */
    public void resetWarmStart() {
        // Clear cached previous solution
    }

    /** Inputs for a shot calculation. */
    public static record ShotInputs(
        Pose2d robotPose,
        Translation2d fieldVelocity,
        Translation2d robotVelocity,
        Translation2d targetPos,
        Translation2d targetForward,
        double visionConfidence
    ) {}

    /** Calculated shot parameters. */
    public static record LaunchParameters(
        double rpm,
        double tofSec,
        Rotation2d driveAngle,
        double solvedDistanceM,
        double confidence
    ) {
        public boolean isValid() { return rpm > 0; }
        public double confidence() { return confidence; }
    }

    /** Solve for the shot given inputs. */
    public LaunchParameters calculate(ShotInputs inputs) {
        double distance = inputs.robotPose.getTranslation().getDistance(inputs.targetPos);
        ShotParam params = lut.get(distance * .95);

        double rpm = params.rpm();
        double tof = params.tofSec();
        Rotation2d driveAngle = inputs.targetPos.minus(inputs.robotPose.getTranslation()).getAngle();

        double confidence = 100; // placeholder, could be vision confidence based

        return new LaunchParameters(rpm, tof, driveAngle, distance, confidence);
    }
}