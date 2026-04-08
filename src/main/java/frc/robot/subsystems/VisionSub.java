package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSub extends SubsystemBase {

    private AprilTagFieldLayout fieldLayout;

    /* Cameras */
    private final PhotonCamera hubFwdCam = new PhotonCamera("hubFwdCam");
    private final PhotonCamera BLSwerve = new PhotonCamera("BLSwerve");
    private final PhotonCamera BRSwerve = new PhotonCamera("BRSwerve");

    /* Robot -> camera transforms */
    private final Transform3d robotToHubFwd = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(3), Units.inchesToMeters(20)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(19), Math.toRadians(0)));

    private final Transform3d robotToBLSwerve = new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.68), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.875)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(28), Math.toRadians(180 - 22.5)));

    private final Transform3d robotToBRSwerve = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.68), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.875)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(180 + 22.5)));

    /* Pose estimators */
    private PhotonPoseEstimator hubFwdEstimator;
    private PhotonPoseEstimator BLSwerveEstimator;
    private PhotonPoseEstimator BRSwerveEstimator;


    public VisionSub() {

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource("/frc/robot/2026AprilTags.json");
        } catch (IOException e) {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            // throw new RuntimeException("Failed to load AprilTag layout", e);
        }

        hubFwdEstimator = new PhotonPoseEstimator(fieldLayout, robotToHubFwd);
        BLSwerveEstimator = new PhotonPoseEstimator(fieldLayout, robotToBLSwerve);
        BRSwerveEstimator = new PhotonPoseEstimator(fieldLayout, robotToBRSwerve);

    }

    /* -------- Internal estimation -------- */

    private Optional<EstimatedRobotPose> getEstimate(PhotonCamera cam, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> result = Optional.empty();

        for (var frame : cam.getAllUnreadResults()) {
            result = estimator.estimateCoprocMultiTagPose(frame);
            if (result.isEmpty()) {
                result = estimator.estimateLowestAmbiguityPose(frame);
            }
        }

        return result;
    }

    private List<EstimatedRobotPose> getAllEstimates() {

        List<EstimatedRobotPose> poses = new ArrayList<>();

        getEstimate(hubFwdCam, hubFwdEstimator).ifPresent(poses::add);
        getEstimate(BLSwerve, BLSwerveEstimator).ifPresent(poses::add);
        getEstimate(BRSwerve, BRSwerveEstimator).ifPresent(poses::add);

        return poses;
    }

    /* -------- Pose methods -------- */

    public Optional<Pose3d> getBestPose() {

        List<EstimatedRobotPose> estimates = getAllEstimates();

        EstimatedRobotPose best = null;
        double bestAmbiguity = Double.MAX_VALUE;

        for (EstimatedRobotPose pose : estimates) {

            double ambiguity = pose.targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average()
                    .orElse(1.0);

            if (ambiguity < bestAmbiguity) {
                bestAmbiguity = ambiguity;
                best = pose;
            }
        }

        return best == null ? Optional.empty() : Optional.of(best.estimatedPose);
    }

    public Optional<EstimatedRobotPose> getBestEstimate() {

        List<EstimatedRobotPose> estimates = getAllEstimates();

        EstimatedRobotPose best = null;
        double bestAmbiguity = Double.MAX_VALUE;

        for (EstimatedRobotPose pose : estimates) {

            double ambiguity = pose.targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average()
                    .orElse(1.0);

            if (ambiguity < bestAmbiguity) {
                bestAmbiguity = ambiguity;
                best = pose;
            }
        }

        return Optional.ofNullable(best);
    }

    /* -------- Swerve Pose Estimator Integration -------- */

    public void addVisionMeasurement(CommandSwerveDrivetrain drivetrain) {

        Optional<EstimatedRobotPose> estimate = getEstimate(hubFwdCam, hubFwdEstimator);
            if (!estimate.isEmpty()) {
                EstimatedRobotPose visionPose = estimate.get();
                drivetrain.addVisionMeasurement(
                    visionPose.estimatedPose.toPose2d(),
                    visionPose.timestampSeconds
                );
            }

        
        // estimate = getEstimate(BLSwerve, BLSwerveEstimator);
        //     if (!estimate.isEmpty()) {
        //         EstimatedRobotPose visionPose = estimate.get();
        //         drivetrain.addVisionMeasurement(
        //             visionPose.estimatedPose.toPose2d(),
        //             visionPose.timestampSeconds
        //         );
        //     }


        
        // estimate = getEstimate(BRSwerve, BRSwerveEstimator);
        //     if (!estimate.isEmpty()) {
        //         EstimatedRobotPose visionPose = estimate.get();
        //         drivetrain.addVisionMeasurement(
        //             visionPose.estimatedPose.toPose2d(),
        //             visionPose.timestampSeconds
        //         );
        //     }

    }

    @Override
    public void periodic() {
    }

}