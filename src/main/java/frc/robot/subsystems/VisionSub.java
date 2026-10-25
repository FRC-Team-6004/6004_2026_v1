package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

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
            new Rotation3d(Math.toRadians(0), Math.toRadians(27.5), Math.toRadians(0)));

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
            // LOUD failure, not a silent one: falling back to the default field layout means every
            // vision pose is computed against the wrong year's field. If you see this error, the
            // 2026 layout resource is missing from the deploy, and vision should not be trusted.
            DriverStation.reportError(
                "VisionSub: failed to load 2026AprilTags.json, falling back to default field layout. Vision poses may be WRONG: "
                    + e.getMessage(),
                false);
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
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

    /**
     * Single-tag estimates can be ambiguous (two possible poses for one tag). Reject poor ones;
     * multi-tag estimates are geometrically unambiguous, so they are always accepted. Tune this
     * if vision stops contributing (too strict) or the pose ever jumps (too loose).
     */
    private static final double SINGLE_TAG_MAX_AMBIGUITY = 0.2;

    /**
     * Fuses all three cameras into the drivetrain pose estimator for full-field localization:
     * the forward hub camera plus the two rear swerve cameras. Each camera's latest estimate is
     * an independent measurement, so the estimator fuses all of them rather than picking one.
     * Called every robot loop from RobotContainer.periodic(), so this runs in teleop too.
     */
    public void addVisionMeasurement(CommandSwerveDrivetrain drivetrain) {
        addEstimateIfValid(drivetrain, "HubFwd", getEstimate(hubFwdCam, hubFwdEstimator));
        addEstimateIfValid(drivetrain, "BLSwerve", getEstimate(BLSwerve, BLSwerveEstimator));
        addEstimateIfValid(drivetrain, "BRSwerve", getEstimate(BRSwerve, BRSwerveEstimator));
    }

    private void addEstimateIfValid(
            CommandSwerveDrivetrain drivetrain, String cameraName, Optional<EstimatedRobotPose> estimate) {
        if (estimate.isEmpty()) {
            return;
        }

        EstimatedRobotPose visionPose = estimate.get();

        if (visionPose.targetsUsed.size() == 1) {
            double ambiguity = visionPose.targetsUsed.get(0).getPoseAmbiguity();
            if (ambiguity > SINGLE_TAG_MAX_AMBIGUITY) {
                Logger.recordOutput("Vision/" + cameraName + "/RejectedAmbiguity", ambiguity);
                return;
            }
        }

        drivetrain.addVisionMeasurement(
            visionPose.estimatedPose.toPose2d(),
            visionPose.timestampSeconds
        );
        Logger.recordOutput("Vision/" + cameraName + "/Pose", visionPose.estimatedPose.toPose2d());
        Logger.recordOutput("Vision/" + cameraName + "/TargetsUsed", (double) visionPose.targetsUsed.size());
    }

    @Override
    public void periodic() {
    }

}