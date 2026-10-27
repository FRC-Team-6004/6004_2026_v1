package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

    /* Robot -> camera transforms; pitches measured from tape-measure photos 2026-10-27 (negative pitch slot = up) */
    private final Transform3d robotToHubFwd = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(3), Units.inchesToMeters(20)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(0)));

    private final Transform3d robotToBLSwerve = new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.68), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.875)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-18), Math.toRadians(180 - 22.5)));

    private final Transform3d robotToBRSwerve = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.68), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.875)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-18), Math.toRadians(180 + 22.5)));

    /* Pose estimators */
    private PhotonPoseEstimator hubFwdEstimator;
    private PhotonPoseEstimator BLSwerveEstimator;
    private PhotonPoseEstimator BRSwerveEstimator;

    /* -------- Simulation: what the cameras would see, from ground truth -------- */
    // Only constructed in sim (RobotBase.isSimulation()). The sim cameras feed the SAME
    // PhotonCamera objects the real code reads, so estimation, fusion, and aiming code
    // under test is byte-identical to the real path. Nothing in this block runs on the
    // robot.
    private VisionSystemSim visionSim;


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

        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(fieldLayout);

            // Placeholder camera model until the real cameras' specs are known: 640x480,
            // 100 deg diagonal FOV, 20 FPS, 35 ms mean latency. Tune to match the hardware.
            SimCameraProperties cameraProps = new SimCameraProperties();
            cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraProps.setCalibError(0.25, 0.08);
            cameraProps.setFPS(20);
            cameraProps.setAvgLatencyMs(35);
            cameraProps.setLatencyStdDevMs(5);

            visionSim.addCamera(new PhotonCameraSim(hubFwdCam, cameraProps), robotToHubFwd);
            visionSim.addCamera(new PhotonCameraSim(BLSwerve, cameraProps), robotToBLSwerve);
            visionSim.addCamera(new PhotonCameraSim(BRSwerve, cameraProps), robotToBRSwerve);
        }

    }

    /**
     * Steps the simulated cameras from the ground-truth robot pose. Call once per loop
     * from Robot.simulationPeriodic (via RobotContainer). No-op on the real robot and
     * when sim was not constructed.
     */
    public void updateSim(Pose2d groundTruthPose) {
        if (visionSim != null) {
            visionSim.update(groundTruthPose);
        }
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
     * Single-tag ambiguity handling, ported from Lynk Robotics' 2026 vision code.
     * A single tag gives two plausible robot poses (best and alternate). Instead of
     * one hard gate, this uses two tiers:
     * <ul>
     *   <li>ambiguity at or below AUTO_ACCEPT_AMBIGUITY: accept the best pose outright;</li>
     *   <li>ambiguity between AUTO_ACCEPT_AMBIGUITY and SINGLE_TAG_MAX_AMBIGUITY: pick
     *       whichever candidate heading (best or alternate) agrees better with the
     *       estimator heading, which is gyro-dominated (see VISION_STDDEVS);</li>
     *   <li>ambiguity above SINGLE_TAG_MAX_AMBIGUITY: reject the sighting.</li>
     * </ul>
     * Note this deliberately loosens the old 0.2 ceiling to 0.35: the gyro pick is what
     * makes the wider band safe, where the old gate was guarding against a blind
     * best-pose choice. If pose jumps ever appear, tighten SINGLE_TAG_MAX_AMBIGUITY
     * back down before touching anything else. Multi-tag estimates are geometrically
     * unambiguous, so they are always accepted.
     */
    private static final double AUTO_ACCEPT_AMBIGUITY = 0.1;
    private static final double SINGLE_TAG_MAX_AMBIGUITY = 0.35;

    /**
     * Vision measurement trust, ported from Lynk: position trust stays at the
     * WPILib default (0.9 m), while heading trust is deliberately weak (2.0 rad),
     * so the estimator's heading stays gyro-dominated and vision corrects mainly
     * position. Tradeoff, stated plainly: if the gyro yaw itself is wrong (bad zero,
     * drift), vision will barely fix heading — gyro alignment becomes critical.
     * This applies to multi-tag measurements too, matching Lynk; a tighter angular
     * value for multi-tag only is a possible later refinement.
     */
    private static final Matrix<N3, N1> VISION_STDDEVS = VecBuilder.fill(0.9, 0.9, 2.0);

    /**
     * Fuses all three cameras into the drivetrain pose estimator for full-field localization:
     * the forward hub camera plus the two rear swerve cameras. Each camera's unread frames
     * are processed (multi-tag via the coprocessor PNP estimate, single-tag via the
     * gyro-based disambiguation below) and every resulting measurement is fed to the
     * estimator. Called every robot loop from RobotContainer.periodic(), so this runs
     * in teleop too.
     */
    public void addVisionMeasurement(CommandSwerveDrivetrain drivetrain) {
        // Estimator heading: with VISION_STDDEVS above this is gyro-dominated, which is
        // exactly what the single-tag alternate pick below wants to compare against.
        Rotation2d estimatorHeading = drivetrain.getState().Pose.getRotation();
        addCameraMeasurements(drivetrain, "HubFwd", hubFwdCam, hubFwdEstimator, robotToHubFwd, estimatorHeading);
        addCameraMeasurements(drivetrain, "BLSwerve", BLSwerve, BLSwerveEstimator, robotToBLSwerve, estimatorHeading);
        addCameraMeasurements(drivetrain, "BRSwerve", BRSwerve, BRSwerveEstimator, robotToBRSwerve, estimatorHeading);
    }

    private void addCameraMeasurements(
            CommandSwerveDrivetrain drivetrain,
            String cameraName,
            PhotonCamera cam,
            PhotonPoseEstimator estimator,
            Transform3d robotToCamera,
            Rotation2d estimatorHeading) {
        for (var frame : cam.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> multiTag = estimator.estimateCoprocMultiTagPose(frame);
            if (multiTag.isPresent()) {
                EstimatedRobotPose visionPose = multiTag.get();
                feedMeasurement(drivetrain, cameraName, visionPose.estimatedPose.toPose2d(),
                        visionPose.timestampSeconds, visionPose.targetsUsed.size(), false);
                continue;
            }
            processSingleTagFrame(drivetrain, cameraName, frame, robotToCamera, estimatorHeading);
        }
    }

    private void processSingleTagFrame(
            CommandSwerveDrivetrain drivetrain,
            String cameraName,
            PhotonPipelineResult frame,
            Transform3d robotToCamera,
            Rotation2d estimatorHeading) {
        if (frame.targets.isEmpty()) {
            return;
        }

        // Lowest-ambiguity target in the frame, matching the old estimateLowestAmbiguityPose behavior.
        PhotonTrackedTarget target = null;
        for (PhotonTrackedTarget candidate : frame.targets) {
            if (target == null || candidate.getPoseAmbiguity() < target.getPoseAmbiguity()) {
                target = candidate;
            }
        }

        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity > SINGLE_TAG_MAX_AMBIGUITY) {
            Logger.recordOutput("Vision/" + cameraName + "/RejectedAmbiguity", ambiguity);
            return;
        }

        Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) {
            Logger.recordOutput("Vision/" + cameraName + "/RejectedAmbiguity", ambiguity);
            return;
        }

        Pose3d bestPose = tagPose.get()
                .plus(target.getBestCameraToTarget().inverse())
                .plus(robotToCamera.inverse());

        boolean altPoseUsed = false;
        Pose3d chosenPose = bestPose;
        if (ambiguity > AUTO_ACCEPT_AMBIGUITY) {
            // Gray zone: both single-tag solutions are plausible, so pick the one whose
            // heading agrees better with the estimator (gyro-dominated) heading.
            Pose3d altPose = tagPose.get()
                    .plus(target.altCameraToTarget.inverse())
                    .plus(robotToCamera.inverse());
            double bestHeadingError = Math.abs(
                    bestPose.getRotation().toRotation2d().minus(estimatorHeading).getRadians());
            double altHeadingError = Math.abs(
                    altPose.getRotation().toRotation2d().minus(estimatorHeading).getRadians());
            if (altHeadingError < bestHeadingError) {
                chosenPose = altPose;
                altPoseUsed = true;
            }
        }

        feedMeasurement(drivetrain, cameraName, chosenPose.toPose2d(),
                frame.getTimestampSeconds(), 1, altPoseUsed);
    }

    private void feedMeasurement(
            CommandSwerveDrivetrain drivetrain,
            String cameraName,
            Pose2d pose,
            double timestampSeconds,
            int targetsUsed,
            boolean altPoseUsed) {
        drivetrain.addVisionMeasurement(pose, timestampSeconds, VISION_STDDEVS);
        Logger.recordOutput("Vision/" + cameraName + "/Pose", pose);
        Logger.recordOutput("Vision/" + cameraName + "/TargetsUsed", (double) targetsUsed);
        Logger.recordOutput("Vision/" + cameraName + "/AltPoseUsed", altPoseUsed);
    }

    @Override
    public void periodic() {
    }

}
