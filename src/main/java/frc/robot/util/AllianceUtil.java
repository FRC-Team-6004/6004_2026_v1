package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.visionConstants;

/** Single source of truth for alliance-dependent decisions. Sampled live from the DriverStation, never from field position. */
public final class AllianceUtil {
    private AllianceUtil() {}

    /** True when the DriverStation reports the Red alliance. Defaults to Blue when unknown (matches PathPlanner's default). */
    public static boolean isRed() {
        return DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
    }

    /** The hub this alliance shoots at. */
    public static Pose2d getHubPose() {
        return isRed() ? visionConstants.redHubPos : visionConstants.hubPos;
    }
}
