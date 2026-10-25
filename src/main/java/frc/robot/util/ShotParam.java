package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

/** Ballistic parameters at a single distance: RPM and time-of-flight. The hood is static, so there is no hood angle. */
public record ShotParam(double rpm, double tofSec) {

    /** All zeros. Safe default. */
    public static final ShotParam ZERO = new ShotParam(0, 0);

    /** Linear interpolator for use in InterpolatingTreeMap. */
    public static Interpolator<ShotParam> interpolator() {
        return (start, end, t) ->
            new ShotParam(
                MathUtil.interpolate(start.rpm, end.rpm, t),
                MathUtil.interpolate(start.tofSec, end.tofSec, t));
    }
}
