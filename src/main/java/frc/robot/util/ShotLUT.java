package frc.robot.util;

/** Distance-keyed lookup table for shot parameters. The hood is static, so entries carry RPM and time-of-flight only. */
public class ShotLUT {

    private final edu.wpi.first.math.interpolation.InterpolatingTreeMap<Double, ShotParam> map;
    private int entryCount = 0;

    public ShotLUT() {
        map = new edu.wpi.first.math.interpolation.InterpolatingTreeMap<>(
                edu.wpi.first.math.interpolation.InverseInterpolator.forDouble(),
                ShotParam.interpolator());

        // Fill LUT with ShooterLookup values + realistic TOF
        put(0.914781, 2650, 1.5);
        put(1.158665, 2750, 1.55);
        put(1.323822, 2800, 1.6);
        put(1.5731,   3000, 1.65);
        put(1.70744,  3100, 1.7);
        put(2.065012, 3250, 1.8);
        put(2.342901, 3375, 1.9);
        put(2.56123,  3500, 2.0);
        put(3.0445,   3500, 2.1);
        put(3.356,    3550, 2.2);
        put(3.62073,  3575, 2.3);
        put(3.837,    3650, 2.4);
        put(4.07965,  3700, 2.5);
        put(4.51245,  3775, 2.65);
        put(4.8312,   3850, 2.8);
    }

    /** Insert or overwrite parameters at a given distance. */
    public void put(double distanceM, double rpm, double tofSec) {
        put(distanceM, new ShotParam(rpm, tofSec));
    }

    public void put(double distanceM, ShotParam params) {
        map.put(distanceM, params);
        entryCount++;
    }

    /** Interpolated parameters at a distance; returns ZERO if empty. */
    public ShotParam get(double distanceM) {
        ShotParam result = map.get(distanceM);
        return result != null ? result : ShotParam.ZERO;
    }

    public double getRPM(double distanceM) { return get(distanceM).rpm(); }
    public double getTOF(double distanceM) { return get(distanceM).tofSec(); }

    public void clear() { map.clear(); entryCount = 0; }
    public int size() { return entryCount; }
}
