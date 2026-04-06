package frc.robot.util;

/** Distance-keyed lookup table for shot parameters. */
public class ShotLUT {

    private final edu.wpi.first.math.interpolation.InterpolatingTreeMap<Double, ShotParam> map;
    private int entryCount = 0;

    public ShotLUT() {
        map = new edu.wpi.first.math.interpolation.InterpolatingTreeMap<>(
                edu.wpi.first.math.interpolation.InverseInterpolator.forDouble(),
                ShotParam.interpolator());

        // Fill LUT with ShooterLookup values + realistic TOF
        put(0.914781, 2650, 0.0, 1.5);
        put(1.158665, 2750, 0.0, 1.55);
        put(1.323822, 2800, 0.0, 1.6);
        put(1.5731,   3000, 0.0, 1.65);
        put(1.70744,  3100, 0.0, 1.7);
        put(2.065012, 3250, 0.0, 1.8);
        put(2.342901, 3375, 0.0, 1.9);
        put(2.56123,  3500, 0.0, 2.0);
        put(3.0445,   3500, 0.1, 2.1);
        put(3.356,    3550, 0.1, 2.2);
        put(3.62073,  3575, 0.15, 2.3);
        put(3.837,    3650, 0.2, 2.4);
        put(4.07965,  3700, 0.25, 2.5);
        put(4.51245,  3775, 0.3, 2.65);
        put(4.8312,   3850, 0.35, 2.8);
    }

    /** Insert or overwrite parameters at a given distance. */
    public void put(double distanceM, double rpm, double angleDeg, double tofSec) {
        put(distanceM, new ShotParam(rpm, angleDeg, tofSec));
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
    public double getAngle(double distanceM) { return get(distanceM).angleDeg(); }
    public double getTOF(double distanceM) { return get(distanceM).tofSec(); }

    public void clear() { map.clear(); entryCount = 0; }
    public int size() { return entryCount; }
}