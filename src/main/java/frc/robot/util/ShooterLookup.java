package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Distance (m) to flywheel speed (RPM) lookup. The hood is static, so no angle is stored. */
public class ShooterLookup {

    private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

    public ShooterLookup() {
        table.put(0.914781, 2650.0);
        table.put(1.158665, 2750.0);
        table.put(1.323822, 2800.0);
        table.put(1.5731, 3000.0);
        table.put(1.70744, 3100.0);
        table.put(2.065012, 3250.0);
        table.put(2.342901, 3375.0);
        table.put(2.56123, 3500.0);
        table.put(3.0445, 3500.0);
        table.put(3.356, 3550.0);
        table.put(3.62073, 3575.0);
        table.put(3.837, 3650.0);
        table.put(4.07965, 3700.0);
        table.put(4.51245, 3775.0);
        table.put(4.8312, 3850.0);
    }

    /** Interpolated RPM at a distance; clamps to the nearest entry outside the table. */
    public double getRPM(double distance) {
        return table.get(distance);
    }
}
