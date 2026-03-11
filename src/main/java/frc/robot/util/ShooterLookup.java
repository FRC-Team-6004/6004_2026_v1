package frc.robot.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ShooterLookup {

    private final InterpolatingDoubleTreeMap tableLeft = new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap tableRight = new InterpolatingDoubleTreeMap();


    public ShooterLookup() {
        tableLeft.put(0.0, 0.0); //Distance in meters - Angle 0-1
        tableRight.put(0.0, 0.0); //Distance in meters - Angle 0-1
    }

    public double getLeftAngleForHubAt(double distance) {
        return tableLeft.get(distance);
    }

        public double getRightAngleForHubAt(double distance) {
        return tableRight.get(distance);
    }
}