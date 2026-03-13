package frc.robot.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

public class ShooterLookup {

    private final InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    public ShooterLookup() {
        putInTable(63.5, 2800, 0.0);
        putInTable(85, 3000, 0.0);
        putInTable(120, 3250, 0.25);
        putInTable(137, 2800, 0.25);
        putInTable(190, 4000, 0.2);
    }

    private void putInTable(double distanceInInches, double RPM, double Angle) {
        Matrix<N2, N1> newVal = new Matrix<>(Nat.N2(), Nat.N1());  
        newVal.set(0, 0, RPM);     
        newVal.set(1, 0, Angle);     
        table.put(Units.inchesToMeters(distanceInInches), newVal);  
    }

    public double getServo(double distance) {
        return table.get(distance).get(0, 1);
    }

    public double getRPM(double distance) {
        return table.get(distance).get(0, 0);
    }
}