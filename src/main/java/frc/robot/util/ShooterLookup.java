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
        putInTable(64, 2700, 0.0);
        putInTable(103, 3100, 0.0);
        putInTable(117, 3200, 0.0);
        putInTable(143, 3400, 0.0);
        putInTable(191, 3900, 0.2);
    }

    private void putInTable(double distanceInInches, double RPM, double Angle) {
        Matrix<N2, N1> newVal = new Matrix<>(Nat.N2(), Nat.N1());  
        newVal.set(0, 0, RPM);     
        newVal.set(1, 0, Angle);     
        table.put(Units.inchesToMeters(distanceInInches), newVal);  
    }

    public double getServo(double distance) {
        System.out.println("Servo");
        System.out.println(table.get(distance).get(1, 0));
        return table.get(distance).get(1, 0);
    }

    public double getRPM(double distance) {
        System.out.println("getRPM");
        System.out.println(table.get(distance).get(0, 0));
        return table.get(distance).get(0, 0);
    }
}