package frc.robot.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ShooterLookup {

    private final InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    public ShooterLookup() {
        putInTable(1.1, 2650, 0.0);
        putInTable(2.525, 3250, 0.0);
        putInTable(2.6029, 3300, 0.0);
        putInTable(2.975, 3450, 0.0);
        putInTable(3.068, 3500, 0.0);
        putInTable(3.669, 3725, 0.1);
        putInTable(4.4, 3800, 0.2);
        putInTable(4.82, 4000, 0.3);
    
    }

    private void putInTable(double distance, double RPM, double Angle) {
        Matrix<N2, N1> newVal = new Matrix<>(Nat.N2(), Nat.N1());  
        newVal.set(0, 0, RPM);     
        newVal.set(1, 0, Angle);     
        table.put(distance, newVal);  
    }

    public double getServo(double distance) {
        return table.get(distance).get(1, 0);
    }

    public double getRPM(double distance) {
        return table.get(distance).get(0, 0);
    }
}