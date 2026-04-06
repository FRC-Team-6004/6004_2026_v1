package frc.robot.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ShooterLookup {

    private final InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    public ShooterLookup() {
        putInTable(0.914781, 2650, 0.0);
        putInTable(1.158665, 2750, 0.0);
        putInTable(1.323822, 2800, 0.0);
        putInTable(1.5731, 3000, 0.0);
        putInTable(1.70744, 3100, 0.0);
        putInTable(2.065012, 3250, 0.0);
        putInTable(2.342901, 3375, 0.0);
        putInTable(2.56123, 3500, 0.0);
        putInTable(3.0445, 3500, 0.1);
        putInTable(3.356, 3550, 0.1);
        putInTable(3.62073, 3575, 0.15);
        putInTable(3.837, 3650, 0.2);
        putInTable(4.07965, 3700, 0.25);
        putInTable(4.51245, 3775, 0.3);
        putInTable(4.8312, 3850, 0.35);

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