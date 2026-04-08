package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Subsystem that computes the current shift timer for FRC 2026 and
 * publishes to NetworkTables for dashboards/Elastic.
 * Handles null/invalid DriverStation data safely.
 */
public class ShiftTimer extends SubsystemBase {

    // NetworkTables entries
    private final NetworkTable table;
    private final NetworkTableEntry secondsUntilNextShiftEntry;
    private final NetworkTableEntry activeNowEntry;
    private final NetworkTableEntry matchTimeEntry; // NEW: match time in seconds

    private boolean active;

    public ShiftTimer() {
        table = NetworkTableInstance.getDefault().getTable("ShiftTimer");
        secondsUntilNextShiftEntry = table.getEntry("secondsUntilNextShift");
        activeNowEntry = table.getEntry("activeNow");
        matchTimeEntry = table.getEntry("matchTime"); // NEW entry
    }

    @Override
    public void periodic() {
        // --- Read DriverStation info safely ---
        double matchTime = DriverStation.getMatchTime();
        if (Double.isNaN(matchTime) || matchTime < 0) {
            matchTime = 0; // fallback if match time unavailable
        }
        int matchTimeSec = (int) Math.ceil(matchTime);

        Alliance alliance = DriverStation.getAlliance().get(); // only Red or Blue

        String gameData = DriverStation.getGameSpecificMessage();
        boolean redInactiveFirst = false; // default to false if missing
        if (gameData != null && gameData.length() > 0) {
            redInactiveFirst = gameData.charAt(0) == 'R';
        }

        // --- Shift boundaries in seconds remaining ---
        int AUTO_END = 140;
        int TRANSITION_END = 130;
        int SHIFT1_END = 105;
        int SHIFT2_END = 80;
        int SHIFT3_END = 55;
        int SHIFT4_END = 30;
        int ENDGAME_END = 0;

        int nextBoundary;

        boolean shift1ActiveForRed = !redInactiveFirst;
        boolean shift1ActiveForBlue = redInactiveFirst;

        // --- Determine shift and active status ---
        if (matchTimeSec > AUTO_END) {
            nextBoundary = matchTimeSec - AUTO_END;
            active = true; // Auto: both active
        } else if (matchTimeSec > TRANSITION_END) {
            nextBoundary = matchTimeSec - TRANSITION_END;
            active = true; // Transition: both active
        } else if (matchTimeSec > SHIFT1_END) {
            nextBoundary = matchTimeSec - SHIFT1_END;
            active = (alliance == Alliance.Red) ? shift1ActiveForRed : shift1ActiveForBlue;
        } else if (matchTimeSec > SHIFT2_END) {
            nextBoundary = matchTimeSec - SHIFT2_END;
            active = (alliance == Alliance.Red) ? !shift1ActiveForRed : !shift1ActiveForBlue;
        } else if (matchTimeSec > SHIFT3_END) {
            nextBoundary = matchTimeSec - SHIFT3_END;
            active = (alliance == Alliance.Red) ? shift1ActiveForRed : shift1ActiveForBlue;
        } else if (matchTimeSec > SHIFT4_END) {
            nextBoundary = matchTimeSec - SHIFT4_END;
            active = (alliance == Alliance.Red) ? !shift1ActiveForRed : !shift1ActiveForBlue;
        } else if (matchTimeSec > ENDGAME_END){
            nextBoundary = matchTimeSec - ENDGAME_END;
            active = true; // Endgame: both active
        } else {
            nextBoundary = -1;
            active = false;
        }

        int secondsUntilNextShift = Math.max(0, nextBoundary);

        // --- Publish to NetworkTables ---
        secondsUntilNextShiftEntry.setNumber(secondsUntilNextShift);
        activeNowEntry.setBoolean(active);
        matchTimeEntry.setNumber(matchTimeSec); // NEW: publish match time
    }

    public boolean Active() {
        return active;
    }
}