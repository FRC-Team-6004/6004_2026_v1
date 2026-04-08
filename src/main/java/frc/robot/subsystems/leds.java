package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class leds extends SubsystemBase {

    private static final int LED_LENGTH = 82;
    private static final int PWM_PORT = 4;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private double tick = 0.0;
    private static final double SPEED = 0.75;

    private static final int PACMAN_SIZE = 2;
    private final int[] ghostOffsets = {4, 8, 12, 16};
    private final int[][] ghostColors = {
        {255, 0, 0},
        {255, 105, 180},
        {0, 255, 255},
        {255, 165, 0}
    };

    private boolean reverseMode = false;

    public leds() {
        led = new AddressableLED(PWM_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);

        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        // update tick based on SPEED
        if (tick >= LED_LENGTH) tick -= LED_LENGTH;
        if (tick < 0) tick += LED_LENGTH;
    }

    public void pacmanMode() {
        clearBuffer();

        int pacmanPos = (int) tick; // Pac-Man forward with tick


        // Pac-Man (yellow)
        for (int i = 0; i < PACMAN_SIZE; i++) {
            buffer.setRGB((pacmanPos + i) % LED_LENGTH, 255, 220, 0);
        }

        // Ghosts normal offsets/colors
        for (int g = 0; g < ghostOffsets.length; g++) {
            int ghostPos = (pacmanPos - ghostOffsets[g] + LED_LENGTH) % LED_LENGTH;
            buffer.setRGB(ghostPos, ghostColors[g][0], ghostColors[g][1], ghostColors[g][2]);
        }

        led.setData(buffer);
        tick += SPEED;

    }

    public void reversedBlueGhostsMode() {
        clearBuffer();

        int pacmanPos = (int) tick; // Pac-Man moves backward

        // Pac-Man (yellow)
        for (int i = 0; i < PACMAN_SIZE; i++) {
            buffer.setRGB((pacmanPos + i) % LED_LENGTH, 255, 220, 0);
        }

        // Ghosts reversed offsets, all blue
        for (int g = 0; g < ghostOffsets.length; g++) {
            int ghostPos = (pacmanPos - ghostOffsets[g] + LED_LENGTH) % LED_LENGTH;
            buffer.setRGB(ghostPos, 0, 0, 255);
        }

        led.setData(buffer);

        tick -= SPEED;

    }

    private void clearBuffer() {
        for (int i = 0; i < LED_LENGTH; i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }
}