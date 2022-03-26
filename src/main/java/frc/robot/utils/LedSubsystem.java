package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    public static LedMode currentLedMode = LedMode.STATIC;
    private final Timer timer = new Timer();
    private final Timer blinkTimer = new Timer();
    private int m_rainbowFirstPixelHue = 0;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private int current = 0;
    private boolean colorOn = true;

    public LedSubsystem() {
        led = new AddressableLED(6);
        ledBuffer = new AddressableLEDBuffer(18);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
        timer.reset();
        timer.start();
        blinkTimer.reset();
        blinkTimer.start();
    }

    public static int[] switchOfGods(int idx) { // 6 states
        switch (idx) {
            case 0:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0};
            case 1:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 1,
                        1, 0, 0, 0, 0, 0, 0, 0, 0};
            case 2:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 1, 1,
                        1, 1, 0, 0, 0, 0, 0, 0, 0};
            case 3:
                return new int[]{0, 0, 0, 0, 0, 0, 1, 1, 1,
                        1, 1, 1, 0, 0, 0, 0, 0, 0};
            case 4:
                return new int[]{0, 0, 0, 0, 0, 1, 1, 1, 1,
                        1, 1, 1, 1, 0, 0, 0, 0, 0};
            case 5:
                return new int[]{0, 0, 0, 0, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 0, 0, 0, 0};
            case 6:
                return new int[]{0, 0, 0, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 0, 0, 0};

            case 7:
                return new int[]{0, 0, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 0, 0};

            case 8:
                return new int[]{0, 1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1, 0};

            case 9:
                return new int[]{1, 1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1, 1};
        }
        return null;
    }

    public void updateCurrent(int states) {
        current++;
        if (current == states) {
            current = 0;
        }
    }

    private void rainbowHood() {
        for (var i = 0; i < 9; i++) {
            final var hue = ((180 - m_rainbowFirstPixelHue) + (i * 180 / 9)) % 180;
            ledBuffer.setHSV(8 - i, hue, 223, 217);
            ledBuffer.setHSV(i + 9, hue, 223, 217);
        }
    }

    public void blink(Color color) {
        if (blinkTimer.hasElapsed(0.1)) {
            blinkTimer.reset();
            colorOn = !colorOn;
        }

        if (colorOn) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, color);
            }
        } else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    @Override
    public void periodic() {

        if (timer.hasElapsed(0.2)) {
            updateCurrent(10);
            timer.reset();
        }

        if (currentLedMode == LedMode.STATIC) {
            for (var i = 0; i < 18; i++) {
                if (switchOfGods(current)[i] == 1) {
                    ledBuffer.setRGB(i, 0, 158, 189);
                } else {
                    ledBuffer.setRGB(i, 25, 25, 25);
                }
            }
        } else if (currentLedMode == LedMode.SHOOTING) {
            rainbowHood();
        } else {
            blink(currentLedMode.color);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        led.setData(ledBuffer);
    }

    public void setCurrentLedMode(LedMode ledMode) {
        currentLedMode = ledMode;
    }

    public enum LedMode {
        STATIC(Color.kBlack), ODOMETRY_ADJUST(Color.kOrange), VISION_ADJUST(Color.kGreen), SHOOTING(Color.kBlack);
        private final Color color;

        LedMode(Color color) {
            this.color = color;
        }
    }
}