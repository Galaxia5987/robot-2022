package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    Timer timer = new Timer();
    private AddressableLED m_led = new AddressableLED(1);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(54);
    private int current = 1;
    private int m_rainbowFirstPixelHue = 0;
    private int m_water = 80;
    private int sign = 1;

    public LedSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        timer.start();
        timer.reset();
    }

    public static int[] switchOfGods(int idx) { // 6 states
        switch (idx) {
//            case 0:
//                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                        0, 0, 0, 0, 0, 0, 0, 0, 0};
            case 1:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                        1, 0, 0, 0, 0, 0, 0, 0, 0};
            case 2:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                        1, 1, 0, 0, 0, 0, 0, 0, 0};
            case 3:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
                        1, 1, 1, 0, 0, 0, 0, 0, 0};
            case 4:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
                        1, 1, 1, 1, 0, 0, 0, 0, 0};
            case 5:
                return new int[]{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 0, 0, 0, 0};
            case 6:
                return new int[]{0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 0, 0, 0};

            case 7:
                return new int[]{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 0, 0};

            case 8:
                return new int[]{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1, 0};

            case 9:
                return new int[]{0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1, 1};
        }
        return null;
    }

    public void updateCurrent(int states) {
        current++;
        if (current == states) {
            current = 1;
        }
    }

    @Override
    public void periodic() {
        double time = 0.2;
        if(current == 9){
            time = 0.4;
        }
        if (timer.hasElapsed(time)) {
            updateCurrent(10);
            timer.reset();
        }
        for (var i = 0; i < 20; i++) {
            if (switchOfGods(current)[i] == 1) {
                m_ledBuffer.setHSV(i, 10, 255, 128);
            } else {
                m_ledBuffer.setRGB(i, 25, 25, 25);
            }
        }
        m_ledBuffer.setRGB(0, 0, 0, 0);
        m_ledBuffer.setRGB(1, 0, 0, 0);

        for (var i = 20; i < m_ledBuffer.getLength(); i++) {
//            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
//            final var hue = (m_water + (i * 35 / 34));
            final var hue = (m_water);
//            m_ledBuffer.setHSV(i, hue, 255, 128);
            m_ledBuffer.setRGB(i, 0, 156, 189);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        m_water += sign;
        if (m_water >= 130) {
            sign = -1;
        }

        if (m_water <= 80) {
            sign = 1;
        }
        m_led.setData(m_ledBuffer);
    }
}