package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    public static boolean climbTime = false;
    private final AddressableLED m_led = new AddressableLED(1);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(54);
    private final int m_water = 80;
    private final int sign = 1;
    Timer timer = new Timer();
    Timer timer2 = new Timer();
    private int current = 1;
    private int current2 = 0;
    private int m_rainbowFirstPixelHue = 0;
    private int percent = 0;
    private boolean neutralMode = true;
    private boolean isTesting = false;
    private Color color = new Color(0, 0, 0);

    public LedSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        timer.start();
        timer.reset();
        timer2.start();
        timer2.reset();
        climbTime = false;
    }

    public void setTesting(boolean testing) {
        isTesting = testing;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public static int[] switchOfGods(int idx) { // 6 states
        switch (idx) {
            case 0:
                return new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0};
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

    public static int[] switchOfGods2(int idx) { // 3 states
        switch (idx) {
            case 0:
                return new int[]{0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
                        1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0};
            case 1:
                return new int[]{0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0
                        ,
                        0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0};
            case 2:
                return new int[]{0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0
                        ,
                        0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0};
            case 3:
                return new int[]{1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0,
                        0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1};
            case 4:
                return new int[]{1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1,
                        1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1};
            case 5:
                return new int[]{0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1,
                        1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0};
            case 6:
                return new int[]{1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0,
                        0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1};
            case 7:
                return new int[]{1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1,
                        1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1};
            case 8:
                return new int[]{0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1,
                        1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0};
            case 9:
                return new int[]{1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0,
                        0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1};
            case 10:
                return new int[]{1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1,
                        1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1};
            case 11:
                return new int[]{0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1,
                        1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0};
            case 12:
                return new int[]{1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
                        0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1};
            case 13:
                return new int[]{1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
                        1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1};
            case 14:
                return new int[]{0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
                        1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0};
            case 15:
                return new int[]{1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
                        0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1};
            case 16:
                return new int[]{1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
                        1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1};
        }
        return null;
    }

    public void updateCurrent(int states) {
        current++;
        if (current == states) {
            current = 1;
        }
    }

    public void updateCurrent2(int states) {
        current2++;
        if (current2 == states) {
            current2 = 0;
        }
    }

    @Override
    public void periodic() {

        if (!isTesting) {
            if (DriverStation.isAutonomous() || DriverStation.getMatchTime() == -1) {
                climbTime = false;
            }
            if (!DriverStation.isAutonomous() && DriverStation.getMatchTime() != -1 && DriverStation.getMatchTime() <= 30) {
                climbTime = true;
            }
            double time = 0.2;
            if (current == 9) {
                time = 0.4;
            }
            if (timer.hasElapsed(time)) {
                updateCurrent(10);
                updateCurrent2(17);
                timer.reset();
            }
            if (neutralMode) {
                for (var i = 0; i < 20; i++) {
                    if (switchOfGods(current)[i] == 1) {
//                    m_ledBuffer.setHSV(i, 10, 255, 128);
                        m_ledBuffer.setRGB(i, 0, 158, 189);
                    } else {
                        m_ledBuffer.setRGB(i, 25, 25, 25);
                    }
                }
            } else {
                for (var i = 0; i < 20; i++) {
                    if (switchOfGods(percent)[i] == 1) {
                        m_ledBuffer.setRGB(i, 0, 255, 0);
                    } else {
                        m_ledBuffer.setRGB(i, 25, 25, 25);
                    }
                }
            }

            if (climbTime) {
                for (var i = 0; i < 20; i++) {
                    m_ledBuffer.setRGB(i, 0, 158, 189);
                }
            }


            m_ledBuffer.setRGB(0, 0, 0, 0);
            m_ledBuffer.setRGB(1, 0, 0, 0);

/*        for (var i = 20; i < m_ledBuffer.getLength(); i++) {
//            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
//            final var hue = (m_water + (i * 35 / 34));
//            final var hue = (m_water);
//            m_ledBuffer.setHSV(i, hue, 255, 128);
            if (switchOfGods2(current2)[i - 20] == 1) {
                m_ledBuffer.setRGB(i, 0, 156, 189);
            } else {
                m_ledBuffer.setRGB(i, 25, 25, 25);
            }
//            m_ledBuffer.setRGB(i, 0, 156, 189);
        }*/
//        m_rainbowFirstPixelHue += 3;
//        m_rainbowFirstPixelHue %= 180;
//        m_water += sign;
//        if (m_water >= 130) {
//            sign = -1;
//        }
//
//        if (m_water <= 80) {
//            sign = 1;
//        }
            rainbow2();
            m_led.setData(m_ledBuffer);
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
        } else {
           for (int i = 37; i < m_ledBuffer.getLength(); i++) {
               m_ledBuffer.setLED(i, color);
           }
           m_led.setData(m_ledBuffer);
        }
    }

    private void rainbow2() {
        // For every pixel
        for (var i = 37; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = ((180 - m_rainbowFirstPixelHue) + (i * 180 / 17)) % 180;
            // Set the value
            if (neutralMode) {
                m_ledBuffer.setLED(i, Color.kPurple);
//                m_ledBuffer.setRGB(i, 219, 127, 142);
                m_ledBuffer.setLED(36 - (i - 37), Color.kPurple);
//                m_ledBuffer.setRGB(36 - (i - 37), 219, 127, 142);
            } else {
                m_ledBuffer.setHSV(i, hue, 223, 217);
                m_ledBuffer.setHSV(36 - (i - 37), hue, 223, 217);
            }
        }
        // Increase by to make the rainbow "move"
//        m_rainbowFirstPixelHue += 3;
        // Check bounds
//        m_rainbowFirstPixelHue %= 180;

        if (climbTime) {
            for (var i = 37; i < m_ledBuffer.getLength(); i++) {
                final var hue = ((180 - m_rainbowFirstPixelHue) + (i * 180 / 17)) % 180;
                m_ledBuffer.setHSV(i, hue, 223, 217);
                m_ledBuffer.setHSV(36 - (i - 37), hue, 223, 217);
            }
        }
    }

    public void setNeutralMode(boolean neutralMode) {
        this.neutralMode = neutralMode;
    }

    public void setPercent(int percent) {
        this.percent = percent;
    }


}