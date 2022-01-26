package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class Leds {

    private final AddressableLED addressableLED = new AddressableLED(0);

    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    private final Timer climbLedsTimer = new Timer();
    private int rainbowFirstPixelHue;

    public Leds() {
        addressableLED.setLength(ledBuffer.getLength());
        addressableLED.setData(ledBuffer);
        addressableLED.start();
    }

    public void disabledPeriodic() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 161, 36);
        }

        addressableLED.setData(ledBuffer);
    }


    public void autonomousPeriodic() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 248, 37, 5);
            }
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 7, 88, 248);
            }
        } else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 60, 248, 236);
            }
        }
        addressableLED.setData(ledBuffer);
    }


    public void teleopPeriodic() {
        if (climbLedsTimer.get() == 0) {
            climbLedsTimer.reset();
            climbLedsTimer.start();
        } else {
            climbLedsTimer.stop();
            climbLedsTimer.reset();
        }

        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 30;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 60 / ledBuffer.getLength())) % 30;
        }
    }

    public enum Colors {
        RED(new int[]{255, 0, 0}),
        GREEN(new int[]{0, 255, 0}),
        BLUE(new int[]{0, 0, 255}),
        PURPLE(new int[]{204, 0, 204}),
        YELLOW(new int[]{255, 255, 0}),
        ORANGE(new int[]{255, 153, 0}),
        NONE(new int[]{0, 0, 0});

        private final int[] rgb;

        Colors(int[] rgb) {
            this.rgb = rgb;
        }

        public int[] getRgb() {
            return rgb;
        }
    }
}







