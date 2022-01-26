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
        rainbowFirstPixelHue += 10;
        rainbowFirstPixelHue %= 360;
        int hue;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            hue = 0;
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            hue = 120;
        } else hue = 10;

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, 255, 45 + (int) (60 * (Math.sin(Math.toRadians(rainbowFirstPixelHue)) + 1) / 2));
        }

        addressableLED.setData(ledBuffer);
    }


    public void autonomousPeriodic() {
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
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
            int a;

        
            }
        }
    }
}






