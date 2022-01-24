package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.climber.Climber;

public class Leds {

    private final AddressableLED addressableLED = new AddressableLED(0);

    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    private final Timer climbLedsTimer = new Timer();
    private final Timer shooterLedsTimer = new Timer();
    private final Climber climber;
    private int rainbowFirstPixelHue;

    public Leds(Climber climber) {
        this.climber = climber;

        addressableLED.setLength(ledBuffer.getLength());
        addressableLED.setData(ledBuffer);
        addressableLED.start();
    }

    public void disabledPeriodic() {
        int red, blue, green;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            red = 255;
            blue = 0;
            green = 0;
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            red = 0;
            blue = 255;
            green = 0;
        } else {
            red = 239;
            blue = 8;
            green = 247;
        }

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
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
    }


}






