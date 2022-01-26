package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.Leds.SWERVE_LED_LENGTH;
import static frc.robot.Ports.Leds.SWERVE_LED_PORT;


public class Leds {

    private final AddressableLED addressableLED = new AddressableLED(0);
    private AddressableLED swerveAddressableLED;

    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    private AddressableLEDBuffer swerveLedBuffer;
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

    private void configSwerveLEDs() {
        swerveAddressableLED = new AddressableLED(SWERVE_LED_PORT);
        swerveLedBuffer = new AddressableLEDBuffer(SWERVE_LED_LENGTH);
    }

    private void updateSwerveLEDs() {
        int[] rgb;
        if(DriverStation.isDisabled()){
            rgb = Colors.ORANGE.getRgb();
        } else {
            if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)){
                rgb = Colors.BLUE.getRgb();
            } else {
                rgb = Colors.RED.getRgb();
            }
        }
        for (int i = 0; i < swerveLedBuffer.getLength(); i++) {
            swerveLedBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
        swerveAddressableLED.setData(swerveLedBuffer);
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







