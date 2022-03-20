package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Random;

//  9
//  3
//  7
public class SnakeSubsystem extends SubsystemBase {
    private final AddressableLED climberLEDs = new AddressableLED(0); // left
    private final AddressableLEDBuffer climberLEDBuffer = new AddressableLEDBuffer(9);
//    private final AddressableLED climberLEDs2 = new AddressableLED(8);
//    private final AddressableLEDBuffer climberLEDBuffer2 = new AddressableLEDBuffer(10);
    public Timer timer = new Timer();
    public Random random = new Random();
    public int world = 16;
    public int tailPosition = 0;
    public int headPosition = 1;
    public int applePosition = 5;
    public int[] array = new int[16];

    public SnakeSubsystem() {
        timer.start();
        timer.reset();
        climberLEDs.setLength(climberLEDBuffer.getLength());
        climberLEDs.setData(climberLEDBuffer);
        climberLEDs.start();
//        climberLEDs2.setLength(climberLEDBuffer2.getLength());
//        climberLEDs2.setData(climberLEDBuffer2);
//        climberLEDs2.start();
    }

    public void annoying() {
        headPosition++;
        tailPosition++;
        headPosition %= world;
        tailPosition %= world;//©€

        for (int i = 0; i < array.length; i++) {
            if (i == headPosition) {
                array[i] = 2;
            } else if (i == tailPosition) {
                array[i] = 1;
            } else {
                array[i] = 0;
            }
        }

        if (headPosition > tailPosition) {
            for (int i = tailPosition; i < headPosition; i++) {
                array[i] = 1;
            }
        } else {
            for (int i = 0; i < headPosition; i++) {
                array[i] = 1;
            }

            for (int i = tailPosition; i < array.length; i++) {
                array[i] = 1;
            }
        }
        boolean realReset = false;
        if (headPosition == applePosition) {
            tailPosition--;
            if (tailPosition < 0) {
                tailPosition = 16 + tailPosition;
            }
            while (true) {
                boolean reset = true;

                for (int i = 0; i < array.length; i++) {
                    if (array[i] == 0) {
                        reset = false;
                    }
                }
                realReset = reset;
                applePosition = random.nextInt(16);
                if (array[applePosition] == 0 || reset) {
                    break;
                }
            }
        }

        for (int i = 0; i < array.length; i++) {
            if (i == headPosition) {
                array[i] = 2;
            } else if (i == tailPosition) {
                array[i] = 1;
            } else {
                array[i] = 0;
            }
        }

        if (headPosition > tailPosition) {
            for (int i = tailPosition; i < headPosition; i++) {
                array[i] = 1;
            }
        } else {
            for (int i = 0; i < headPosition; i++) {
                array[i] = 1;
            }

            for (int i = tailPosition; i < array.length; i++) {
                array[i] = 1;
            }
        }


        array[applePosition] = 8;
        if (realReset) {
            array = new int[16];
            tailPosition = 0;
            headPosition = 1;
            applePosition = 5;
        }
    }

    @Override
    public void periodic() {
        if (timer.hasElapsed(0.5)) {
            annoying();
            timer.reset();
        }
        for (int i = 0; i < 9; i++) {
            if (array[i] == 0) {
                climberLEDBuffer.setLED(i, Color.kWhite);
            }
            if (array[i] == 1 || array[i] == 2) {
                climberLEDBuffer.setLED(i, Color.kGreen);
            }
            if (array[i] == 8) {
                climberLEDBuffer.setLED(i, Color.kRed);
            }
        }

        for (int i = 0; i < 7; i++) {
            if (array[i] == 0) {
//                climberLEDBuffer2.setLED(i + 3, Color.kWhite);
            }
            if (array[i] == 1 || array[i] == 2) {
//                climberLEDBuffer2.setLED(i + 3, Color.kGreen);
            }
            if (array[i] == 8) {
//                climberLEDBuffer2.setLED(i + 3, Color.kRed);
            }
        }


        climberLEDs.setData(climberLEDBuffer);
//        climberLEDs2.setData(climberLEDBuffer2);
    }
}
