package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static class Conveyor {
        public static final double POWER = 0.5; // [%]
        public static final int MAX_CARGO_AMOUNT = 2;
        public static final int MIN_PROXIMITY_VALUE = 100; // random bit stuff
        public static final Color RED = new Color(1, 0, 0);
        public static final Color BLUE = new Color(0, 0, 1);
        public static final Color NONE = new Color(0, 0, 0);
        public static final int BLUE_DEADBAND = 2000;
        public static final int RED_DEADBAND = 1500;
    }
}
