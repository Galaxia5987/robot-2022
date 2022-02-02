package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static class Conveyor {
        public static final double POWER = 0.5; // [%]
        public static final int MAX_CARGO_AMOUNT = 2;
        public static final int OVERRIDE_INVALID_COLOR_DISTANCE = 100;
        public static final Color RED = new Color(0.03, 0.18, 0.76);
        public static final Color BLUE = new Color(0.22, 0.04, 0.73);
        public static final Color YELLOW = new Color(0.11, 0.16, 0.71);
        public static final Color NONE = new Color(0.01, 0.006, 0.98);
        public static final int BLUE_DEADBAND = 2000;
        public static final int RED_DEADBAND = 1500;
    }
}
