package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static class Conveyor {
        public static final double POWER = 0.5; // [%]
        public static final int MAX_CARGO_AMOUNT = 2;
        public static final int MIN_PROXIMITY_VALUE = 100;
        public static final Color RED = new Color(0.15, 0.55, 0.3);
        public static final Color BLUE = new Color(0.02, 0.51, 0.44);
        public static final Color GREEN = new Color(0.06, 0.54, 0.39);
        public static final Color NONE   = new Color(2e-4, 0.94, 0.045);
    }
}
