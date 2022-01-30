package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Ports {
    public static class Conveyor {
        public static final int MOTOR = 0;
        public static final int SOLENOID = 0;
        public static final int BEAM_BREAKER = 0;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
    }

    public static class Controls {
        public static final int XBOX = 0;
    }

}
