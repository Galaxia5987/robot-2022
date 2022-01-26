package frc.robot;

import frc.robot.valuetuner.WebConstant;

public final class Constants {
    public static class ExampleSubsystem {
        private static final String NAME = ExampleSubsystem.class.getName();
        public static final WebConstant POWER = WebConstant.of(NAME, "power", 0.5); // [%]
    }
}
