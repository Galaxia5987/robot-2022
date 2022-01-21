package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class Constants {

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Vision {
        public static final double CAMERA_HEIGHT = 0.767; // [m]
        public static final double TARGET_HEIGHT = 2.64; // [m]
        public static final double CAMERA_PITCH = 0.471239; // [rads]

        public static final Pose2d HUB_POSE = new Pose2d();
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d();

    }
}
