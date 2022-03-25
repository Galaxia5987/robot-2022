import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Shooter.TICKS_PER_REVOLUTION;

public class Test {
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);

    @org.junit.Test
    public void test() {
        var odom = new Translation2d(5, 2);
        var target = new Translation2d(3, 3);
        Translation2d relative = odom.minus(target);
        System.out.println(relative);
        double alpha = Math.atan2(relative.getY(), relative.getX());
        System.out.println(Math.toDegrees(alpha));
        double visionDistance = 2.23;
        Translation2d newTranslation = new Translation2d(Math.cos(alpha) * visionDistance, Math.sin(alpha) * visionDistance);
        System.out.println(target.plus(newTranslation));

    }
}
