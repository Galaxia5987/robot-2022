import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Units;
import frc.robot.utils.Utils;

import static frc.robot.Constants.Shooter.TICKS_PER_REVOLUTION;

public class Test {
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);

    @org.junit.Test
    public void test() {
        System.out.println(unitModel.toTicks100ms(Utils.rpmToRps(50)));

    }
}
