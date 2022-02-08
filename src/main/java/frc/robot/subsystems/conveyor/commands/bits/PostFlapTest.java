package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class PostFlapTest extends CommandBase {
    private final Conveyor conveyor;
    private int expected = 5;
    private int actual;

    public PostFlapTest(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setPower(0.5);
        if (!conveyor.isPostFlapBeamConnected()) {
            if (conveyor.isPostFlapBeamConnected()) {
                actual++;
                System.out.println(actual);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("actual:" + " " + actual);
        System.out.println("expected:" + " " + expected);
        System.out.println("delta:" + " " + (expected - actual));
    }
}
