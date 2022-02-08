package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class PostFlapTest extends CommandBase {
    private final Conveyor conveyor;
    private final double expected;
    private boolean isConnected = true;
    private boolean wasConnected = true;

    private int actual;

    public PostFlapTest(Conveyor conveyor, double expected) {
        this.conveyor = conveyor;
        this.expected = expected;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setPower(0.5);
        isConnected = conveyor.isPostFlapBeamConnected();

        if (wasConnected && !isConnected){
            actual++;
            System.out.println(actual);
        }
        wasConnected = isConnected;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("actual:" + " " + actual);
        System.out.println("expected:" + " " + expected);
        System.out.println("delta:" + " " + (expected - actual));
    }
}
