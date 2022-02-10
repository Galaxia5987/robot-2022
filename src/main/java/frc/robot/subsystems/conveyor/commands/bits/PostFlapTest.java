package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class PostFlapTest extends CommandBase {
    private final Conveyor conveyor;
    private boolean wasConnected = true;
    private int cargoPassedCount = 0;

    public PostFlapTest(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setPower(Constants.Conveyor.DEFAULT_POWER);
        boolean isConnected = conveyor.isPostFlapBeamConnected();

        if (wasConnected && !isConnected) {
            cargoPassedCount++;
            System.out.println("Cargo passed");
        }
        wasConnected = isConnected;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Cargo", cargoPassedCount);
    }
}
