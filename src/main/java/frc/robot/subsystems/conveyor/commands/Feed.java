package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class Feed extends CommandBase {
    private final double power;
    private final Conveyor conveyor;

    public Feed(double power, Conveyor conveyor) {
        this.power = power;
        this.conveyor = conveyor;
    }

    @Override
    public void execute() {
        if(power > 0) {
            conveyor.openFlap();
        } else {
            conveyor.closeFlap();
        }
        conveyor.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getCargoCount() == 0;
    }
}
