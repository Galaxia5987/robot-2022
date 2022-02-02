package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorDefaultCommand extends CommandBase {
    private final Conveyor conveyor;
    private final double power;

    public ConveyorDefaultCommand(Conveyor conveyor, double power) {
        this.conveyor = conveyor;
        this.power = power;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        if (conveyor.getProximityValue() >= Constants.Conveyor.OVERRIDE_INVALID_COLOR_DISTANCE || conveyor.getPreFlapBeam()){
            conveyor.setPower(power);
        } else {
            conveyor.setPower(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
