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
    public void initialize() {
        conveyor.closeFlap();
    }

    @Override
    public void execute() {
        if (conveyor.getProximityValue() >= Constants.Conveyor.MIN_PROXIMITY_VALUE) {
            if (conveyor.getCargoCount() >= Constants.Conveyor.MAX_CARGO_AMOUNT) {
                conveyor.setPower(0);
            } else {
               conveyor.setPower(power);
            }
        } else if (conveyor.getCargoCount() == 1 && conveyor.getPreFlapBeamInput()) {
            conveyor.setPower(power);
        } else {
            conveyor.setPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }
}
