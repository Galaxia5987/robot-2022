package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoOut extends CommandBase {
    private final Conveyor conveyor;

    public CargoOut(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        var color = DriverStation.getAlliance();
        if (conveyor.getColorPosition(true) == color) {
            if (conveyor.getColorPosition(false) == color) {
                // do nothing
            }
            if (conveyor.getColorPosition(false) != color) {
                conveyor.setPower(-Constants.Conveyor.POWER);
                // use InTake
            }

        }
        if (conveyor.getColorPosition(true) != color) {
            if (conveyor.getColorPosition(false) == color) {
                conveyor.setPower(Constants.Conveyor.POWER);
                //use shooter
            }
            if (conveyor.getColorPosition(false) != color) {
                // out from any direction
            }
            if (conveyor.getColorPosition(false) == color) {
                conveyor.setPower(Constants.Conveyor.POWER);
                // use shooter - in case there is only one ball
            }
        }

    }


    @Override
    public void execute() {


    }

    // leave me alone
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}