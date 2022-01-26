package frc.robot.subsystems.conveyor.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoIn extends CommandBase {
    private final Conveyor conveyor;

    public CargoIn(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setPower(Constants.Conveyor.POWER);
    }

    @Override
    public void execute() {

    }

    // leave me alone
    @Override
    public boolean isFinished() {
        return conveyor.getCargoCount() > Constants.Conveyor.MAX_CARGO_AMOUNT;
    }

    @Override
    public void end(boolean interrupted) {
    conveyor.setPower(0);
    }
}



