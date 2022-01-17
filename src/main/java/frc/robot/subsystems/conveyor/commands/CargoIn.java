package frc.robot.subsystems.conveyor.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.conveyor.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoIn extends CommandBase {
    private final Conveyor conveyor;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public CargoIn(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setPower(Constants.POWER);
    }

    @Override
    public void execute() {

    }

    // leave me alone
    @Override
    public boolean isFinished() {
        if (conveyor.getCargoCount() > 2) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
    conveyor.setPower(0);
    }
}



