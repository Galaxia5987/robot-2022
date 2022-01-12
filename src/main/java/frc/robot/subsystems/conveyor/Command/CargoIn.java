package frc.robot.subsystems.conveyor.Command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.example.Conveyor;

public class CargoIn extends CommandBase {
    private final Conveyor CargoIn;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public CargoIn(Conveyor CargoIn) {
        this.CargoIn = CargoIn;
        addRequirements(CargoIn);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}



