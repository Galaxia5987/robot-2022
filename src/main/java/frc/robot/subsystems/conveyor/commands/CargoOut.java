package frc.robot.subsystems.conveyor.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoOut extends CommandBase {
    private final Conveyor conveyor;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
//    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public CargoOut(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        if (conveyor.getColorPosition(true) == conveyor.dr)
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