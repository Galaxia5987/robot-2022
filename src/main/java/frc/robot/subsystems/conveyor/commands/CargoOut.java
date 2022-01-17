package frc.robot.subsystems.conveyor.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.conveyor.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoOut extends CommandBase {
    private final Conveyor conveyor;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public CargoOut(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
        DriverStation.getAlliance().equals(DriverStation.Alliance)
    }

    @Override
    public void initialize() {
        conveyor.setPower(-Constants.POWER);
    }

    @Override
    public void execute() {

    }

    // leave me alone
    @Override
    public boolean isFinished() {
    }

    @Override
    public void end(boolean interrupted) {

    }
}