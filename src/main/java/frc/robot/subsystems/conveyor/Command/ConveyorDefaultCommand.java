package frc.robot.subsystems.conveyor.Command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.example.Conveyor;

public class ConveyorDefaultCommand extends CommandBase {
    private final Conveyor ConveyorDefaultCommand;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public ConveyorDefaultCommand(Conveyor ConveyorDefaultCommand) {
        this.ConveyorDefaultCommand = ConveyorDefaultCommand;
        addRequirements(ConveyorDefaultCommand);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

