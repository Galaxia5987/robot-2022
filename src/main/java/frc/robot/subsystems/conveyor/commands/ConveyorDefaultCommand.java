package frc.robot.subsystems.conveyor.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.example.Deez_Nuts;

public class ConveyorDefaultCommand extends CommandBase {
    private final Deez_Nuts ConveyorDefaultCommand;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
//    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public ConveyorDefaultCommand(Deez_Nuts ConveyorDefaultCommand) {
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

