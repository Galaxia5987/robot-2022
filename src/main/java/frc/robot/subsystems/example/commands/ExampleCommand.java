package frc.robot.subsystems.example.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.example.Conveyor;

public class ExampleCommand extends CommandBase {
    private final Conveyor exampleSubsystem;

    public ExampleCommand(Conveyor exampleSubsystem) {
        this.exampleSubsystem = exampleSubsystem;
        addRequirements(exampleSubsystem);
    }

    @Override
    public void initialize() {
        exampleSubsystem.setPower(Constants.ExampleSubsystem.POWER);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        exampleSubsystem.setPower(0);
    }
}
