package frc.robot.subsystems.example.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.example.Deez_Nuts;

public class ExampleCommand extends CommandBase {
    private final Deez_Nuts exampleSubsystem;

    public ExampleCommand(Deez_Nuts exampleSubsystem) {
        this.exampleSubsystem = exampleSubsystem;
        addRequirements(exampleSubsystem);
    }

    @Override
    public void initialize() {
        exampleSubsystem.setPower(0);
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
