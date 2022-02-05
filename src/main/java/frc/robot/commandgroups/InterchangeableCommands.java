package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

public class InterchangeableCommands extends CommandBase {
    private final BooleanSupplier condition;
    private final Command defaultCommand;
    private final Command auxiliaryCommand;

    public InterchangeableCommands(BooleanSupplier condition, Command defaultCommand, Command auxiliaryCommand) {
        this.condition = condition;
        this.defaultCommand = defaultCommand;
        this.auxiliaryCommand = auxiliaryCommand;

        requireUngrouped(defaultCommand, auxiliaryCommand);
    }

    @Override
    public void execute() {
        boolean currentConditionState = condition.getAsBoolean();

        if(currentConditionState) {
            auxiliaryCommand.end(true);
            if(!defaultCommand.isFinished()) {
                defaultCommand.initialize();
                defaultCommand.execute();
            }
        } else {
            defaultCommand.end(true);
            if(!auxiliaryCommand.isFinished()) {
                auxiliaryCommand.initialize();
                auxiliaryCommand.execute();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        defaultCommand.end(interrupted);
        auxiliaryCommand.end(interrupted);
    }
}
