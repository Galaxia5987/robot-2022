package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

public class InterchangeableCommands extends CommandBase {
    private final BooleanSupplier condition;
    private final Command defaultCommand;
    private final Command auxiliaryCommand;
    private boolean lastConditionState;

    public InterchangeableCommands(BooleanSupplier condition, Command defaultCommand, Command auxiliaryCommand) {
        this.condition = condition;
        this.defaultCommand = defaultCommand;
        this.auxiliaryCommand = auxiliaryCommand;

        requireUngrouped(defaultCommand, auxiliaryCommand);
    }

    @Override
    public void initialize() {
        lastConditionState = condition.getAsBoolean();
    }

    @Override
    public void execute() {
        boolean currentConditionState = condition.getAsBoolean();
        Command currentCommand = currentConditionState ? defaultCommand : auxiliaryCommand;
        Command lastCommand = lastConditionState ? defaultCommand : auxiliaryCommand;

        if (currentConditionState != lastConditionState) {
            lastCommand.end(true);
            currentCommand.initialize();
        } else {
            if(lastCommand.isFinished()) {
                lastCommand.end(false);
            } else {
                lastCommand.execute();
            }
        }

        lastConditionState = currentConditionState;
    }

    @Override
    public void end(boolean interrupted) {
        defaultCommand.end(interrupted);
        auxiliaryCommand.end(interrupted);
    }
}
