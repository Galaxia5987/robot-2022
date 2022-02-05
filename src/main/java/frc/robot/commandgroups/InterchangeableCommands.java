package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

public class InterchangeableCommands extends CommandBase {
    private final BooleanSupplier condition;
    private final Command auxiliaryCommand;
    private final Command defaultCommand;
    private boolean lastConditionState;

    public InterchangeableCommands(BooleanSupplier condition, Command auxiliaryCommand, Command defaultCommand) {
        this.condition = condition;
        this.auxiliaryCommand = auxiliaryCommand;
        this.defaultCommand = defaultCommand;

        requireUngrouped(auxiliaryCommand, defaultCommand);
    }

    @Override
    public void initialize() {
        lastConditionState = condition.getAsBoolean();
    }

    @Override
    public void execute() {
        boolean currentConditionState = condition.getAsBoolean();

        if (currentConditionState) {
            if (lastConditionState != currentConditionState) {
                defaultCommand.end(true);
                auxiliaryCommand.initialize();
            }
            if (!auxiliaryCommand.isFinished()) {
                auxiliaryCommand.execute();
            }
        } else {
            if (lastConditionState != currentConditionState) {
                auxiliaryCommand.end(true);
                defaultCommand.initialize();
            }
            if (!defaultCommand.isFinished()) {
                defaultCommand.execute();
            }
        }

        lastConditionState = currentConditionState;
    }

    @Override
    public void end(boolean interrupted) {
        auxiliaryCommand.end(interrupted);
        defaultCommand.end(interrupted);
    }
}
