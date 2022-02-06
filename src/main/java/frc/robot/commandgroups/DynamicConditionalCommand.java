package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

public class DynamicConditionalCommand extends CommandBase {
    private final BooleanSupplier condition;
    private final Command onTrueCommand;
    private final Command onFalseCommand;
    private boolean lastConditionState;

    public DynamicConditionalCommand(BooleanSupplier condition, Command onTrueCommand, Command onFalseCommand) {
        this.condition = condition;
        this.onTrueCommand = onTrueCommand;
        this.onFalseCommand = onFalseCommand;

        requireUngrouped(onTrueCommand, onFalseCommand);
    }

    @Override
    public void initialize() {
        lastConditionState = condition.getAsBoolean();
    }

    @Override
    public void execute() {
        boolean currentConditionState = condition.getAsBoolean();
        var change = lastConditionState & !currentConditionState;
        currentConditionState ^= change;

        if (currentConditionState) {
            if (onFalseCommand.isScheduled()) {
                onFalseCommand.end(true);
            }
            if (!lastConditionState) {
                onTrueCommand.schedule();
            }
        } else {
            if (onTrueCommand.isScheduled()) {
                onTrueCommand.end(true);
            }
            if (lastConditionState) {
                onFalseCommand.schedule();
            }
        }

        lastConditionState = currentConditionState;
    }

    @Override
    public void end(boolean interrupted) {
        onTrueCommand.end(interrupted);
        onFalseCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return onFalseCommand.isFinished() & onTrueCommand.isFinished();
    }
}
