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

        if (currentConditionState) {
            if (lastConditionState != currentConditionState) {
                onFalseCommand.end(true);
                onTrueCommand.initialize();
            }
            if (!onTrueCommand.isFinished()) {
                onTrueCommand.execute();
            }
        } else {
            if (lastConditionState != currentConditionState) {
                onTrueCommand.end(true);
                onFalseCommand.initialize();
            }
            if (!onFalseCommand.isFinished()) {
                onFalseCommand.execute();
            }
        }

        lastConditionState = currentConditionState;
    }

    @Override
    public void end(boolean interrupted) {
        onTrueCommand.end(interrupted);
        onFalseCommand.end(interrupted);
    }
}
