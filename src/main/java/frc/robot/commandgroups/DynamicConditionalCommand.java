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

    /**
     * For logic documentation see link below.
     *
     * @see <https://docs.google.com/document/d/1DLzhcw4T3GlePEOU7x3P3x5kkEZ1wxjmsLIv5DoKhzg/edit?usp=sharing>
     */
    @Override
    public void execute() {
        boolean currentConditionState = condition.getAsBoolean();
        var toggle = !lastConditionState && currentConditionState;
        var output = toggle ^ currentConditionState;

        if (output) {
            onFalseCommand.cancel();
            onTrueCommand.schedule();
        } else {
            onTrueCommand.cancel();
            onFalseCommand.schedule();
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
        return onFalseCommand.isFinished() && onTrueCommand.isFinished();
    }
}
