package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

public class OverridableCommand extends CommandBase {
    private final BooleanSupplier override;
    private final Command defaultCommand;
    private final Command auxiliaryCommand;
    private boolean lastInput;

    public OverridableCommand(BooleanSupplier override, Command defaultCommand, Command auxiliaryCommand) {
        this.override = override;
        this.defaultCommand = defaultCommand;
        this.auxiliaryCommand = auxiliaryCommand;

        requireUngrouped(defaultCommand, auxiliaryCommand);
    }

    @Override
    public void initialize() {
        lastInput = override.getAsBoolean();
    }

    @Override
    public void execute() {
        boolean currentInput = override.getAsBoolean();
        Command currentCommand = currentInput ? defaultCommand : auxiliaryCommand;
        Command lastCommand = lastInput ? defaultCommand : auxiliaryCommand;

        if (currentInput != lastInput) {
            lastCommand.end(true);
            currentCommand.initialize();
        } else {
            if(lastCommand.isFinished()) {
                lastCommand.end(false);
            } else {
                lastCommand.execute();
            }
        }

        lastInput = currentInput;
    }

    @Override
    public void end(boolean interrupted) {
        defaultCommand.end(interrupted);
        auxiliaryCommand.end(interrupted);
    }
}
