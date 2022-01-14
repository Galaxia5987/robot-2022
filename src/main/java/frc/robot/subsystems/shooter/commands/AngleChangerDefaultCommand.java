package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.shooter.AngleChanger;

public class AngleChangerDefaultCommand extends CommandBase {
    private final AngleChanger angleChanger = AngleChanger.getINSTANCE();
    private JoystickButton y;
    private boolean input;
    private boolean lastInput;

    public AngleChangerDefaultCommand(JoystickButton y) {
        this.y = y;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        input = y.get();
        if(input && !lastInput){
            angleChanger.changeAngle();
        }
        lastInput = input;
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
