package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class AdjustAngle extends CommandBase {

    private final Climber climber;

    public AdjustAngle(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setAngleZero();
    }

    @Override
    public boolean isFinished() {
        if (climber.getPosition() == 0) {
            return true;
        }
        else return false;
    }
}
