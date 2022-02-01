package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        return Math.abs(climber.getAbsolutePosition()) <= Constants.Climber.ZERO_POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
