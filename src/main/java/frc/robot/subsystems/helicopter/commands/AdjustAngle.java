package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.helicopter.Helicopter;

public class AdjustAngle extends CommandBase {
    private final Helicopter helicopter;

    public AdjustAngle(Helicopter helicopter) {
        this.helicopter = helicopter;

        addRequirements(helicopter);
    }

    @Override
    public void execute() {
        helicopter.setAngleZero();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(helicopter.getAbsolutePosition()) <= Constants.Helicopter.ZERO_POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
    }
}
