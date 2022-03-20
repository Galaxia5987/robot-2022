package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.helicopter.Helicopter;

public class MoveHelicopter extends CommandBase {
    private final Helicopter helicopter;
    private final double desiredRad;

    public MoveHelicopter(Helicopter helicopter, double desiredRad) {
        this.helicopter = helicopter;
        this.desiredRad = desiredRad;

        addRequirements(helicopter);
    }

    @Override
    public void initialize() {
        helicopter.setStopperMode(false);
    }

    @Override
    public void execute() {
        helicopter.setAbsolutePosition(new Rotation2d(desiredRad));
    }


    @Override
    public boolean isFinished() {
        return Math.abs(helicopter.getAbsolutePosition() - desiredRad) <= Constants.Helicopter.POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
    }
}
