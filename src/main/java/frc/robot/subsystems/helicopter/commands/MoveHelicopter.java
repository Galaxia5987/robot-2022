package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.helicopter.Helicopter;

public class AutoHelicopter extends CommandBase {
    private final Helicopter helicopter;
    private final double desiredRad;

    public AutoHelicopter(Helicopter helicopter, double desiredRad) {
        this.helicopter = helicopter;
        this.desiredRad = desiredRad;

        addRequirements(helicopter);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        helicopter.setPosition(desiredRad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
    }
}
