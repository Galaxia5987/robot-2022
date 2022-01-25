package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.climber.Climber;


public class ClimberStop extends InstantCommand {
    private final Climber climber;

    public ClimberStop(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (Robot.isReal() && !Robot.debug) {
            boolean isReady = Timer.getFPGATimestamp() > Constants.Climber.AUTONOMY_TIME;
            if (isReady) {
                climber.toggleStopper();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
