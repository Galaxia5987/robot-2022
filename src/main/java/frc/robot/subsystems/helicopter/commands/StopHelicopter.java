package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.helicopter.Helicopter;


public class StopHelicopter extends InstantCommand {
    private final Helicopter helicopter;

    public StopHelicopter(Helicopter helicopter) {
        this.helicopter = helicopter;

        addRequirements(helicopter);
    }

    @Override
    public void initialize() {
        if (Robot.isReal() && !Robot.debug) {
            boolean notReady = DriverStation.isAutonomous();
            if (!notReady) {
                helicopter.toggleStopper();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
    }
}
