package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.helicopter.Helicopter;


public class ToggleHelicopterStopper extends InstantCommand {
    private final Helicopter helicopter;

    public ToggleHelicopterStopper(Helicopter helicopter) {
        this.helicopter = helicopter;

        addRequirements(helicopter);
    }

    @Override
    public void initialize() {
        helicopter.toggleStopper();
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
    }
}
