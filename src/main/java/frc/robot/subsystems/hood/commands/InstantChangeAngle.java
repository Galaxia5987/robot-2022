package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.hood.Hood;

public class InstantChangeAngle extends InstantCommand {
    private final Hood hood;
    private final Hood.Mode mode;

    public InstantChangeAngle(Hood hood, Hood.Mode mode) {
        this.hood = hood;
        this.mode = mode;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setSolenoid(mode);
    }
}
