package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.Hood;

public class HoodCommand extends CommandBase {
    private final Hood hood;
    private final Timer timer = new Timer();
    private Hood.Mode mode = Hood.Mode.ShortDistance;

    public HoodCommand(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        if (RobotContainer.hardCodedVelocity) {
            mode = RobotContainer.hardCodedDistance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
        } else {
            if (RobotContainer.cachedHasTarget) {
                mode = RobotContainer.cachedDistanceForHood < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
            } else {
                mode = RobotContainer.odometryCachedSetpoint < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
            }
        }
    }

    @Override
    public void execute() {
        hood.setSolenoid(mode);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
