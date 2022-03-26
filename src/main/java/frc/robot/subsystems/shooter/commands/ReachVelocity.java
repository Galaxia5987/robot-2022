package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.Shooter;

public class ReachVelocity extends CommandBase {
    private final Shooter shooter;

    public ReachVelocity(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (RobotContainer.warmUpShooting) {
            if (RobotContainer.hardCodedVelocity) {
                shooter.setVelocity(RobotContainer.hardCodedVelocityValue);
            } else {
                shooter.setVelocity(RobotContainer.setpointVelocity);
            }
        } else {
            shooter.setPower(0);
        }
    }
}
