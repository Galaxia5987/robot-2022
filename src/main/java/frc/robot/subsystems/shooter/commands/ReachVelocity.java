package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.DriverStation;
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
                if (RobotContainer.hasTarget.getAsBoolean() && !RobotContainer.playWithoutVision && !DriverStation.isAutonomous()) {
                    if (Math.abs(RobotContainer.Suppliers.yawSupplier.getAsDouble()) < 10) {
                        shooter.setVelocity(RobotContainer.setpointVelocity - 100);
                    }
                } else {
                    shooter.setVelocity(RobotContainer.setpointVelocity);
                }
            }
        } else {
            shooter.setPower(0);
        }
    }
}
